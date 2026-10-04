#!/usr/bin/env python3
"""Offline package verifier and explicit TLS/HMAC uploader. Never auto-reboots.

CA validation + hostname/SAN verification + independently provisioned certificate
pin are all required. The manifest is provenance metadata, not a digital signature.
Firmware authorization is the existing nonce-bound HMAC protocol.
"""
import argparse
import hashlib
import hmac
import http.client
from pathlib import Path
import re
import ssl
import struct
import sys
import urllib.parse

import build_manifest
import matdog_layout

FQBN = ('esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,'
        'CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,'
        'DebugLevel=none,PSRAM=opi')


def validate_package(manifest_path, expected_source, profile='ROBOT_POWERED', ingest='0'):
    manifest_path = Path(manifest_path)
    manifest = build_manifest.parse_manifest(manifest_path.read_text())
    if not re.fullmatch(r'[0-9a-f]{40}', expected_source):
        raise ValueError('Expected source must be an independently checked full commit SHA')
    if manifest.get("APPLICATION_BINARY") != "MATDOG_Controller.ino.bin":
        raise ValueError("Unexpected image name")
    binary = manifest_path.parent / manifest["APPLICATION_BINARY"]
    if binary.name != 'MATDOG_Controller.ino.bin':
        raise ValueError('Unexpected image name')
    image = binary.read_bytes()
    table = build_manifest.partition_table_path_for(binary)
    verdict = build_manifest.verify_manifest(
        manifest, head_commit=expected_source, expected_fqbn=FQBN, tree_state='CLEAN',
        binary_exists=True, binary_size=len(image), binary_sha256=hashlib.sha256(image).hexdigest(),
        requested_profile=profile, requested_ota_ingest=ingest,
        build_partition_table_sha256=build_manifest.sha256_file(table),
        binary_embeds_layout_id=matdog_layout.LAYOUT_MARKER.encode() in image)
    if not verdict.ok:
        raise ValueError(f'Package refused: {verdict.reason}: {verdict.detail}')
    if len(image) < 24 or image[0] != 0xe9 or struct.unpack_from('<H', image, 12)[0] != 9:
        raise ValueError('Not an ESP32-S3 application image')
    build = manifest['BUILD_ID']
    if build != expected_source[:12] or build.encode() not in image:
        raise ValueError('Clean source/build identity mismatch')
    return manifest, binary


def signed_payload(nonce, manifest):
    if len(nonce) != 16:
        raise ValueError('Malformed nonce')
    build = manifest['BUILD_ID'].encode('ascii')
    if len(build) > 23:
        raise ValueError('Build ID too long')
    return (nonce + struct.pack('>II', 1, int(manifest['APPLICATION_SIZE'])) +
            bytes.fromhex(manifest['APPLICATION_SHA256']) + build.ljust(24, b'\0'))


class PinnedConnection(http.client.HTTPSConnection):
    def __init__(self, hostname, context, pin):
        if context.verify_mode != ssl.CERT_REQUIRED or not context.check_hostname:
            raise ValueError('CA and hostname verification are mandatory')
        context.hostname_checks_common_name = False  # Require SAN; no legacy CN fallback.
        super().__init__(hostname, port=443, context=context, timeout=10)
        self.pin = pin.lower()

    def connect(self):
        super().connect()  # CA and hostname/SAN are always verified
        actual = hashlib.sha256(self.sock.getpeercert(binary_form=True)).hexdigest()
        if not hmac.compare_digest(actual, self.pin):
            self.close()
            raise ValueError('Device certificate pin mismatch')


def tls_connection(url, ca, pin):
    target = urllib.parse.urlsplit(url)
    if (target.scheme != 'https' or not target.hostname or target.username or target.password or
            target.query or target.fragment or target.path not in ('', '/') or target.port not in (None, 443)):
        raise ValueError('Use an HTTPS device origin on port 443, without credentials/path')
    if not re.fullmatch(r'[0-9a-fA-F]{64}', pin):
        raise ValueError('Certificate pin must be DER SHA256 from a trusted physical channel')
    context = ssl.create_default_context(cafile=str(ca))
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    connection = PinnedConnection(target.hostname, context, pin)
    connection.connect()  # Also repeated (and pinned) for any new socket.
    return connection


def bounded_response(connection, expected_status=200, limit=8192):
    response = connection.getresponse()
    body = response.read(limit + 1)
    if len(body) > limit:
        raise ValueError('Oversized device response')
    if response.status != expected_status:
        # Do not print attacker-supplied bodies or secret-bearing headers.
        raise ValueError(f'Device refused request: HTTP {response.status}')
    return body


def upload(args, manifest, binary):
    if not args.secret_file or not args.url or not args.ca_cert or not args.cert_sha256:
        raise ValueError('Upload requires URL, CA, pin and a private HMAC secret file')
    secret = Path(args.secret_file).read_bytes().rstrip(b'\r\n')
    if len(secret) < 32:
        raise ValueError('Use at least 32 bytes of high-entropy HMAC secret')
    # Each retry performs a new challenge and restarts the entire image.
    # Automatic replay/resume after a transport failure is deliberately absent.
    connection = tls_connection(args.url, args.ca_cert, args.cert_sha256)
    try:
        connection.request('GET', '/ota/challenge')
        challenge = bounded_response(connection, limit=32)
        if len(challenge) != 32 or not re.fullmatch(b'[0-9a-fA-F]{32}', challenge):
            raise ValueError('Invalid challenge')
        nonce = bytes.fromhex(challenge.decode('ascii'))
        signature = hmac.new(secret, signed_payload(nonce, manifest), hashlib.sha256).hexdigest()
        size = int(manifest['APPLICATION_SIZE'])
        connection.putrequest('POST', '/ota/update')
        headers = {'Content-Type': 'application/octet-stream', 'Content-Length': str(size),
                   'X-Ota-Nonce': nonce.hex(), 'X-Ota-Signature': signature,
                   'X-Ota-Sha256': manifest['APPLICATION_SHA256'],
                   'X-Ota-Build-Id': manifest['BUILD_ID']}
        for name, value in headers.items():
            connection.putheader(name, value)
        connection.endheaders()
        sent = 0
        with binary.open('rb') as stream:
            for block in iter(lambda: stream.read(1024), b''):
                connection.send(block)
                sent += len(block)
                if sent == size or sent % 65536 == 0:
                    print(f'{sent}/{size} bytes sent', flush=True)
        result = bounded_response(connection, limit=128)
        if result != b'COMMITTED_PENDING_REBOOT':
            raise ValueError('Commit was not confirmed; inspect USB/status before a fresh session')
        print('COMMITTED_PENDING_REBOOT. Remote reboot is BLOCKED in V1; use a separately authorized recovery procedure.')
    finally:
        connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--expected-source', required=True)
    parser.add_argument('--expected-profile', choices=['USB_ONLY', 'ROBOT_POWERED'], default='ROBOT_POWERED')
    parser.add_argument('--expected-ingest', choices=['0', '1'], default='0')
    parser.add_argument('--upload', action='store_true', help='Explicit real device write; never use during offline development')
    parser.add_argument('--url')
    parser.add_argument('--ca-cert', type=Path)
    parser.add_argument('--cert-sha256')
    parser.add_argument('--secret-file', type=Path)
    args = parser.parse_args()
    try:
        manifest, binary = validate_package(args.manifest, args.expected_source,
                                            args.expected_profile, args.expected_ingest)
        print(f'PACKAGE_PASS build={manifest["BUILD_ID"]} profile={manifest["HARDWARE_PROFILE"]} '
              f'ingest={manifest["OTA_INGEST_ENABLED"]} size={manifest["APPLICATION_SIZE"]}')
        if args.upload:
            upload(args, manifest, binary)
    except (OSError, ValueError, ssl.SSLError, http.client.HTTPException) as exc:
        print(f'REFUSED: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
