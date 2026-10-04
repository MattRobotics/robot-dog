import hashlib
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import ssl
import subprocess
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import ota_tls_client as client
import build_manifest
import matdog_layout

class ClientTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.sha='a'*40
        header=bytearray(24);header[0]=0xe9;struct.pack_into('<H',header,12,9)
        self.image=bytes(header)+matdog_layout.LAYOUT_MARKER.encode()+self.sha[:12].encode()
        self.binary=self.root/'MATDOG_Controller.ino.bin';self.binary.write_bytes(self.image)
        # Use the canonical pinned table bytes from layout generator/fixture.
        import subprocess
        generator=Path.home()/'.arduino15/packages/esp32/hardware/esp32/3.3.11/tools/gen_esp32part.py'
        source=Path(__file__).resolve().parents[2]/'partitions.csv'
        table_path=self.root/'table.bin'
        subprocess.run([sys.executable,str(generator),'-q',str(source),str(table_path)],check=True,capture_output=True)
        table=table_path.read_bytes()
        self.table=client.build_manifest.partition_table_path_for(self.binary);self.table.write_bytes(table)
        self.fields=dict(source_commit=self.sha,build_id=self.sha[:12],source_state='CLEAN',profile='ROBOT_POWERED',ota_ingest_enabled='0',fqbn=client.FQBN,application_binary=self.binary.name,application_size=len(self.image),application_sha256=hashlib.sha256(self.image).hexdigest(),layout_id=matdog_layout.LAYOUT_ID,partition_table_sha256=matdog_layout.EXPECTED_TABLE_SHA256,app_partition_size=matdog_layout.APP_SLOT_SIZE)
        self.manifest=self.root/'matdog_build_manifest.txt'
        self.write()
    def write(self):
        self.manifest.write_text(build_manifest.render_manifest(**self.fields))
    def test_positive_package(self):
        m,b=client.validate_package(self.manifest,self.sha)
        self.assertEqual(b,self.binary)
        self.assertEqual(m["HARDWARE_PROFILE"],"ROBOT_POWERED")
    def test_payload(self):
        m=build_manifest.parse_manifest(self.manifest.read_text());p=client.signed_payload(bytes(range(16)),m)
        self.assertEqual(len(p),80)
        self.assertEqual(p[:16],bytes(range(16)))
        self.assertEqual(p[16:24],struct.pack('>II',1,len(self.image)))
        self.assertEqual(p[56:68],b'a'*12)
        self.assertEqual(p[68:],bytes(12))
        with self.assertRaises(ValueError):client.signed_payload(b'x',m)
    def test_source_gate(self):
        with self.assertRaises(ValueError):client.validate_package(self.manifest,'b'*40)
        with self.assertRaises(ValueError):client.validate_package(self.manifest,'main')
    def test_dirty_gate(self):
        self.fields['source_state']='DIRTY';self.write()
        with self.assertRaises(ValueError):client.validate_package(self.manifest,self.sha)
    def test_hash_gate(self):
        self.binary.write_bytes(self.image+b'wrong')
        with self.assertRaises(ValueError):client.validate_package(self.manifest,self.sha)
    def test_http_refused_before_connection(self):
        for url in ['http://192.168.1.53','https://user:pass@localhost','https://localhost:80','https://localhost/path']:
            with self.assertRaises(ValueError):client.tls_connection(url,'missing.pem','0'*64)
    def test_no_automatic_reboot_resume_or_insecure_tls(self):
        source=Path(client.__file__).read_text()
        self.assertNotIn('CERT_NONE',source)
        self.assertNotIn('check_hostname = False',source)
        self.assertNotIn("request('POST', '/ota/reboot'",source)
        self.assertNotIn('Range',source)

class TlsPeerTest(unittest.TestCase):
    """Real loopback TLS sockets only. No device address or firmware ingest."""
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory()
        root=Path(cls.temp.name)
        cls.cert=root/'cert.pem';cls.key=root/'key.pem'
        subprocess.run(['openssl','req','-x509','-newkey','rsa:2048','-nodes',
                        '-keyout',str(cls.key),'-out',str(cls.cert),'-days','1',
                        '-subj','/CN=localhost','-addext','subjectAltName=DNS:localhost'],
                       check=True,capture_output=True)
        cls.other=root/'other.pem'
        subprocess.run(['openssl','req','-x509','-newkey','rsa:2048','-nodes',
                        '-keyout',str(root/'other.key'),'-out',str(cls.other),'-days','1',
                        '-subj','/CN=other'],check=True,capture_output=True)
        der=ssl.PEM_cert_to_DER_cert(cls.cert.read_text())
        cls.pin=hashlib.sha256(der).hexdigest()
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200);self.send_header('Content-Length','2')
                self.end_headers();self.wfile.write(b'OK')
            def log_message(self,*args):pass
        cls.server=HTTPServer(('127.0.0.1',0),Handler)
        context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(str(cls.cert),str(cls.key))
        cls.server.socket=context.wrap_socket(cls.server.socket,server_side=True)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True)
        cls.thread.start()
        cls.no_san=root/'no-san.pem'
        no_san_key=root/'no-san.key'
        subprocess.run(['openssl','req','-x509','-newkey','rsa:2048','-nodes',
                        '-keyout',str(no_san_key),'-out',str(cls.no_san),'-days','1',
                        '-subj','/CN=localhost'],check=True,capture_output=True)
        cls.no_san_server=HTTPServer(('127.0.0.1',0),Handler)
        no_san_context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        no_san_context.load_cert_chain(str(cls.no_san),str(no_san_key))
        cls.no_san_server.socket=no_san_context.wrap_socket(cls.no_san_server.socket,server_side=True)
        cls.no_san_thread=threading.Thread(target=cls.no_san_server.serve_forever,daemon=True)
        cls.no_san_thread.start()
    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close();cls.thread.join()
        cls.no_san_server.shutdown();cls.no_san_server.server_close();cls.no_san_thread.join()
        cls.temp.cleanup()
    def connect(self,hostname='localhost',ca=None,pin=None):
        context=ssl.create_default_context(cafile=str(ca or self.cert))
        connection=client.PinnedConnection(hostname,context,pin or self.pin)
        # Only the test uses an ephemeral loopback port. Production is fixed 443.
        connection.port=self.server.server_port
        self.addCleanup(connection.close)
        connection.connect()
        return connection
    def test_ca_san_and_pin_pass_and_reconnect_is_rechecked(self):
        connection=self.connect()
        connection.request('GET','/');self.assertEqual(client.bounded_response(connection),b'OK')
        connection.close();connection.pin='0'*64
        with self.assertRaisesRegex(ValueError,'pin mismatch'):connection.request('GET','/')
    def test_wrong_pin_refused(self):
        with self.assertRaisesRegex(ValueError,'pin mismatch'):self.connect(pin='0'*64)
    def test_untrusted_ca_refused(self):
        with self.assertRaises(ssl.SSLCertVerificationError):self.connect(ca=self.other)
    def test_wrong_san_refused(self):
        with self.assertRaises(ssl.SSLCertVerificationError):self.connect(hostname='127.0.0.1')
    def test_common_name_without_san_refused(self):
        context=ssl.create_default_context(cafile=str(self.no_san))
        pin=hashlib.sha256(ssl.PEM_cert_to_DER_cert(self.no_san.read_text())).hexdigest()
        connection=client.PinnedConnection('localhost',context,pin)
        connection.port=self.no_san_server.server_port
        self.addCleanup(connection.close)
        with self.assertRaises(ssl.SSLCertVerificationError):connection.connect()
    def test_insecure_context_refused(self):
        context=ssl.create_default_context(cafile=str(self.cert))
        context.check_hostname=False
        with self.assertRaisesRegex(ValueError,'mandatory'):
            client.PinnedConnection('localhost',context,self.pin)

if __name__=='__main__':unittest.main()
