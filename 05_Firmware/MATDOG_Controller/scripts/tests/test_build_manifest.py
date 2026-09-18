#!/usr/bin/env python3
"""Offline tests for build_manifest.py — no device I/O, no flash writes, no
hardware required. Run directly:

    python3 scripts/tests/test_build_manifest.py

Covers the G2 pre-G3 hardening (review Finding 1): build.sh can produce a
USB_ONLY or a ROBOT_POWERED image from the SAME commit at the SAME path, so
the pre-existing "binary embeds the current build id" gate cannot tell them
apart. Every refusal path of the manifest gate that closes that hole is
asserted here by its exact reason code, so a regression cannot be mistaken
for a different failure.

The authorization matrix under test:

    manifest USB_ONLY      + requested USB_ONLY (default)   -> ALLOW
    manifest ROBOT_POWERED + requested ROBOT_POWERED        -> ALLOW
    manifest ROBOT_POWERED + default (no authorization)     -> REFUSE
    manifest USB_ONLY      + requested ROBOT_POWERED        -> REFUSE
    manifest profile unknown/missing                        -> REFUSE
    manifest FQBN != flasher's pinned FQBN                  -> REFUSE
    binary size or sha256 mismatch                          -> REFUSE
    manifest commit != HEAD                                 -> REFUSE
    tree dirty (at build time or now)                       -> REFUSE
    manifest missing / unparseable / incomplete             -> REFUSE
"""
import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build_manifest import (  # noqa: E402
    DEFAULT_FLASH_PROFILE,
    KNOWN_PROFILES,
    MANIFEST_VERSION,
    Refusal,
    load_manifest,
    main,
    parse_manifest,
    render_manifest,
    sha256_file,
    verify_manifest,
)

HEAD = "34afbc7808e276d7483de9f2c817750683a65088"
OTHER_COMMIT = "1a8c5bc3ced4f49ea36902526f232b2785a7ab70"
SHA = "6e6d92f898dbe95000b53dbb252c7eb5d3deaa9a4b161e2b1934436a76b29364"

# The real pinned FQBN used by scripts/build.sh and scripts/flash_app_only.sh.
# Kept verbatim so these tests exercise the actual string, not a stand-in.
CANONICAL_FQBN = (
    "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,"
    "CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,"
    "DebugLevel=none,PSRAM=opi"
)


def manifest(**overrides):
    base = {
        "MATDOG_MANIFEST_VERSION": MANIFEST_VERSION,
        "SOURCE_COMMIT": HEAD,
        "BUILD_ID": "34afbc7808e2",
        "SOURCE_STATE": "CLEAN",
        "HARDWARE_PROFILE": "USB_ONLY",
        "FQBN": CANONICAL_FQBN,
        "APPLICATION_BINARY": "MATDOG_Controller.ino.bin",
        "APPLICATION_SIZE": "387164",
        "APPLICATION_SHA256": SHA,
    }
    base.update(overrides)
    return base


def verify(m=None, *, head=HEAD, expected_fqbn=CANONICAL_FQBN, tree_state="CLEAN",
           binary_exists=True, binary_size=387164, binary_sha256=SHA,
           requested_profile="USB_ONLY"):
    return verify_manifest(
        manifest() if m is None else m,
        head_commit=head,
        expected_fqbn=expected_fqbn,
        tree_state=tree_state,
        binary_exists=binary_exists,
        binary_size=binary_size,
        binary_sha256=binary_sha256,
        requested_profile=requested_profile,
    )


class TestHappyPath(unittest.TestCase):
    def test_usb_only_manifest_with_default_request_is_allowed(self):
        v = verify(requested_profile=DEFAULT_FLASH_PROFILE)
        self.assertTrue(v.ok, v.detail)
        self.assertEqual(v.profile, "USB_ONLY")

    def test_robot_powered_manifest_with_explicit_authorization_is_allowed(self):
        v = verify(manifest(HARDWARE_PROFILE="ROBOT_POWERED"),
                   requested_profile="ROBOT_POWERED")
        self.assertTrue(v.ok, v.detail)
        self.assertEqual(v.profile, "ROBOT_POWERED")

    def test_default_flash_profile_is_the_backwards_safe_one(self):
        self.assertEqual(DEFAULT_FLASH_PROFILE, "USB_ONLY")
        self.assertIn("USB_ONLY", KNOWN_PROFILES)
        self.assertIn("ROBOT_POWERED", KNOWN_PROFILES)


class TestProfileAuthorization(unittest.TestCase):
    def test_robot_powered_manifest_without_authorization_refuses(self):
        # THE case this gate exists for: a powered image left in the shared
        # build directory must never be flashed by an unqualified invocation.
        v = verify(manifest(HARDWARE_PROFILE="ROBOT_POWERED"),
                   requested_profile=DEFAULT_FLASH_PROFILE)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.PROFILE_MISMATCH)
        self.assertIn("MATDOG_FLASH_PROFILE=ROBOT_POWERED", v.detail)

    def test_usb_only_manifest_when_powered_was_requested_refuses(self):
        # The other direction: operator intends a powered flash but the
        # build directory holds a stale USB_ONLY binary.
        v = verify(manifest(HARDWARE_PROFILE="USB_ONLY"),
                   requested_profile="ROBOT_POWERED")
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.PROFILE_MISMATCH)

    def test_unknown_manifest_profile_refuses(self):
        for bad in ("ROBOT-POWERED", "usb_only", "POWERED", "TRUE", "1"):
            v = verify(manifest(HARDWARE_PROFILE=bad))
            self.assertFalse(v.ok, bad)
            self.assertEqual(v.reason, Refusal.PROFILE_UNKNOWN, bad)

    def test_empty_manifest_profile_refuses_as_incomplete(self):
        v = verify(manifest(HARDWARE_PROFILE=""))
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.MANIFEST_INCOMPLETE)

    def test_unknown_requested_profile_refuses(self):
        v = verify(requested_profile="ANYTHING_ELSE")
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.REQUESTED_PROFILE_UNKNOWN)

    def test_matching_profiles_are_never_inferred_from_one_side(self):
        # Both sides must be stated; the verifier never derives one from
        # the other.
        for m_profile in KNOWN_PROFILES:
            for r_profile in KNOWN_PROFILES:
                v = verify(manifest(HARDWARE_PROFILE=m_profile),
                           requested_profile=r_profile)
                self.assertEqual(v.ok, m_profile == r_profile,
                                 f"{m_profile} vs {r_profile}")


class TestBinaryBinding(unittest.TestCase):
    def test_size_mismatch_refuses(self):
        v = verify(binary_size=387632)  # e.g. the ROBOT_POWERED build's size
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.BINARY_SIZE_MISMATCH)

    def test_sha256_mismatch_refuses(self):
        v = verify(binary_sha256="0" * 64)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.BINARY_SHA256_MISMATCH)

    def test_sha256_comparison_is_case_insensitive(self):
        v = verify(binary_sha256=SHA.upper())
        self.assertTrue(v.ok, v.detail)

    def test_missing_binary_refuses(self):
        v = verify(binary_exists=False, binary_size=0, binary_sha256="")
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.BINARY_MISSING)

    def test_same_size_different_bytes_still_refuses(self):
        # Two profiles could coincidentally produce equal-sized binaries;
        # the digest is what actually binds the bytes.
        v = verify(binary_size=387164, binary_sha256="a" * 64)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.BINARY_SHA256_MISMATCH)


class TestFqbnBinding(unittest.TestCase):
    """The manifest recorded FQBN from the start, but nothing compared it.
    The right source compiled with the wrong toolchain configuration is
    still the wrong artifact: the FQBN carries the partition scheme, flash
    size/mode, PSRAM mode, USB/CDC mode and CPU frequency."""

    def test_canonical_fqbn_passes(self):
        v = verify(expected_fqbn=CANONICAL_FQBN)
        self.assertTrue(v.ok, v.detail)

    def test_different_fqbn_refuses(self):
        v = verify(expected_fqbn="esp32:esp32:esp32s3:FlashSize=8M")
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.FQBN_MISMATCH)

    def test_refusal_detail_names_both_values(self):
        other = "esp32:esp32:esp32s3:FlashSize=8M"
        v = verify(expected_fqbn=other)
        self.assertIn(other, v.detail)
        self.assertIn(CANONICAL_FQBN, v.detail)

    def test_single_option_difference_refuses(self):
        # The realistic failure: one option drifts. A partition-scheme
        # change silently relocates the application partition.
        for wrong in (
                CANONICAL_FQBN.replace("PartitionScheme=app3M_fat9M_16MB",
                                       "PartitionScheme=default"),
                CANONICAL_FQBN.replace("FlashSize=16M", "FlashSize=8M"),
                CANONICAL_FQBN.replace("PSRAM=opi", "PSRAM=disabled"),
                CANONICAL_FQBN.replace("CPUFreq=240", "CPUFreq=160"),
                CANONICAL_FQBN.replace("FlashMode=qio", "FlashMode=dio"),
                CANONICAL_FQBN.replace("CDCOnBoot=cdc", "CDCOnBoot=default")):
            v = verify(expected_fqbn=wrong)
            self.assertFalse(v.ok, wrong)
            self.assertEqual(v.reason, Refusal.FQBN_MISMATCH, wrong)

    def test_comparison_is_exact_not_substring(self):
        # A prefix, a suffix and a reordering must all refuse: this is an
        # exact equality check, never a pattern match.
        for wrong in (CANONICAL_FQBN[:-4],
                      CANONICAL_FQBN + ",ExtraOption=1",
                      "esp32:esp32:esp32s3",
                      CANONICAL_FQBN.upper()):
            v = verify(expected_fqbn=wrong)
            self.assertFalse(v.ok, wrong)
            self.assertEqual(v.reason, Refusal.FQBN_MISMATCH, wrong)

    def test_modified_manifest_fqbn_with_identical_everything_else_refuses(self):
        # Same commit, same profile, same binary size and digest — only the
        # manifest's recorded FQBN was altered.
        v = verify(manifest(FQBN="esp32:esp32:esp32s3:PartitionScheme=default"))
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.FQBN_MISMATCH)

    def test_empty_fqbn_is_incomplete_not_fqbn_mismatch(self):
        # An empty field is a broken manifest, reported as such upstream of
        # the comparison.
        v = verify(manifest(FQBN=""))
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.MANIFEST_INCOMPLETE)

    def test_missing_fqbn_key_is_incomplete(self):
        m = manifest()
        del m["FQBN"]
        v = verify(m)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.MANIFEST_INCOMPLETE)

    def test_fqbn_checked_before_tree_state_and_binary(self):
        # Ordering: a wrong build configuration is reported as such even
        # when later checks would also fail, so the operator sees the most
        # upstream cause.
        v = verify(expected_fqbn="esp32:esp32:esp32s3:FlashSize=8M",
                   tree_state="DIRTY", binary_sha256="0" * 64)
        self.assertEqual(v.reason, Refusal.FQBN_MISMATCH)

    def test_commit_mismatch_still_outranks_fqbn(self):
        v = verify(manifest(SOURCE_COMMIT=OTHER_COMMIT),
                   expected_fqbn="esp32:esp32:esp32s3:FlashSize=8M")
        self.assertEqual(v.reason, Refusal.SOURCE_COMMIT_MISMATCH)

    def test_both_profiles_pass_with_canonical_fqbn(self):
        # A valid artifact of either profile, under its own authorization,
        # passes once the FQBN matches.
        for profile in KNOWN_PROFILES:
            v = verify(manifest(HARDWARE_PROFILE=profile),
                       requested_profile=profile, expected_fqbn=CANONICAL_FQBN)
            self.assertTrue(v.ok, f"{profile}: {v.detail}")
            self.assertEqual(v.profile, profile)

    def test_both_profiles_refuse_with_wrong_fqbn(self):
        for profile in KNOWN_PROFILES:
            v = verify(manifest(HARDWARE_PROFILE=profile),
                       requested_profile=profile,
                       expected_fqbn="esp32:esp32:esp32s3:FlashSize=8M")
            self.assertFalse(v.ok, profile)
            self.assertEqual(v.reason, Refusal.FQBN_MISMATCH, profile)


class TestSourceBinding(unittest.TestCase):
    def test_commit_mismatch_refuses(self):
        v = verify(manifest(SOURCE_COMMIT=OTHER_COMMIT))
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.SOURCE_COMMIT_MISMATCH)

    def test_manifest_built_from_dirty_tree_refuses(self):
        v = verify(manifest(SOURCE_STATE="DIRTY"))
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.TREE_NOT_CLEAN)

    def test_manifest_built_without_git_refuses(self):
        v = verify(manifest(SOURCE_STATE="NO_GIT"))
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.TREE_NOT_CLEAN)

    def test_currently_dirty_tree_refuses(self):
        v = verify(tree_state="DIRTY")
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.TREE_NOT_CLEAN)

    def test_built_clean_then_edited_refuses(self):
        # The manifest says CLEAN, but the tree has moved since: the live
        # state must be checked too, not just the recorded one.
        v = verify(manifest(SOURCE_STATE="CLEAN"), tree_state="DIRTY")
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.TREE_NOT_CLEAN)


class TestManifestIntegrity(unittest.TestCase):
    def test_missing_manifest_refuses(self):
        v = verify_manifest(None, head_commit=HEAD, expected_fqbn=CANONICAL_FQBN,
                            tree_state="CLEAN", binary_exists=True,
                            binary_size=387164, binary_sha256=SHA,
                            requested_profile="USB_ONLY")
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.MANIFEST_MISSING)

    def test_incomplete_manifest_refuses(self):
        for key in ("SOURCE_COMMIT", "HARDWARE_PROFILE", "APPLICATION_SHA256",
                    "APPLICATION_SIZE", "SOURCE_STATE", "BUILD_ID", "FQBN",
                    "APPLICATION_BINARY", "MATDOG_MANIFEST_VERSION"):
            m = manifest()
            del m[key]
            v = verify(m)
            self.assertFalse(v.ok, key)
            self.assertEqual(v.reason, Refusal.MANIFEST_INCOMPLETE, key)

    def test_unknown_manifest_version_refuses(self):
        v = verify(manifest(MATDOG_MANIFEST_VERSION="2"))
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.MANIFEST_VERSION_UNKNOWN)

    def test_parser_rejects_malformed_input(self):
        for bad in ("SOURCE_COMMIT\n", "=value\n", "A=1\nA=2\n"):
            with self.assertRaises(ValueError, msg=bad):
                parse_manifest(bad)

    def test_parser_accepts_comments_and_blank_lines(self):
        m = parse_manifest("# a comment\n\nHARDWARE_PROFILE=USB_ONLY\n")
        self.assertEqual(m["HARDWARE_PROFILE"], "USB_ONLY")

    def test_parser_preserves_equals_signs_in_values(self):
        # The FQBN is full of '=' — the value must not be truncated at the
        # first one.
        fqbn = "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M"
        m = parse_manifest(f"FQBN={fqbn}\n")
        self.assertEqual(m["FQBN"], fqbn)

    def test_render_then_parse_roundtrip(self):
        text = render_manifest(
            source_commit=HEAD, build_id="34afbc7808e2", source_state="CLEAN",
            profile="ROBOT_POWERED", fqbn=CANONICAL_FQBN,
            application_binary="MATDOG_Controller.ino.bin",
            application_size=387632, application_sha256=SHA)
        m = parse_manifest(text)
        self.assertEqual(m["HARDWARE_PROFILE"], "ROBOT_POWERED")
        self.assertEqual(m["APPLICATION_SIZE"], "387632")
        self.assertEqual(m["SOURCE_COMMIT"], HEAD)
        # Every key the verifier requires must be produced by the writer —
        # otherwise a freshly built tree would refuse its own manifest.
        v = verify_manifest(m, head_commit=HEAD, expected_fqbn=CANONICAL_FQBN,
                            tree_state="CLEAN", binary_exists=True,
                            binary_size=387632, binary_sha256=SHA,
                            requested_profile="ROBOT_POWERED")
        self.assertTrue(v.ok, v.detail)


class TestCliEndToEnd(unittest.TestCase):
    """Exercises write -> verify through the real CLI against real files,
    which is what build.sh and flash_app_only.sh actually invoke.

    The CLI's own stdout is captured rather than printed: this suite is run
    unattended from static_audit.py, whose failure message embeds captured
    output, so a quiet pass keeps that readable."""

    @staticmethod
    def _quiet(fn, *args, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            return fn(*args, **kwargs)

    def _build(self, tmp, profile, content=b"firmware-bytes", fqbn=CANONICAL_FQBN):
        binary = Path(tmp) / "MATDOG_Controller.ino.bin"
        binary.write_bytes(content)
        out = Path(tmp) / "matdog_build_manifest.txt"
        rc = self._quiet(main, ["write", "--output", str(out), "--binary", str(binary),
                                 "--source-commit", HEAD, "--build-id", "34afbc7808e2",
                                 "--source-state", "CLEAN", "--profile", profile,
                                 "--fqbn", fqbn])
        self.assertEqual(rc, 0)
        return binary, out

    def _verify(self, binary, out, requested, head=HEAD, tree="CLEAN",
                expected_fqbn=CANONICAL_FQBN):
        return self._quiet(main, ["verify", "--manifest", str(out), "--binary", str(binary),
                                  "--head", head, "--expected-fqbn", expected_fqbn,
                                  "--tree-state", tree,
                                  "--requested-profile", requested])

    def test_write_then_verify_usb_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "USB_ONLY")
            self.assertEqual(self._verify(binary, out, "USB_ONLY"), 0)
            self.assertEqual(self._verify(binary, out, "ROBOT_POWERED"), 1)

    def test_write_then_verify_robot_powered(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "ROBOT_POWERED")
            self.assertEqual(self._verify(binary, out, "ROBOT_POWERED"), 0)
            # No authorization -> refused.
            self.assertEqual(self._verify(binary, out, DEFAULT_FLASH_PROFILE), 1)

    def test_rebuilt_binary_invalidates_a_stale_manifest(self):
        # The exact scenario from the review: a manifest from one profile
        # left next to a binary that was since replaced by the other
        # profile's build.
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "USB_ONLY")
            binary.write_bytes(b"a-different-robot-powered-image")
            self.assertEqual(self._verify(binary, out, "USB_ONLY"), 1)

    def test_missing_manifest_file_refuses(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "USB_ONLY")
            out.unlink()
            self.assertEqual(self._verify(binary, out, "USB_ONLY"), 1)

    def test_unparseable_manifest_file_refuses(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "USB_ONLY")
            out.write_text("this is not a manifest\n")
            self.assertEqual(self._verify(binary, out, "USB_ONLY"), 1)

    def test_write_refuses_unknown_profile(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "MATDOG_Controller.ino.bin"
            binary.write_bytes(b"x")
            rc = self._quiet(main, ["write", "--output", str(Path(tmp) / "m.txt"),
                                    "--binary", str(binary), "--source-commit", HEAD,
                                    "--build-id", "x", "--source-state", "CLEAN",
                                    "--profile", "SOMETHING_ELSE", "--fqbn", CANONICAL_FQBN])
            self.assertEqual(rc, 1)

    def test_write_refuses_missing_binary(self):
        with tempfile.TemporaryDirectory() as tmp:
            rc = self._quiet(main, ["write", "--output", str(Path(tmp) / "m.txt"),
                                    "--binary", str(Path(tmp) / "nope.bin"),
                                    "--source-commit", HEAD, "--build-id", "x",
                                    "--source-state", "CLEAN", "--profile", "USB_ONLY",
                                    "--fqbn", CANONICAL_FQBN])
            self.assertEqual(rc, 1)

    def test_fqbn_mismatch_via_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "USB_ONLY")
            self.assertEqual(self._verify(binary, out, "USB_ONLY"), 0)
            self.assertEqual(
                self._verify(binary, out, "USB_ONLY",
                             expected_fqbn="esp32:esp32:esp32s3:FlashSize=8M"), 1)

    def test_artifact_built_with_wrong_fqbn_refuses_via_cli(self):
        # End to end: a build genuinely recorded under a different FQBN,
        # verified against the flasher's pinned one.
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "USB_ONLY",
                                      fqbn="esp32:esp32:esp32s3:PartitionScheme=default")
            self.assertEqual(self._verify(binary, out, "USB_ONLY"), 1)

    def test_both_profiles_end_to_end_with_canonical_fqbn(self):
        for profile in KNOWN_PROFILES:
            with tempfile.TemporaryDirectory() as tmp:
                binary, out = self._build(tmp, profile)
                self.assertEqual(self._verify(binary, out, profile), 0, profile)

    def test_commit_mismatch_via_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "USB_ONLY")
            self.assertEqual(self._verify(binary, out, "USB_ONLY",
                                          head=OTHER_COMMIT), 1)

    def test_dirty_tree_via_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "USB_ONLY")
            self.assertEqual(self._verify(binary, out, "USB_ONLY", tree="DIRTY"), 1)

    def test_recorded_sha256_matches_real_file_digest(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary, out = self._build(tmp, "USB_ONLY", content=b"deterministic")
            m, err = load_manifest(out)
            self.assertIsNone(err)
            self.assertEqual(m["APPLICATION_SHA256"], sha256_file(binary))
            self.assertEqual(int(m["APPLICATION_SIZE"]), binary.stat().st_size)


if __name__ == "__main__":
    unittest.main()
