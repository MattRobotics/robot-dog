#!/usr/bin/env bash
# Compiles MATDOG Controller with the pinned FQBN. Does not touch
# hardware — no upload happens here.
#
# FLASH LAYOUT (P2.3). PartitionScheme=custom makes the Arduino core use
# the sketch-folder partitions.csv (layout MATDOG_16M_2x5M_NVS_V1: two 5 MiB
# app slots, default NVS BEFORE matdog_nvs). The installed core is never
# modified. The FQBN name alone does not prove the table, so after the compile
# scripts/matdog_layout.py checks the binary table the build ACTUALLY produced
# (exact SHA-256), the application size against the 5 MiB slot (> 5,242,880 B
# is not an acceptable build; >= 4 MiB is a growth warning) and that the layout
# id is compiled into the binary. upload.maximum_size is only passed so the
# arduino-cli size report agrees; it is never relied on.
set -euo pipefail
export GIT_OPTIONAL_LOCKS=0
export PYTHONDONTWRITEBYTECODE=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$SKETCH_DIR/../.." && pwd)"

ARDUINO="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
FQBN='esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,DebugLevel=none,PSRAM=opi'

SOURCE_COMMIT="nogit"
SOURCE_STATE="NO_GIT"
GIT_DIRTY="1"
if git -C "$REPO_ROOT" rev-parse HEAD >/dev/null 2>&1; then
  SOURCE_COMMIT="$(git -C "$REPO_ROOT" rev-parse HEAD)"
  if [ -n "$(git -C "$REPO_ROOT" status --porcelain)" ]; then
    SOURCE_STATE="DIRTY"
  else
    SOURCE_STATE="CLEAN"
    GIT_DIRTY="0"
  fi
fi
BUILD_ID="${SOURCE_COMMIT:0:12}"
if [ "$GIT_DIRTY" = "1" ]; then BUILD_ID="${BUILD_ID}-dirty"; fi

# UTC is one observation shared by the embedded identity and manifest.
# SOURCE_DATE_EPOCH is the explicit reproducible-build policy, when supplied.
mapfile -t IDENTITY < <(python3 - "$SKETCH_DIR" <<'PY'
import datetime, os, re, sys
from pathlib import Path
root = Path(sys.argv[1])
config = (root / "src/config/BuildConfig.h").read_text()
def literal(name):
    match = re.search(r'\b' + name + r'\s*=\s*"([^"\n]+)"', config)
    if not match:
        raise SystemExit("missing identity literal: " + name)
    return match[1]
epoch = os.environ.get("SOURCE_DATE_EPOCH")
if epoch is not None and not re.fullmatch(r"[0-9]+", epoch):
    raise SystemExit("SOURCE_DATE_EPOCH must be a nonnegative integer")
now = (datetime.datetime.fromtimestamp(int(epoch), datetime.timezone.utc)
       if epoch is not None else datetime.datetime.now(datetime.timezone.utc))
print(literal("kFirmwareVersion"))
print(literal("kMotionStack"))
print(now.strftime("%Y-%m-%dT%H:%M:%SZ"))
print("SOURCE_DATE_EPOCH" if epoch is not None else "UTC_NOW")
PY
)
if [ "${#IDENTITY[@]}" != "4" ]; then echo "ERROR: build identity unavailable" >&2; exit 1; fi
FW_VERSION="${IDENTITY[0]}"
MOTION_STACK="${IDENTITY[1]}"
BUILD_UTC="${IDENTITY[2]}"
BUILD_UTC_POLICY="${IDENTITY[3]}"

# Hardware profile override. The SOURCE default is USB_ONLY and must stay
# USB_ONLY (scripts/static_audit.py enforces that — it is the G3
# authorization gate). A powered-validation session overrides it here, for
# one build, without editing the repository default:
#
#   MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh
#
# Deliberately explicit and loud: the chosen profile is echoed below and
# ends up in the boot banner, so a ROBOT_POWERED image can never be
# produced or flashed silently.
PROFILE_FLAG=""
PROFILE_NAME="USB_ONLY (source default)"
PROFILE_ID="USB_ONLY"
case "${MATDOG_PROFILE:-}" in
  "")
    ;;
  USB_ONLY)
    PROFILE_FLAG=" -DMATDOG_ACTIVE_HARDWARE_PROFILE=::matdog::config::HardwareProfile::USB_ONLY"
    PROFILE_NAME="USB_ONLY (explicit)"
    PROFILE_ID="USB_ONLY"
    ;;
  ROBOT_POWERED)
    PROFILE_FLAG=" -DMATDOG_ACTIVE_HARDWARE_PROFILE=::matdog::config::HardwareProfile::ROBOT_POWERED"
    PROFILE_NAME="ROBOT_POWERED (OVERRIDE — requires G3 authorization)"
    PROFILE_ID="ROBOT_POWERED"
    ;;
  *)
    echo "ERROR: MATDOG_PROFILE='${MATDOG_PROFILE}' is not a known profile" >&2
    echo "       valid values: USB_ONLY, ROBOT_POWERED" >&2
    exit 1
    ;;
esac

# OTA-ingest validation override (I7 hardening, 2026-09-25). The SOURCE
# default is 0 and must stay 0 — scripts/static_audit.py fails the build if
# the #define's own default in src/update/OtaManager.h is ever anything
# else, so this override can never change what a plain `scripts/build.sh`
# produces. It exists so the ONE hardware-validation candidate can carry a
# real, reachable OTA ingest writer instead of needing a second flash later
# just to turn it on:
#
#   MATDOG_OTA_INGEST_VALIDATION=1 MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh
#
# Deliberately explicit and loud, the same shape as MATDOG_PROFILE above: the
# chosen value is echoed below and recorded in the build manifest
# (OTA_INGEST_ENABLED=1), so a binary with the firmware-ingest writer
# compiled in can never be produced, flashed or mistaken for an ordinary
# build silently. It is a SEPARATE axis from MATDOG_PROFILE — a
# ROBOT_POWERED build with ingest still 0 is the ordinary case.
OTA_INGEST_FLAG=""
OTA_INGEST_ID="0"
OTA_INGEST_NAME="DISABLED (source default)"
case "${MATDOG_OTA_INGEST_VALIDATION:-}" in
  ""|0)
    ;;
  1)
    OTA_INGEST_FLAG=" -DMATDOG_OTA_INGEST_ENABLED=1"
    OTA_INGEST_ID="1"
    OTA_INGEST_NAME="ENABLED (OVERRIDE — hardware-validation candidate only)"
    ;;
  *)
    echo "ERROR: MATDOG_OTA_INGEST_VALIDATION='${MATDOG_OTA_INGEST_VALIDATION}' must be 0 or 1" >&2
    exit 1
    ;;
esac

echo "== MATDOG Controller build =="
echo "sketch     : $SKETCH_DIR"
echo "fqbn       : $FQBN"
echo "build_id   : $BUILD_ID"
echo "profile    : $PROFILE_NAME"
echo "ota_ingest : $OTA_INGEST_NAME"
echo "fw_version : $FW_VERSION"
echo "git_sha    : $SOURCE_COMMIT"
echo "git_dirty  : $GIT_DIRTY"
echo "build_utc  : $BUILD_UTC ($BUILD_UTC_POLICY)"
echo

BUILD_DIR="$SKETCH_DIR/build/esp32.esp32.esp32s3"
CLI_OPTIONS=()
COMPILE_OPTIONS=(--export-binaries)

# This explicit mode keeps compile products, temporary files, indexes and CLI
# user/cache state outside every registered worktree and Git common directory.
# Installed packages and libraries are inputs only: this script never invokes
# install, update, discovery, upload or any hardware command. The CLI savehex
# hook writes into the sketch when --export-binaries is used, so external mode
# omits that flag and packages the generated products itself.
if [ -n "${MATDOG_BUILD_ROOT:-}" ]; then
  mapfile -t EXTERNAL_PATHS < <(python3 - "$REPO_ROOT" "$PROFILE_ID" "$OTA_INGEST_ID" <<'PY'
import os, subprocess, sys
from pathlib import Path
repo, profile, ota = sys.argv[1:]
raw = Path(os.environ["MATDOG_BUILD_ROOT"])
if not raw.is_absolute():
    raise SystemExit("MATDOG_BUILD_ROOT must be absolute")
root = raw.resolve()
protected = [Path(repo).resolve()]
result = subprocess.run(["git", "-C", repo, "worktree", "list", "--porcelain"],
                        capture_output=True, text=True, check=True)
protected += [Path(line[9:]).resolve() for line in result.stdout.splitlines()
              if line.startswith("worktree ")]
common = subprocess.run(["git", "-C", repo, "rev-parse", "--git-common-dir"],
                        capture_output=True, text=True, check=True).stdout.strip()
protected.append((Path(repo) / common).resolve())
for path in protected:
    if root == path or root.is_relative_to(path) or path.is_relative_to(root):
        raise SystemExit("external build root intersects a protected Git path: " + str(path))
paths = [root]
for name, default in (("MATDOG_BUILD_DIR", root / "compile" / (profile + "-ota" + ota)),
                      ("MATDOG_OUTPUT_DIR", root / "artifacts" / (profile + "-ota" + ota)),
                      ("MATDOG_BUILD_CACHE_DIR", root / "cache"),
                      ("MATDOG_BUILD_TMP_DIR", root / "tmp")):
    candidate = Path(os.environ.get(name, str(default)))
    if not candidate.is_absolute():
        raise SystemExit(name + " must be absolute")
    candidate = candidate.resolve()
    if candidate == root or not candidate.is_relative_to(root):
        raise SystemExit(name + " escapes the external build root")
    if any(candidate == p or candidate.is_relative_to(p) or p.is_relative_to(candidate)
           for p in protected):
        raise SystemExit(name + " intersects a protected Git path")
    paths.append(candidate)
if len(set(paths)) != len(paths):
    raise SystemExit("external output/cache/temp paths must be distinct")
for path in paths:
    print(path)
PY
  )
  if [ "${#EXTERNAL_PATHS[@]}" != "5" ]; then echo "ERROR: unsafe external build paths" >&2; exit 1; fi
  EXTERNAL_ROOT="${EXTERNAL_PATHS[0]}"
  COMPILE_DIR="${EXTERNAL_PATHS[1]}"
  BUILD_DIR="${EXTERNAL_PATHS[2]}"
  export ARDUINO_BUILD_CACHE_PATH="${EXTERNAL_PATHS[3]}"
  export TMPDIR="${EXTERNAL_PATHS[4]}"
  # Additional CLI arguments cannot override this mode's paths or run upload.
  # --jobs is the sole compile tuning parameter accepted here.
  COMPILE_EXTRA=()
  while [ "$#" -gt 0 ]; do
    if [ "$1" = "--jobs" ] && [ "$#" -ge 2 ] && [[ "$2" =~ ^[1-9][0-9]*$ ]]; then
      COMPILE_EXTRA+=(--jobs "$2"); shift 2
    else
      echo "ERROR: external mode refuses additional argument '$1'" >&2; exit 1
    fi
  done
  mkdir -p "$COMPILE_DIR" "$BUILD_DIR" "$ARDUINO_BUILD_CACHE_PATH" "$TMPDIR"
  CLI_CONFIG="$EXTERNAL_ROOT/arduino-cli.yaml"
  python3 - "$EXTERNAL_ROOT" "$ARDUINO_BUILD_CACHE_PATH" <<'PY'
import json, os, shutil, sys
from pathlib import Path
root, cache = map(Path, sys.argv[1:])
source = Path(os.environ.get("MATDOG_ARDUINO_DATA_SOURCE", str(Path.home() / ".arduino15"))).resolve()
libraries = Path(os.environ.get("MATDOG_ARDUINO_LIBRARY_ROOT", str(Path.home() / "Arduino/libraries"))).resolve()
if not (source / "packages/esp32/hardware/esp32/3.3.11").is_dir() or not libraries.is_dir():
    raise SystemExit("installed offline Arduino core/libraries are unavailable")
data, downloads, user = [root / "arduino" / name for name in ("data", "downloads", "user")]
for path in (data, downloads, user):
    if path.resolve() != path or not path.is_relative_to(root):
        raise SystemExit("Arduino state path escaped external root")
    path.mkdir(parents=True, exist_ok=True)
packages = data / "packages"
if packages.is_symlink():
    if packages.resolve() != source / "packages":
        raise SystemExit("external packages pointer differs from installed input")
elif packages.exists():
    raise SystemExit("external packages path is not the expected input symlink")
else:
    packages.symlink_to(source / "packages", target_is_directory=True)
for path in source.iterdir():
    if path.is_file() and (path.name.endswith((".json", ".json.sig")) or path.name == "inventory.yaml"):
        target = data / path.name
        if target.is_symlink():
            raise SystemExit("external index file must not be a symlink")
        shutil.copyfile(path, target)
# JSON is valid YAML; paths are safely quoted, no shell/YAML interpolation.
config = {"directories": {"data": str(data), "downloads": str(downloads), "user": str(user)},
          "build_cache": {"path": str(cache)}, "board_manager": {"additional_urls": []}}
target = root / "arduino-cli.yaml"
if target.is_symlink():
    raise SystemExit("external CLI config must not be a symlink")
target.write_text(json.dumps(config, indent=2) + "\n")
PY
  CLI_OPTIONS=(--config-file "$CLI_CONFIG")
  LIBRARY_ROOT="${MATDOG_ARDUINO_LIBRARY_ROOT:-$HOME/Arduino/libraries}"
  COMPILE_OPTIONS=(--clean --build-path "$COMPILE_DIR" --libraries "$LIBRARY_ROOT" "${COMPILE_EXTRA[@]}")
fi
APPLICATION_BINARY="$BUILD_DIR/MATDOG_Controller.ino.bin"
PARTITION_ARTIFACT="$BUILD_DIR/MATDOG_Controller.ino.partitions.bin"
MANIFEST="$BUILD_DIR/matdog_build_manifest.txt"

# A previous build's table or manifest must never be mistaken for this one's.
rm -f "$MANIFEST" "$PARTITION_ARTIFACT"
if [ -n "${EXTERNAL_ROOT:-}" ]; then
  # Failed or no-output compiles must not repackage an earlier application.
  rm -f "$APPLICATION_BINARY" "$COMPILE_DIR/MATDOG_Controller.ino.bin" "$COMPILE_DIR/MATDOG_Controller.ino.partitions.bin"
fi

python3 "$SCRIPT_DIR/matdog_layout.py" check-fqbn --fqbn "$FQBN" >/dev/null

"$ARDUINO" "${CLI_OPTIONS[@]}" compile \
  --fqbn "$FQBN" \
  --build-property "upload.maximum_size=5242880" \
  --build-property "compiler.cpp.extra_flags=-DMATDOG_BUILD_ID=\"${BUILD_ID}\" -DMATDOG_GIT_SHA=\"${SOURCE_COMMIT}\" -DMATDOG_GIT_DIRTY=${GIT_DIRTY} -DMATDOG_BUILD_UTC=\"${BUILD_UTC}\"${PROFILE_FLAG}${OTA_INGEST_FLAG}" \
  --warnings all \
  "${COMPILE_OPTIONS[@]}" \
  "$SKETCH_DIR" \
  "$@"

if [ -n "${EXTERNAL_ROOT:-}" ]; then
  for product in "$COMPILE_DIR"/MATDOG_Controller.ino.* "$COMPILE_DIR/sdkconfig"; do
    if [ -f "$product" ]; then cp -- "$product" "$BUILD_DIR/"; fi
  done
fi

# --- Build manifest (G2 pre-G3 hardening, review Finding 1) ----------------
# Both profiles produce the same artifact pathname from the same commit, so
# the embedded build id alone cannot prove WHICH profile a binary came from.
# This binds profile + exact bytes + source state to the build, and
# scripts/flash_app_only.sh refuses to write anything it cannot verify
# against this file. The manifest lives inside the gitignored build output
# directory, adjacent to the binary — it is a build artifact, never
# committed.
# Stale-manifest safety: if the compile produced no binary, make sure a
# previous build's manifest cannot be left behind to be verified against.
if [ ! -f "$APPLICATION_BINARY" ]; then
  rm -f "$MANIFEST"
  echo "ERROR: expected application binary not found after compile: $APPLICATION_BINARY" >&2
  exit 1
fi

if [ ! -f "$PARTITION_ARTIFACT" ]; then
  echo "ERROR: the build produced no binary partition table: $PARTITION_ARTIFACT" >&2
  exit 1
fi

echo
echo "== Flash layout gate =="
python3 "$SCRIPT_DIR/matdog_layout.py" check-build \
  --partitions "$PARTITION_ARTIFACT" \
  --binary "$APPLICATION_BINARY" \
  --fqbn "$FQBN"

python3 "$SCRIPT_DIR/build_manifest.py" write \
  --output "$MANIFEST" \
  --binary "$APPLICATION_BINARY" \
  --source-commit "$SOURCE_COMMIT" \
  --build-id "$BUILD_ID" \
  --source-state "$SOURCE_STATE" \
  --profile "$PROFILE_ID" \
  --ota-ingest "$OTA_INGEST_ID" \
  --fqbn "$FQBN" \
  --fw-version "$FW_VERSION" \
  --build-utc "$BUILD_UTC" \
  --build-utc-policy "$BUILD_UTC_POLICY" \
  --motion-stack "$MOTION_STACK" \
  --cal-record-schema 1 \
  --cal-marker-schema 2
