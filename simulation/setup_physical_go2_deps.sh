#!/usr/bin/env bash
# Install the unversioned upstream LibTorch runtime needed by HIMLoco.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PHYSICAL_WS="${PHYSICAL_WS:-$REPO_ROOT/simulation/physical_go2_ws}"
TARGET="$PHYSICAL_WS/library/inference_runtime/libtorch"

if [[ -f "$TARGET/lib/libtorch.so" ]]; then
  echo "[physical_go2] LibTorch already available: $TARGET"
  exit 0
fi

UPSTREAM_ROOT="${RLSAR_UPSTREAM_ROOT:-$REPO_ROOT/../new_algorithm/rl_sar_upstream}"
SOURCE="$UPSTREAM_ROOT/library/inference_runtime/libtorch"
if [[ -f "$SOURCE/lib/libtorch.so" ]]; then
  mkdir -p "$(dirname "$TARGET")"
  # Preserve a local runtime path; hard links avoid a second 764 MB copy on
  # the same filesystem. It remains intentionally gitignored.
  cp -al "$SOURCE" "$TARGET"
  echo "[physical_go2] Linked local LibTorch runtime into $TARGET"
  exit 0
fi

echo "[physical_go2] LibTorch is absent; preparing repository-local runtime"
if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "x86_64" ]]; then
  echo "[physical_go2] automatic LibTorch provisioning currently supports Linux x86_64 only" >&2
  exit 2
fi

RUNTIME_DIR="$PHYSICAL_WS/library/inference_runtime"
ARCHIVE="$RUNTIME_DIR/libtorch-2.3.0-cpu.zip"
URL="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.3.0%2Bcpu.zip"
mkdir -p "$RUNTIME_DIR"
echo "[physical_go2] downloading LibTorch 2.3.0 CPU into repository-local ignored directory"
curl -fL --progress-bar -o "$ARCHIVE" "$URL"
unzip -q "$ARCHIVE" -d "$RUNTIME_DIR"
rm -f "$ARCHIVE"
[[ -f "$TARGET/lib/libtorch.so" ]] || {
  echo "[physical_go2] extracted runtime is incomplete: $TARGET" >&2
  exit 2
}
echo "[physical_go2] LibTorch ready: $TARGET"
