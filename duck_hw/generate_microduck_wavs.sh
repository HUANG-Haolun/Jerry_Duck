#!/usr/bin/env bash
set -euo pipefail

# Run from any directory; generated wavs go to duck_hw/duck_sounds/.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/duck_hw/duck_sounds"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

if ! command -v cargo >/dev/null 2>&1; then
    echo "Rust/Cargo is required. Install Rust, then rerun this script." >&2
    exit 1
fi
if ! command -v git >/dev/null 2>&1; then
    echo "git is required to fetch the official MicroDuck source." >&2
    exit 1
fi

git clone --depth 1 https://github.com/pollen-robotics/microduck.git "$TMP/microduck"
cargo build --manifest-path "$TMP/microduck/Cargo.toml" --release -p sounds
mkdir -p "$DEST"
"$TMP/microduck/target/release/sounds" render-all "$DEST" --seed 1234
echo "Generated official MicroDuck voice WAVs in: $DEST"
echo "Count: $(find "$DEST" -type f -name '*.wav' | wc -l)"
