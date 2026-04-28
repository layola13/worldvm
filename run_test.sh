#!/usr/bin/env bash
set -euo pipefail

PER_TEST_TIMEOUT="${PER_TEST_TIMEOUT:-120}"
BATCH_OVERHEAD="${BATCH_OVERHEAD:-15}"

if ! [[ "$PER_TEST_TIMEOUT" =~ ^[0-9]+$ ]]; then
  echo "PER_TEST_TIMEOUT must be an integer number of seconds" >&2
  exit 2
fi
if ! [[ "$BATCH_OVERHEAD" =~ ^[0-9]+$ ]]; then
  echo "BATCH_OVERHEAD must be an integer number of seconds" >&2
  exit 2
fi

if (( PER_TEST_TIMEOUT <= 0 )); then
  echo "PER_TEST_TIMEOUT must be > 0" >&2
  exit 2
fi
if (( PER_TEST_TIMEOUT > 120 )); then
  echo "PER_TEST_TIMEOUT=$PER_TEST_TIMEOUT exceeds policy cap, clamping to 120s" >&2
  PER_TEST_TIMEOUT=120
fi

declare -a TARGETS=()
if (( $# == 0 )); then
  while IFS= read -r path; do
    TARGETS+=("$path")
  done < <(find src -maxdepth 1 -type f -name "*.zig" | sort)
else
  TARGETS=("$@")
fi

if (( ${#TARGETS[@]} == 0 )); then
  echo "No test targets provided." >&2
  exit 2
fi

BATCH_TIMEOUT=$(( PER_TEST_TIMEOUT * ${#TARGETS[@]} + BATCH_OVERHEAD ))
echo "Per-target timeout: ${PER_TEST_TIMEOUT}s"
echo "Target count: ${#TARGETS[@]}"
echo "Batch budget (count-aware): ${BATCH_TIMEOUT}s"

run_target() {
  local target="$1"
  if [[ "$target" == "build" ]]; then
    echo "=== zig build ==="
    timeout "${PER_TEST_TIMEOUT}s" zig build
    return
  fi

  if [[ ! -f "$target" ]]; then
    echo "Target not found: $target" >&2
    exit 2
  fi

  echo "=== zig test $target ==="
  timeout "${PER_TEST_TIMEOUT}s" zig test "$target"
}

for target in "${TARGETS[@]}"; do
  run_target "$target"
done

echo "All targets completed within per-target timeout policy."
