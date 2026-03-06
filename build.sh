#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source "$SCRIPT_DIR/.env"

export ZEPHYR_BASE="$ZEPHYR_WORKSPACE/zephyr"
export ZEPHYR_SDK_INSTALL_DIR="$ZEPHYR_SDK"

BUILD_DIR="$SCRIPT_DIR/build"

source "$ZEPHYR_WORKSPACE/.venv/bin/activate"

west build -b "$BOARD" "$SCRIPT_DIR" --build-dir "$BUILD_DIR" --pristine "$@"
