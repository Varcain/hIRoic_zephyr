#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source "$SCRIPT_DIR/.env"
BUILD_DIR="$SCRIPT_DIR/build"

export ZEPHYR_BASE="$ZEPHYR_WORKSPACE/zephyr"
export ZEPHYR_SDK_INSTALL_DIR="$ZEPHYR_SDK"

source "$ZEPHYR_WORKSPACE/.venv/bin/activate"

west flash --build-dir "$BUILD_DIR" --runner openocd "$@"
