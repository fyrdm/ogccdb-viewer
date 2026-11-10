#!/usr/bin/env bash
# Builds ogccdb-core, ogccdb-runtimepublisher and ogccdb-viewer from clones that sit
# next to each other (e.g. ~/ogccdb-core, ~/ogccdb-runtimepublisher, ~/ogccdb-viewer),
# installing the libraries to $OGCCDB_PREFIX (default ~/ogccdb-prefix).
# Missing clones are cloned; existing ones are built as checked out (no pull).
set -euo pipefail

HERE=$(cd "$(dirname "$0")/.." && pwd)
PARENT=$(dirname "$HERE")
PREFIX=${OGCCDB_PREFIX:-$HOME/ogccdb-prefix}
BUILD_DIR=build-release

clone() {
    local name=$1
    [ -d "$PARENT/$name" ] && return
    if command -v gh >/dev/null 2>&1; then
        gh repo clone "fyrdm/$name" "$PARENT/$name"
    else
        git clone "https://github.com/fyrdm/$name.git" "$PARENT/$name"
    fi
}

build() {  # build <dir> <extra cmake args...>
    local dir=$1; shift
    echo "== $(basename "$dir")"
    cmake -S "$dir" -B "$dir/$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH="$PREFIX" -DCMAKE_INSTALL_PREFIX="$PREFIX" "$@"
    cmake --build "$dir/$BUILD_DIR"
}

clone ogccdb-core
clone ogccdb-runtimepublisher

build "$PARENT/ogccdb-core" -DOGCCDB_BUILD_TESTS=OFF
cmake --install "$PARENT/ogccdb-core/$BUILD_DIR"
build "$PARENT/ogccdb-runtimepublisher" -DOGCCDB_RUNTIME_BUILD_TESTS=OFF
cmake --install "$PARENT/ogccdb-runtimepublisher/$BUILD_DIR"
build "$HERE"

echo
echo "Viewer: $HERE/$BUILD_DIR/ogccdb-viewer <cdb-root>"
