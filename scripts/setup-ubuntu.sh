#!/usr/bin/env bash
# System packages for ogccdb-viewer on Ubuntu/Debian (run once, needs sudo).
# ogccdb-core's own packages (GDAL, pugixml, ...) come from its scripts/setup-ubuntu.sh.
set -euo pipefail
sudo apt-get update
sudo apt-get install -y cmake ninja-build g++ git libglfw3-dev libgl-dev libsqlite3-dev libzstd-dev liblz4-dev
