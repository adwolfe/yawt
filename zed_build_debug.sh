#!/bin/bash
set -e
cd "$(dirname "$0")/build"
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=$(qtpaths --install-prefix)
make -j$(sysctl -n hw.logicalcpu)
open -n "$(dirname "$0")/build/yawt.app" --args --debug-mode
