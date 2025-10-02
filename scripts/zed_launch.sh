#!/bin/bash
set -e
cd "$(dirname "$0")/../build"
"./yawt.app/Contents/MacOS/yawt" -verbosity=1
