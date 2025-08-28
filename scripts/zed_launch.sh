#!/bin/bash
set -e
cd "$(dirname "$0")/build"
open -n "$ZED_WORKTREE_ROOT/build/yawt.app" --args --debug-mode
