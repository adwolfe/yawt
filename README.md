## YET ANOTHER WORM TRACKER

#### Because why not? 

This was written to process C. elegans chemotaxis behavior and follows a simple flow: if I know how many worms there are in an experiment (say, 8) then I should be able to start from a point where I can see all 8 worms and track outwards from that. This was written to see whether that is a better way of doing things. 

---

## Building

YAWT is a C++17 / Qt 6 / OpenCV application built with CMake. These steps assume you've downloaded the `centerline` branch and are building on **macOS** (Apple Silicon or Intel) with [Homebrew](https://brew.sh).

### Requirements

- CMake ≥ 3.16
- Qt 6 (Core, Gui, Widgets, Svg)
- OpenCV

Install them with Homebrew:

```bash
brew install cmake qt opencv
```

### Get the `centerline` branch

If you haven't already cloned the repo:

```bash
git clone -b centerline https://github.com/adwolfe/yawt.git
cd yawt
```

(If you downloaded the branch as a ZIP from GitHub, just unzip it and `cd` into that folder instead.)

### Build & run

From the repository root:

```bash
# 1. Configure (Release build)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(qtpaths6 --install-prefix)"

# 2. Compile (uses all CPU cores; rebuilds are incremental)
cmake --build build -j"$(sysctl -n hw.logicalcpu)"

# 3. Launch
open -n build/yawt.app

# In Finder, the app may appear as `yawt` instead of `yawt.app` because macOS often hides the `.app` extension.
```

After editing source, just re-run steps 2–3 — only the files you changed get recompiled.

To see live log output (handy for debugging), run the binary directly instead of `open`:

```bash
./build/yawt.app/Contents/MacOS/yawt
```