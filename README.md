## YET ANOTHER WORM TRACKER

#### Because why not?

This was written to process C. elegans chemotaxis behavior and follows a simple flow: if I know how many worms there are in an experiment (say, 8), then I should be able to start from a point where I can see all 8 worms and track outwards from that. This was written to see whether that is a better way of doing things.

---

## Building

YAWT is a C++17 / Qt 6 / OpenCV application built with CMake.

These instructions assume you are building the `centerline` branch.

### Requirements

- CMake 3.16 or newer
- Qt 6 with Core, Gui, Widgets, and Svg
- OpenCV
- A C++17 compiler

### Get the `centerline` branch

If you have not already cloned the repo:

```bash
git clone -b centerline https://github.com/adwolfe/yawt.git
cd yawt
```

If you downloaded the branch as a ZIP from GitHub, unzip it and `cd` into that folder instead.

---

## macOS

These steps assume macOS with Homebrew.

### Install dependencies

```bash
brew install cmake qt opencv
```

### Build and run

From the repository root:

```bash
# 1. Configure
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(qtpaths6 --install-prefix)"

# 2. Compile
cmake --build build -j"$(sysctl -n hw.logicalcpu)"

# 3. Launch
open -n build/yawt.app
```

In Finder, the app may appear as `yawt` instead of `yawt.app` because macOS often hides the `.app` extension.

To see live log output, run the app binary directly:

```bash
./build/yawt.app/Contents/MacOS/yawt
```

After editing source code, usually only steps 2 and 3 are needed again.

### macOS troubleshooting

If CMake cannot find Qt, make sure `CMAKE_PREFIX_PATH` points at Qt 6. On Apple Silicon Homebrew this is usually:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/homebrew
```

On Intel Macs it is often:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local
```

Prefer `qtpaths6` over `qtpaths`; on some systems, especially when Anaconda is earlier on `PATH`, `qtpaths` may point to the wrong Qt.

If CMake cannot find OpenCV, point it at Homebrew's OpenCV package:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$(qtpaths6 --install-prefix)" \
  -DOpenCV_DIR="$(brew --prefix opencv)/lib/cmake/opencv4"
```

---

## Windows

These steps are for building from source on Windows. The exact Qt and OpenCV folder names may differ on your computer, so adjust the paths in the commands.

### Install dependencies

Install:

- Visual Studio 2022 with the "Desktop development with C++" workload
- CMake
- Qt 6 for MSVC 64-bit
- OpenCV for Windows

Use matching 64-bit MSVC builds for Qt, OpenCV, and Visual Studio. A mismatch can cause build or launch failures.

### Configure and build

Open "x64 Native Tools Command Prompt for VS 2022" or a PowerShell terminal where Visual Studio build tools are available.

From the repository root:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_PREFIX_PATH="C:\Qt\6.x.x\msvc2022_64" `
  -DOpenCV_DIR="C:\opencv\build"

cmake --build build --config Release --parallel
```

Replace:

- `C:\Qt\6.x.x\msvc2022_64` with your Qt 6 MSVC folder
- `C:\opencv\build` with the folder containing `OpenCVConfig.cmake`

### Run from the build folder

For a developer run, make sure Qt and OpenCV DLL folders are on `PATH`:

```powershell
$env:Path = "C:\Qt\6.x.x\msvc2022_64\bin;C:\opencv\build\x64\vc16\bin;$env:Path"
.\build\Release\yawt.exe
```

Adjust the OpenCV `vc16` folder if your OpenCV install uses a different Visual Studio runtime folder.

### Make a portable Windows folder

To run YAWT on a Windows computer that does not have Qt and OpenCV installed, copy the required DLLs next to `yawt.exe`.

Example:

```powershell
New-Item -ItemType Directory -Force .\dist\yawt-windows | Out-Null
Copy-Item .\build\Release\yawt.exe .\dist\yawt-windows\

& "C:\Qt\6.x.x\msvc2022_64\bin\windeployqt.exe" --compiler-runtime .\dist\yawt-windows\yawt.exe

Copy-Item "C:\opencv\build\x64\vc16\bin\opencv_world*.dll" .\dist\yawt-windows\
Copy-Item "C:\opencv\build\x64\vc16\bin\opencv_videoio_ffmpeg*.dll" .\dist\yawt-windows\ -ErrorAction SilentlyContinue
Copy-Item "C:\opencv\build\x64\vc16\bin\openh264*.dll" .\dist\yawt-windows\ -ErrorAction SilentlyContinue
```

Then launch:

```powershell
.\dist\yawt-windows\yawt.exe
```

### Windows smoke test

After building or packaging on Windows, verify:

- the app opens
- a video loads
- circular crop selection saves a cropped movie
- the cropped movie exists, is not zero bytes, and plays
- tracking works from frame 0
- tracking works from a middle frame
- tracks and centerlines display
- `.xlsx` output files are written

The crop tool currently uses an H264 writer. On Windows, H264 saving may require `openh264*.dll`; if crop export fails or creates an unplayable file, use the checklist above to catch it.
