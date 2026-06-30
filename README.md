## YET ANOTHER WORM TRACKER

#### Because why not? 

This was written to process C. elegans chemotaxis behavior and follows a simple flow: if I know how many worms there are in an experiment (say, 8) then I should be able to start from a point where I can see all 8 worms and track outwards from that. This was written to see whether that is a better way of doing things. 

## Build notes

YAWT uses CMake with Qt 6 and OpenCV. Basler Pylon support is not part of the current build.

### Windows

Install Qt 6 and OpenCV built for the same compiler you use with CMake. For Visual Studio 2022, OpenCV's CMake package directory is commonly:

```powershell
C:\opencv\build\x64\vc16\lib
```

Configure with explicit Qt and OpenCV package paths:

```powershell
cmake -S . -B build\windows-msvc -G Ninja `
  -DCMAKE_PREFIX_PATH=C:\Qt\6.7.0\msvc2019_64 `
  -DOpenCV_DIR=C:\opencv\build\x64\vc16\lib
cmake --build build\windows-msvc
```

The same values can be set in Qt Creator under the kit's CMake configuration:

```text
CMAKE_PREFIX_PATH=C:/Qt/6.7.0/msvc2019_64
OpenCV_DIR=C:/opencv/build/x64/vc16/lib
```

To build and package from an x64 MSVC command prompt with Qt on `PATH`:

```bat
set OpenCV_DIR=C:\opencv\build
scripts\package_windows_msvc.cmd --clean
```

The packaged app is written to `dist\windows\yawt`, with a zip archive beside it.
  
