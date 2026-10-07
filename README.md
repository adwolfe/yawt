## YET ANOTHER WORM TRACKER

#### Because why not? 

This was written to process C. elegans chemotaxis behavior and follows a simple flow: if I know how many worms there are in an experiment (say, 8) then I should be able to start from a point where I can see all 8 worms and track outwards from that. This was written to see whether that is a better way of doing things. 

In the Processing tab, **Subtract background** below **Enable blur** removes stationary debris before thresholding. It estimates a median background from up to 31 frames evenly spaced across the video, using the existing **Light/Dark** background selection. The same background is used for the preview and tracking and is rebuilt when the video changes. This option is saved with tracking settings; older runs default to off. Objects that remain stationary in most sampled frames can also be removed. Adjust the threshold after enabling subtraction if needed.

Use **Start frame** and **Stop frame** above **Begin Tracking** to process an inclusive range of zero-based frame numbers. The scrub bar keeps the full video scale, with thick gray bars over excluded frames; scrubbing, frame navigation, and playback stay inside the selected range. The worms’ selection frame must be inside it. Outputs keep the original video frame numbers. Opening a video resets the range to the whole video.

## Build notes

YAWT uses CMake with >= Qt 6.10 and OpenCV. Basler Pylon support is not part of the current build.

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

The packaging script copies OpenCV runtime DLLs and explicitly looks for `opencv_videoio_ffmpeg*.dll` and `openh264*.dll`. Those are needed for OpenCV MP4/H264 video I/O, including crop export. If the script prints a warning for either DLL, copy the missing DLL from your OpenCV install next to `yawt.exe`, or use an OpenCV build that includes the FFmpeg video I/O backend.
  
