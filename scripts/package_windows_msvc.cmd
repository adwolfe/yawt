@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem Build and package YAWT for portable Windows distribution from an MSVC cmd.
rem Requirements:
rem   - Run from "x64 Native Tools Command Prompt for VS" or equivalent.
rem   - Qt bin directory in PATH, including windeployqt.exe.
rem   - CMake in PATH.
rem   - OpenCV_DIR set to the directory containing OpenCVConfig.cmake, or pass
rem     --opencv-dir C:\path\to\opencv\build.

set "APP_NAME=yawt"
set "BUILD_DIR=build\windows-msvc-package"
set "PACKAGE_DIR=dist\windows"
set "BUILD_TYPE=Release"
set "GENERATOR=Ninja"
set "CLEAN_BUILD=0"
set "SKIP_BUILD=0"
set "CREATE_ARCHIVE=1"
set "OPEN_EXPLORER=0"
set "OPENCV_DIR_ARG="

call :parse_args %*
if errorlevel 2 exit /b 0
if errorlevel 1 exit /b 1

call :print_step "YAWT Windows Packaging"
echo Build Type: %BUILD_TYPE%
echo Generator: %GENERATOR%
echo Build Dir: %BUILD_DIR%
echo Package Dir: %PACKAGE_DIR%
echo Clean Build: %CLEAN_BUILD%
echo Skip Build: %SKIP_BUILD%
echo Create Archive: %CREATE_ARCHIVE%
echo.

set "WINDEPLOYQT_MODE=--release"
if /I "%BUILD_TYPE%"=="Debug" set "WINDEPLOYQT_MODE=--debug"

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "ROOT_DIR=%%~fI"
set "BUILD_ABS=%ROOT_DIR%\%BUILD_DIR%"
set "PACKAGE_ABS=%ROOT_DIR%\%PACKAGE_DIR%"
set "APP_DIR=%PACKAGE_ABS%\%APP_NAME%"

where cmake >nul 2>nul
if errorlevel 1 (
    call :print_error "cmake.exe was not found in PATH."
    exit /b 1
)

where windeployqt >nul 2>nul
if errorlevel 1 (
    call :print_error "windeployqt.exe was not found in PATH. Add your Qt bin directory to PATH."
    exit /b 1
)

if defined OPENCV_DIR_ARG (
    set "OpenCV_DIR=%OPENCV_DIR_ARG%"
)

if not defined OpenCV_DIR (
    if defined OpenCV_ROOT (
        set "OpenCV_DIR=%OpenCV_ROOT%"
    )
)

if not defined OpenCV_DIR (
    call :print_error "OpenCV_DIR is not set. Point it at the directory containing OpenCVConfig.cmake."
    echo Example:
    echo   set OpenCV_DIR=C:\opencv\build
    echo   scripts\package_windows_msvc.cmd
    exit /b 1
)

call :find_opencv_config "%OpenCV_DIR%"
if errorlevel 1 (
    call :print_error "OpenCVConfig.cmake was not found under: %OpenCV_DIR%"
    exit /b 1
)
set "OpenCV_DIR=%OPENCV_CONFIG_DIR%"
echo OpenCV_DIR: %OpenCV_DIR%

for %%I in ("%OpenCV_DIR%\..\..") do set "OPENCV_BASE_A=%%~fI"
for %%I in ("%OpenCV_DIR%\..") do set "OPENCV_BASE_B=%%~fI"
set "OPENCV_BIN_DIR="
call :find_opencv_bin "%OpenCV_DIR%"
if errorlevel 1 (
    call :print_error "Could not locate OpenCV runtime DLLs near OpenCV_DIR."
    echo Expected a bin directory containing opencv_*.dll or opencv_world*.dll.
    exit /b 1
)
echo OpenCV bin: %OPENCV_BIN_DIR%
echo.

if "%CLEAN_BUILD%"=="1" (
    call :print_step "Cleaning build and package directories"
    if exist "%BUILD_ABS%" rmdir /s /q "%BUILD_ABS%"
    if exist "%PACKAGE_ABS%" rmdir /s /q "%PACKAGE_ABS%"
)

if not "%SKIP_BUILD%"=="1" (
    call :print_step "Configuring"
    cmake -S "%ROOT_DIR%" -B "%BUILD_ABS%" -G "%GENERATOR%" -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DOpenCV_DIR="%OpenCV_DIR%"
    if errorlevel 1 exit /b 1

    call :print_step "Building"
    cmake --build "%BUILD_ABS%" --config %BUILD_TYPE% --parallel
    if errorlevel 1 exit /b 1
)

call :find_exe
if errorlevel 1 (
    call :print_error "Could not find %APP_NAME%.exe in %BUILD_ABS%."
    exit /b 1
)

call :print_step "Preparing portable folder"
if exist "%APP_DIR%" rmdir /s /q "%APP_DIR%"
mkdir "%APP_DIR%" || exit /b 1
copy /y "%APP_EXE%" "%APP_DIR%\" >nul || exit /b 1

call :print_step "Running windeployqt"
windeployqt %WINDEPLOYQT_MODE% --compiler-runtime --dir "%APP_DIR%" "%APP_DIR%\%APP_NAME%.exe"
if errorlevel 1 exit /b 1

call :print_step "Copying OpenCV runtime DLLs"
set "COPIED_OPENCV=0"
for %%F in ("%OPENCV_BIN_DIR%\opencv_world*.dll" "%OPENCV_BIN_DIR%\opencv_*.dll") do (
    if exist "%%~fF" (
        copy /y "%%~fF" "%APP_DIR%\" >nul
        set "COPIED_OPENCV=1"
        echo Copied %%~nxF
    )
)

if not "%COPIED_OPENCV%"=="1" (
    call :print_error "No OpenCV DLLs were copied from %OPENCV_BIN_DIR%."
    exit /b 1
)

call :print_step "Copying OpenCV video codec/runtime DLLs"
call :copy_optional_runtime_pattern "OpenCV FFmpeg video I/O backend" "opencv_videoio_ffmpeg*.dll" "%APP_DIR%"
call :copy_optional_runtime_pattern "OpenH264 codec runtime" "openh264*.dll" "%APP_DIR%"

call :print_step "Copying MSVC runtime DLLs found on PATH"
for %%D in (vcruntime140.dll vcruntime140_1.dll msvcp140.dll concrt140.dll) do (
    call :copy_from_path "%%D" "%APP_DIR%"
)

if "%CREATE_ARCHIVE%"=="1" (
    call :print_step "Creating zip archive"
    set "ARCHIVE=%PACKAGE_ABS%\%APP_NAME%-windows-%BUILD_TYPE%.zip"
    if exist "!ARCHIVE!" del /q "!ARCHIVE!"
    powershell -NoProfile -ExecutionPolicy Bypass -Command "Compress-Archive -Path '%APP_DIR%' -DestinationPath '!ARCHIVE!' -Force"
    if errorlevel 1 exit /b 1
    echo Archive: !ARCHIVE!
)

call :print_success "Portable package ready: %APP_DIR%"
if "%OPEN_EXPLORER%"=="1" start "" "%PACKAGE_ABS%"
exit /b 0

:usage
echo Usage: scripts\package_windows_msvc.cmd [options]
echo.
echo Options:
echo   --debug                 Build Debug instead of Release.
echo   --clean                 Remove build and dist output first.
echo   --no-build              Skip configure/build and package existing exe.
echo   --no-archive            Do not create dist\windows\yawt-windows-*.zip.
echo   --build-dir DIR         CMake build directory. Default: build\windows-msvc-package
echo   --package-dir DIR       Package output directory. Default: dist\windows
echo   --generator NAME        CMake generator. Default: Ninja
echo   --opencv-dir DIR        Directory containing OpenCVConfig.cmake.
echo   --open                  Open package folder in Explorer when done.
echo   --help                  Show this help.
exit /b 0

:parse_args
if "%~1"=="" exit /b 0
if /I "%~1"=="--debug" (
    set "BUILD_TYPE=Debug"
    shift
    goto parse_args
)
if /I "%~1"=="--clean" (
    set "CLEAN_BUILD=1"
    shift
    goto parse_args
)
if /I "%~1"=="--no-build" (
    set "SKIP_BUILD=1"
    shift
    goto parse_args
)
if /I "%~1"=="--no-archive" (
    set "CREATE_ARCHIVE=0"
    shift
    goto parse_args
)
if /I "%~1"=="--open" (
    set "OPEN_EXPLORER=1"
    shift
    goto parse_args
)
if /I "%~1"=="--help" (
    call :usage
    exit /b 2
)
if /I "%~1"=="-h" (
    call :usage
    exit /b 2
)
if /I "%~1"=="--build-dir" (
    if "%~2"=="" (
        call :print_error "--build-dir requires a value."
        exit /b 1
    )
    set "BUILD_DIR=%~2"
    shift
    shift
    goto parse_args
)
if /I "%~1"=="--package-dir" (
    if "%~2"=="" (
        call :print_error "--package-dir requires a value."
        exit /b 1
    )
    set "PACKAGE_DIR=%~2"
    shift
    shift
    goto parse_args
)
if /I "%~1"=="--generator" (
    if "%~2"=="" (
        call :print_error "--generator requires a value."
        exit /b 1
    )
    set "GENERATOR=%~2"
    shift
    shift
    goto parse_args
)
if /I "%~1"=="--opencv-dir" (
    if "%~2"=="" (
        call :print_error "--opencv-dir requires a value."
        exit /b 1
    )
    set "OPENCV_DIR_ARG=%~2"
    shift
    shift
    goto parse_args
)
call :print_error "Unknown option: %~1"
call :usage
exit /b 1

:find_opencv_config
set "OPENCV_CONFIG_DIR="
if exist "%~1\OpenCVConfig.cmake" (
    set "OPENCV_CONFIG_DIR=%~f1"
    exit /b 0
)
for /r "%~1" %%F in (OpenCVConfig.cmake) do (
    set "OPENCV_CONFIG_DIR=%%~dpF"
    set "OPENCV_CONFIG_DIR=!OPENCV_CONFIG_DIR:~0,-1!"
    exit /b 0
)
exit /b 1

:find_opencv_bin
for %%D in (
    "%OpenCV_DIR%\..\bin"
    "%OpenCV_DIR%\..\..\bin"
    "%OpenCV_DIR%\..\..\..\bin"
    "%OPENCV_BASE_A%\bin"
    "%OPENCV_BASE_A%\x64\vc17\bin"
    "%OPENCV_BASE_A%\x64\vc16\bin"
    "%OPENCV_BASE_A%\x64\vc15\bin"
    "%OPENCV_BASE_B%\bin"
    "%OPENCV_BASE_B%\x64\vc17\bin"
    "%OPENCV_BASE_B%\x64\vc16\bin"
    "%OPENCV_BASE_B%\x64\vc15\bin"
) do (
    if exist "%%~fD\opencv_world*.dll" (
        set "OPENCV_BIN_DIR=%%~fD"
        exit /b 0
    )
    if exist "%%~fD\opencv_*.dll" (
        set "OPENCV_BIN_DIR=%%~fD"
        exit /b 0
    )
)
for /r "%OpenCV_DIR%" %%F in (opencv_world*.dll opencv_*.dll) do (
    set "OPENCV_BIN_DIR=%%~dpF"
    set "OPENCV_BIN_DIR=!OPENCV_BIN_DIR:~0,-1!"
    exit /b 0
)
exit /b 1

:find_exe
set "APP_EXE="
for %%F in (
    "%BUILD_ABS%\%APP_NAME%.exe"
    "%BUILD_ABS%\%BUILD_TYPE%\%APP_NAME%.exe"
    "%BUILD_ABS%\src\%APP_NAME%.exe"
    "%BUILD_ABS%\src\%BUILD_TYPE%\%APP_NAME%.exe"
) do (
    if exist "%%~fF" (
        set "APP_EXE=%%~fF"
        echo App exe: !APP_EXE!
        exit /b 0
    )
)
exit /b 1

:copy_from_path
set "DLL_NAME=%~1"
set "DEST_DIR=%~2"
for %%P in ("%DLL_NAME%") do (
    if exist "%%~$PATH:P" (
        copy /y "%%~$PATH:P" "%DEST_DIR%\" >nul
        echo Copied %%~nxP
        exit /b 0
    )
)
exit /b 0

:copy_optional_runtime_pattern
set "RUNTIME_LABEL=%~1"
set "RUNTIME_PATTERN=%~2"
set "DEST_DIR=%~3"
set "COPIED_RUNTIME=0"

for %%D in (
    "%OPENCV_BIN_DIR%"
    "%OpenCV_DIR%"
    "%OpenCV_DIR%\.."
    "%OpenCV_DIR%\..\.."
    "%OpenCV_DIR%\..\..\.."
    "%OPENCV_BASE_A%"
    "%OPENCV_BASE_A%\bin"
    "%OPENCV_BASE_A%\x64\vc17\bin"
    "%OPENCV_BASE_A%\x64\vc16\bin"
    "%OPENCV_BASE_A%\x64\vc15\bin"
    "%OPENCV_BASE_B%"
    "%OPENCV_BASE_B%\bin"
    "%OPENCV_BASE_B%\x64\vc17\bin"
    "%OPENCV_BASE_B%\x64\vc16\bin"
    "%OPENCV_BASE_B%\x64\vc15\bin"
) do (
    call :copy_pattern_from_dir "%%~fD" "%RUNTIME_PATTERN%" "%DEST_DIR%"
)

if "%COPIED_RUNTIME%"=="1" (
    echo Included %RUNTIME_LABEL%.
) else (
    echo [WARN] Did not find %RUNTIME_LABEL% ^(%RUNTIME_PATTERN%^).
    echo        Crop/video export may fail for H264/MP4 unless this DLL is installed beside %APP_NAME%.exe.
)
exit /b 0

:copy_pattern_from_dir
set "SEARCH_DIR=%~1"
set "SEARCH_PATTERN=%~2"
set "DEST_DIR=%~3"
if not exist "%SEARCH_DIR%" exit /b 0
for %%F in ("%SEARCH_DIR%\%SEARCH_PATTERN%") do (
    if exist "%%~fF" (
        copy /y "%%~fF" "%DEST_DIR%\" >nul
        set "COPIED_RUNTIME=1"
        echo Copied %%~nxF
    )
)
exit /b 0

:print_step
echo.
echo === %~1 ===
exit /b 0

:print_success
echo [OK] %~1
exit /b 0

:print_error
echo [ERROR] %~1
exit /b 0
