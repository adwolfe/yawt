@echo off
REM ============================================
REM  open-vcvars.cmd
REM  Finds and calls vcvarsall.bat for MSVC
REM  Usage: open-vcvars.cmd [arch] [vcvars_ver]
REM  Example: open-vcvars.cmd x64 14.29
REM ============================================

setlocal

REM --- Default architecture if not provided ---
set "ARCH=%~1"
if "%ARCH%"=="" set "ARCH=x64"

REM --- Optional MSVC toolset version ---
set "VCVARS_VER=%~2"

REM --- Locate vswhere.exe ---
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
    echo ERROR: vswhere.exe not found. Install Visual Studio or vswhere.
    exit /b 1
)

REM --- Find latest VS installation path ---
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -products * -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
    set "VSINSTALL=%%i"
)

if "%VSINSTALL%"=="" (
    echo ERROR: No Visual Studio installation with C++ tools found.
    exit /b 1
)

REM --- Build path to vcvarsall.bat ---
set "VCVARSALL=%VSINSTALL%\VC\Auxiliary\Build\vcvarsall.bat"
if not exist "%VCVARSALL%" (
    echo ERROR: vcvarsall.bat not found at "%VCVARSALL%"
    exit /b 1
)

REM --- Call vcvarsall with arch and optional version ---
if "%VCVARS_VER%"=="" (
    call "%VCVARSALL%" %ARCH%
) else (
    call "%VCVARSALL%" %ARCH% -vcvars_ver=%VCVARS_VER%
)

REM --- Load Qt environment into the same shell ---
call "C:\Qt\6.11.1\msvc2022_64\bin\qtenv2.bat"

set "OpenCV_DIR=C:/opencv/build/x64/vc16/lib"

REM --- Change to your desired working directory ---
cd /d "C:\Users\dispim\Desktop\Aaron_Devel\Github\yawt"

REM --- Keep shell open ---
cmd /k
endlocal
