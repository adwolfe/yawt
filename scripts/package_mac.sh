#!/bin/bash
set -euo pipefail

# Build + bundle + sign + zip for macOS distribution.
# - Bundles Qt + OpenCV + dependent Homebrew dylibs
# - Fixes install_name paths
# - Ad-hoc signs by default; optional Developer ID signing

APP_NAME="yawt"
BUILD_DIR="build"
BUILD_TYPE="Release"
CLEAN_BUILD=false
SKIP_BUILD=false
USE_DEVELOPER_ID=false

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_step() { echo -e "${BLUE}=== $1 ===${NC}"; }
print_success() { echo -e "${GREEN}✓ $1${NC}"; }
print_warning() { echo -e "${YELLOW}⚠ $1${NC}"; }
print_error() { echo -e "${RED}✗ $1${NC}"; }

usage() {
    cat << EOF
Usage: $0 [options]

Options:
  -d, --debug          Build Debug (default: Release)
  -c, --clean          Clean build directory before building
  -n, --no-build       Skip build step (assumes build/yawt.app exists)
  --developer-id       Sign with Apple Developer ID if available
  -h, --help           Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--debug) BUILD_TYPE="Debug"; shift ;;
        -c|--clean) CLEAN_BUILD=true; shift ;;
        -n|--no-build) SKIP_BUILD=true; shift ;;
        --developer-id) USE_DEVELOPER_ID=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) print_error "Unknown option: $1"; usage; exit 1 ;;
    esac
done

print_step "YAWT macOS Packaging"
echo "Build Type: $BUILD_TYPE"
echo "Clean Build: $CLEAN_BUILD"
echo "Skip Build: $SKIP_BUILD"
echo "Developer ID Signing: $USE_DEVELOPER_ID"
echo

if $CLEAN_BUILD; then
    print_step "Cleaning Build Directory"
    rm -rf "$BUILD_DIR"
fi

if ! $SKIP_BUILD; then
    print_step "Building"
    mkdir -p "$BUILD_DIR"
    pushd "$BUILD_DIR" >/dev/null
    if command -v qtpaths >/dev/null 2>&1; then
        QT_PREFIX="$(qtpaths --install-prefix)"
    elif command -v qtpaths6 >/dev/null 2>&1; then
        QT_PREFIX="$(qtpaths6 --install-prefix)"
    else
        print_error "qtpaths/qtpaths6 not found in PATH"
        exit 1
    fi
    cmake .. -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DCMAKE_PREFIX_PATH="$QT_PREFIX"
    make -j"$(sysctl -n hw.logicalcpu)"
    popd >/dev/null
fi

if [ ! -d "$BUILD_DIR/$APP_NAME.app" ]; then
    print_error "App not found at $BUILD_DIR/$APP_NAME.app"
    exit 1
fi

# Detect Homebrew prefix (Apple Silicon default /opt/homebrew, Intel default /usr/local)
if command -v brew >/dev/null 2>&1; then
    HOMEBREW_PREFIX="$(brew --prefix 2>/dev/null || true)"
else
    ARCH="$(uname -m)"
    if [ "$ARCH" = "x86_64" ]; then
        HOMEBREW_PREFIX="/usr/local"
    else
        HOMEBREW_PREFIX="/opt/homebrew"
    fi
fi

print_step "Bundling Dependencies"

pushd "$BUILD_DIR" >/dev/null

FRAMEWORKS_DIR="$APP_NAME.app/Contents/Frameworks"
PLUGINS_DIR="$APP_NAME.app/Contents/PlugIns"
mkdir -p "$FRAMEWORKS_DIR" "$PLUGINS_DIR/platforms" "$PLUGINS_DIR/imageformats"

QT_LIB_PATH="$HOMEBREW_PREFIX/opt/qt/lib"
for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
    if [ -d "$QT_LIB_PATH/$framework.framework" ]; then
        cp -R "$QT_LIB_PATH/$framework.framework" "$FRAMEWORKS_DIR/"
        chmod -R u+w "$FRAMEWORKS_DIR/$framework.framework"
        rm -rf "$FRAMEWORKS_DIR/$framework.framework/Versions/A/Headers" 2>/dev/null || true
        rm -rf "$FRAMEWORKS_DIR/$framework.framework/Headers" 2>/dev/null || true
        rm -f "$FRAMEWORKS_DIR/$framework.framework/Versions/A"/*.prl 2>/dev/null || true
    else
        print_warning "$framework.framework not found"
    fi
done

if [ -d "$HOMEBREW_PREFIX/lib/qt6/plugins" ]; then
    QT_PLUGINS_PATH="$HOMEBREW_PREFIX/lib/qt6/plugins"
else
    QT_PLUGINS_PATH="$HOMEBREW_PREFIX/share/qt/plugins"
fi
if [ -f "$QT_PLUGINS_PATH/platforms/libqcocoa.dylib" ]; then
    cp "$QT_PLUGINS_PATH/platforms/libqcocoa.dylib" "$PLUGINS_DIR/platforms/"
fi
for plugin in libqjpeg.dylib libqpng.dylib libqgif.dylib libqico.dylib libqsvg.dylib; do
    if [ -f "$QT_PLUGINS_PATH/imageformats/$plugin" ]; then
        cp "$QT_PLUGINS_PATH/imageformats/$plugin" "$PLUGINS_DIR/imageformats/"
    fi
done

OPENCV_LIBS=$(otool -L "$APP_NAME.app/Contents/MacOS/$APP_NAME" | grep "$HOMEBREW_PREFIX.*opencv" | awk '{print $1}')
for lib_path in $OPENCV_LIBS; do
    if [ -f "$lib_path" ]; then
        lib_name=$(basename "$lib_path")
        cp "$lib_path" "$FRAMEWORKS_DIR/"
        chmod u+w "$FRAMEWORKS_DIR/$lib_name"
    fi
done

if [ -d "$HOMEBREW_PREFIX/opt/opencv/lib" ]; then
    OPENCV_LIB_DIR="$HOMEBREW_PREFIX/opt/opencv/lib"
elif [ -d "$HOMEBREW_PREFIX/opt/opencv@4/lib" ]; then
    OPENCV_LIB_DIR="$HOMEBREW_PREFIX/opt/opencv@4/lib"
else
    OPENCV_LIB_DIR=""
fi
if [ -d "$OPENCV_LIB_DIR" ]; then
    for lib in "$OPENCV_LIB_DIR"/libopencv_*.dylib; do
        [ -f "$lib" ] || continue
        lib_name=$(basename "$lib")
        if [ ! -f "$FRAMEWORKS_DIR/$lib_name" ]; then
            cp "$lib" "$FRAMEWORKS_DIR/"
            chmod u+w "$FRAMEWORKS_DIR/$lib_name"
        fi
    done
else
    for lib in "$HOMEBREW_PREFIX/lib"/libopencv_*.dylib; do
        [ -f "$lib" ] || continue
        lib_name=$(basename "$lib")
        if [ ! -f "$FRAMEWORKS_DIR/$lib_name" ]; then
            cp "$lib" "$FRAMEWORKS_DIR/"
            chmod u+w "$FRAMEWORKS_DIR/$lib_name"
        fi
    done
fi

for opencv_lib in "$FRAMEWORKS_DIR"/libopencv_*.dylib; do
    if [ -f "$opencv_lib" ]; then
        DEPS=$(otool -L "$opencv_lib" | grep "$HOMEBREW_PREFIX" | grep -v opencv | awk '{print $1}')
        for dep in $DEPS; do
            if [ -f "$dep" ]; then
                dep_name=$(basename "$dep")
                if [ ! -f "$FRAMEWORKS_DIR/$dep_name" ]; then
                    cp "$dep" "$FRAMEWORKS_DIR/"
                    chmod u+w "$FRAMEWORKS_DIR/$dep_name"
                fi
            fi
        done
    fi
done

CHANGED=true
PASS=0
while $CHANGED && [ $PASS -lt 5 ]; do
    CHANGED=false
    PASS=$((PASS+1))
    for f in "$APP_NAME.app/Contents/MacOS/$APP_NAME" "$FRAMEWORKS_DIR"/*.dylib "$FRAMEWORKS_DIR"/Qt*.framework/Versions/A/* "$PLUGINS_DIR"/*/*.dylib; do
        [ -f "$f" ] || continue
        DEPS=$(otool -L "$f" 2>/dev/null | awk '{print $1}' | grep "^$HOMEBREW_PREFIX/" || true)
        for dep in $DEPS; do
            dep_name=$(basename "$dep")
            if [[ "$dep" == *"/Qt"*"framework/Versions/"*"/*" ]]; then
                fw_dir=$(echo "$dep" | sed -E 's#(Qt[^/]*\.framework)/.*#\1#')
                fw_name=$(basename "$fw_dir")
                if [ ! -d "$FRAMEWORKS_DIR/$fw_name" ] && [ -d "$QT_LIB_PATH/$fw_name" ]; then
                    cp -R "$QT_LIB_PATH/$fw_name" "$FRAMEWORKS_DIR/"
                    chmod -R u+w "$FRAMEWORKS_DIR/$fw_name"
                    rm -rf "$FRAMEWORKS_DIR/$fw_name/Versions/A/Headers" "$FRAMEWORKS_DIR/$fw_name/Headers" 2>/dev/null || true
                    CHANGED=true
                fi
            else
                if [ ! -f "$FRAMEWORKS_DIR/$dep_name" ] && [ -f "$dep" ]; then
                    cp "$dep" "$FRAMEWORKS_DIR/"
                    chmod u+w "$FRAMEWORKS_DIR/$dep_name"
                    CHANGED=true
                fi
            fi
        done
    done
done

for libglob in \
    "$HOMEBREW_PREFIX"/lib/libdouble-conversion*.dylib \
    "$HOMEBREW_PREFIX"/opt/double-conversion/lib/libdouble-conversion*.dylib \
    "$HOMEBREW_PREFIX"/lib/libmd4c*.dylib \
    "$HOMEBREW_PREFIX"/opt/md4c/lib/libmd4c*.dylib; do
    [ -f "$libglob" ] || continue
    b=$(basename "$libglob")
    if [ ! -f "$FRAMEWORKS_DIR/$b" ]; then
        cp "$libglob" "$FRAMEWORKS_DIR/"
        chmod u+w "$FRAMEWORKS_DIR/$b"
    fi
done

cat > "$APP_NAME.app/Contents/Resources/qt.conf" << 'EOF'
[Paths]
Plugins = PlugIns
EOF

print_step "Fixing Library Paths"

fix_lib_path() {
    local old_path="$1"
    local new_path="$2"
    local target="$3"
    if [ -f "$target" ] && [ -w "$target" ]; then
        if otool -L "$target" 2>/dev/null | grep -q "$old_path"; then
            install_name_tool -change "$old_path" "$new_path" "$target" 2>/dev/null || true
        fi
    fi
}

install_name_tool -add_rpath "@executable_path/../Frameworks" "$APP_NAME.app/Contents/MacOS/$APP_NAME" 2>/dev/null || true
for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
    old_path="$HOMEBREW_PREFIX/opt/qt/lib/$framework.framework/Versions/A/$framework"
    new_path="@executable_path/../Frameworks/$framework.framework/Versions/A/$framework"
    fix_lib_path "$old_path" "$new_path" "$APP_NAME.app/Contents/MacOS/$APP_NAME"
done
for lib_path in $OPENCV_LIBS; do
    lib_name=$(basename "$lib_path")
    fix_lib_path "$lib_path" "@executable_path/../Frameworks/$lib_name" "$APP_NAME.app/Contents/MacOS/$APP_NAME"
done

for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
    if [ -f "$FRAMEWORKS_DIR/$framework.framework/Versions/A/$framework" ]; then
        install_name_tool -id "@executable_path/../Frameworks/$framework.framework/Versions/A/$framework" \
            "$FRAMEWORKS_DIR/$framework.framework/Versions/A/$framework" 2>/dev/null || true
    fi
done

for lib_file in "$FRAMEWORKS_DIR"/*.dylib; do
    [ -f "$lib_file" ] || continue
    lib_name=$(basename "$lib_file")
    install_name_tool -id "@executable_path/../Frameworks/$lib_name" "$lib_file" 2>/dev/null || true
    DEPS=$(otool -L "$lib_file" 2>/dev/null | grep "$HOMEBREW_PREFIX" | awk '{print $1}' || true)
    for dep_path in $DEPS; do
        dep_name=$(basename "$dep_path")
        if [ -f "$FRAMEWORKS_DIR/$dep_name" ]; then
            install_name_tool -change "$dep_path" "@executable_path/../Frameworks/$dep_name" "$lib_file" 2>/dev/null || true
        fi
    done
    for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
        old_path="$HOMEBREW_PREFIX/opt/qt/lib/$framework.framework/Versions/A/$framework"
        new_path="@executable_path/../Frameworks/$framework.framework/Versions/A/$framework"
        fix_lib_path "$old_path" "$new_path" "$lib_file"
    done
    if otool -L "$lib_file" 2>/dev/null | grep -q "@rpath/libopencv_"; then
        for cv in "$FRAMEWORKS_DIR"/libopencv_*.dylib; do
            [ -f "$cv" ] || continue
            cv_name=$(basename "$cv")
            if otool -L "$lib_file" 2>/dev/null | grep -q "@rpath/$cv_name"; then
                install_name_tool -change "@rpath/$cv_name" "@executable_path/../Frameworks/$cv_name" "$lib_file" 2>/dev/null || true
            fi
        done
    fi
done

for plugin in "$PLUGINS_DIR"/*/*.dylib; do
    [ -f "$plugin" ] || continue
    for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
        old_path="$HOMEBREW_PREFIX/opt/qt/lib/$framework.framework/Versions/A/$framework"
        new_path="@executable_path/../Frameworks/$framework.framework/Versions/A/$framework"
        fix_lib_path "$old_path" "$new_path" "$plugin"
    done
    DEPS=$(otool -L "$plugin" 2>/dev/null | awk '{print $1}' | grep "^$HOMEBREW_PREFIX/" || true)
    for dep_path in $DEPS; do
        dep_name=$(basename "$dep_path")
        if [ -f "$FRAMEWORKS_DIR/$dep_name" ]; then
            install_name_tool -change "$dep_path" "@executable_path/../Frameworks/$dep_name" "$plugin" 2>/dev/null || true
        fi
    done
    for cv in "$FRAMEWORKS_DIR"/libopencv_*.dylib; do
        [ -f "$cv" ] || continue
        cv_name=$(basename "$cv")
        if otool -L "$plugin" 2>/dev/null | grep -q "@rpath/$cv_name"; then
            install_name_tool -change "@rpath/$cv_name" "@executable_path/../Frameworks/$cv_name" "$plugin" 2>/dev/null || true
        fi
    done
done

print_step "Signing"

SIGN_ID="-"
ENTITLEMENTS_FILE=""
if $USE_DEVELOPER_ID; then
    CERT_LIST=$(security find-identity -v -p codesigning | grep "Developer ID Application" | head -10 || true)
    if [ -z "$CERT_LIST" ]; then
        print_warning "Developer ID certificate not found; falling back to ad-hoc signing"
    else
        CERT_COUNT=$(echo "$CERT_LIST" | wc -l | xargs)
        if [ "$CERT_COUNT" -eq 1 ]; then
            SIGN_ID=$(echo "$CERT_LIST" | awk '{print $2}')
        else
            echo "Available Developer ID certificates:"
            echo "$CERT_LIST" | nl
            read -p "Select certificate (1-$CERT_COUNT): " CERT_SELECTION
            if ! [[ "$CERT_SELECTION" =~ ^[1-9][0-9]*$ ]] || [ "$CERT_SELECTION" -gt "$CERT_COUNT" ]; then
                print_error "Invalid selection"
                exit 1
            fi
            SELECTED_LINE=$(echo "$CERT_LIST" | sed -n "${CERT_SELECTION}p")
            SIGN_ID=$(echo "$SELECTED_LINE" | awk '{print $2}')
        fi
        ENTITLEMENTS_FILE="$PWD/entitlements.plist"
        cat > "$ENTITLEMENTS_FILE" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>com.apple.security.cs.allow-jit</key>
    <true/>
    <key>com.apple.security.cs.allow-unsigned-executable-memory</key>
    <true/>
    <key>com.apple.security.cs.allow-dyld-environment-variables</key>
    <true/>
    <key>com.apple.security.cs.disable-library-validation</key>
    <true/>
    <key>com.apple.security.cs.disable-executable-page-protection</key>
    <true/>
</dict>
</plist>
EOF
    fi
fi

codesign --remove-signature "$APP_NAME.app" 2>/dev/null || true

if [ "$SIGN_ID" = "-" ]; then
    find "$FRAMEWORKS_DIR" -name "*.dylib" -exec codesign --force --sign - {} \; 2>/dev/null
    find "$FRAMEWORKS_DIR" -name "Qt*" -type f -path "*/Versions/A/*" -exec codesign --force --sign - {} \; 2>/dev/null
    find "$PLUGINS_DIR" -name "*.dylib" -exec codesign --force --sign - {} \; 2>/dev/null
    codesign --force --sign - "$APP_NAME.app/Contents/MacOS/$APP_NAME"
    codesign --force --sign - "$APP_NAME.app"
else
    find "$APP_NAME.app" -name "*.dylib" -exec codesign --force --sign "$SIGN_ID" --timestamp --options runtime {} \;
    find "$APP_NAME.app" -name "*.framework" -exec codesign --force --sign "$SIGN_ID" --timestamp --options runtime {} \;
    if [ -n "$ENTITLEMENTS_FILE" ]; then
        codesign --force --sign "$SIGN_ID" --timestamp --options runtime --entitlements "$ENTITLEMENTS_FILE" "$APP_NAME.app/Contents/MacOS/$APP_NAME"
        codesign --force --sign "$SIGN_ID" --timestamp --options runtime --entitlements "$ENTITLEMENTS_FILE" "$APP_NAME.app"
    else
        codesign --force --sign "$SIGN_ID" --timestamp --options runtime "$APP_NAME.app/Contents/MacOS/$APP_NAME"
        codesign --force --sign "$SIGN_ID" --timestamp --options runtime "$APP_NAME.app"
    fi
fi

print_step "Packaging"
xattr -dr com.apple.quarantine "$APP_NAME.app" 2>/dev/null || true
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
ARCHIVE_NAME="$APP_NAME-macos-$TIMESTAMP.zip"
zip -r "$ARCHIVE_NAME" "$APP_NAME.app" >/dev/null

if [ -n "$ENTITLEMENTS_FILE" ]; then
    rm -f "$ENTITLEMENTS_FILE"
fi

print_success "Done"
echo "App bundle: $PWD/$APP_NAME.app"
echo "Archive: $PWD/$ARCHIVE_NAME"

popd >/dev/null
