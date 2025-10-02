#!/bin/bash

# Simple Working Deployment Script for YAWT macOS App
# This script bundles all dependencies and creates a distributable app

set -e

APP_NAME="yawt"
BUILD_DIR="build"
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

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_step() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_step "Simple YAWT Deployment"

# Locate app bundle (support running from repo root or from build/)
if [ -d "$BUILD_DIR/$APP_NAME.app" ]; then
    # Running from repository root; descend into build directory
    cd "$BUILD_DIR"
elif [ -d "$APP_NAME.app" ]; then
    # Already in build directory; keep working directory as-is
    BUILD_DIR="."
else
    print_error "App not found."
    echo "Expected at: '$BUILD_DIR/$APP_NAME.app' (repo root) or './$APP_NAME.app' (inside build)"
    echo "Please build first, for example:"
    echo "  mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make"
    exit 1
fi

# Create bundle directories
FRAMEWORKS_DIR="$APP_NAME.app/Contents/Frameworks"
PLUGINS_DIR="$APP_NAME.app/Contents/PlugIns"
mkdir -p "$FRAMEWORKS_DIR"
mkdir -p "$PLUGINS_DIR/platforms"
mkdir -p "$PLUGINS_DIR/imageformats"

print_step "Step 1: Copying Qt Frameworks"

# Copy Qt frameworks
QT_LIB_PATH="$HOMEBREW_PREFIX/opt/qt/lib"
for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
    if [ -d "$QT_LIB_PATH/$framework.framework" ]; then
        echo "  Copying $framework.framework"
        cp -R "$QT_LIB_PATH/$framework.framework" "$FRAMEWORKS_DIR/"

        # Make writable and clean up
        chmod -R u+w "$FRAMEWORKS_DIR/$framework.framework"
        rm -rf "$FRAMEWORKS_DIR/$framework.framework/Versions/A/Headers" 2>/dev/null || true
        rm -rf "$FRAMEWORKS_DIR/$framework.framework/Headers" 2>/dev/null || true
        rm -f "$FRAMEWORKS_DIR/$framework.framework/Versions/A"/*.prl 2>/dev/null || true
    else
        print_warning "$framework.framework not found"
    fi
done

print_step "Step 2: Copying Qt Plugins"

# Copy essential Qt plugins
if [ -d "$HOMEBREW_PREFIX/lib/qt6/plugins" ]; then
    QT_PLUGINS_PATH="$HOMEBREW_PREFIX/lib/qt6/plugins"
else
    QT_PLUGINS_PATH="$HOMEBREW_PREFIX/share/qt/plugins"
fi
if [ -f "$QT_PLUGINS_PATH/platforms/libqcocoa.dylib" ]; then
    cp "$QT_PLUGINS_PATH/platforms/libqcocoa.dylib" "$PLUGINS_DIR/platforms/"
    echo "  Copied Cocoa platform plugin"
fi

# Copy some image format plugins
for plugin in libqjpeg.dylib libqpng.dylib libqgif.dylib libqico.dylib libqsvg.dylib; do
    if [ -f "$QT_PLUGINS_PATH/imageformats/$plugin" ]; then
        cp "$QT_PLUGINS_PATH/imageformats/$plugin" "$PLUGINS_DIR/imageformats/"
    fi
done

print_step "Step 3: Copying OpenCV Libraries"

# Get all OpenCV libraries the app depends on (respect the detected Homebrew prefix)
OPENCV_LIBS=$(otool -L "$APP_NAME.app/Contents/MacOS/$APP_NAME" | grep "$HOMEBREW_PREFIX.*opencv" | awk '{print $1}')

for lib_path in $OPENCV_LIBS; do
    if [ -f "$lib_path" ]; then
        lib_name=$(basename "$lib_path")
        echo "  Copying $lib_name"
        cp "$lib_path" "$FRAMEWORKS_DIR/"
        chmod u+w "$FRAMEWORKS_DIR/$lib_name"
    fi
done

# Also copy ALL OpenCV modules from the keg (defensive in case some are pulled transitively)
if [ -d "$HOMEBREW_PREFIX/opt/opencv/lib" ]; then
    OPENCV_LIB_DIR="$HOMEBREW_PREFIX/opt/opencv/lib"
elif [ -d "$HOMEBREW_PREFIX/opt/opencv@4/lib" ]; then
    OPENCV_LIB_DIR="$HOMEBREW_PREFIX/opt/opencv@4/lib"
else
    OPENCV_LIB_DIR=""
fi
if [ -d "$OPENCV_LIB_DIR" ]; then
    echo "  Scanning $OPENCV_LIB_DIR for OpenCV modules..."
    for lib in "$OPENCV_LIB_DIR"/libopencv_*.dylib; do
        [ -f "$lib" ] || continue
        lib_name=$(basename "$lib")
        if [ ! -f "$FRAMEWORKS_DIR/$lib_name" ]; then
            echo "    Copying $lib_name"
            cp "$lib" "$FRAMEWORKS_DIR/"
            chmod u+w "$FRAMEWORKS_DIR/$lib_name"
        fi
    done
else
    # Fallback to prefix/lib if keg path isn't available
    echo "  Keg path not found; scanning $HOMEBREW_PREFIX/lib for OpenCV modules..."
    for lib in "$HOMEBREW_PREFIX/lib"/libopencv_*.dylib; do
        [ -f "$lib" ] || continue
        lib_name=$(basename "$lib")
        if [ ! -f "$FRAMEWORKS_DIR/$lib_name" ]; then
            echo "    Copying $lib_name"
            cp "$lib" "$FRAMEWORKS_DIR/"
            chmod u+w "$FRAMEWORKS_DIR/$lib_name"
        fi
    done
fi

# Copy OpenCV's dependencies (like TBB, OpenBLAS)
echo "  Finding OpenCV dependencies..."
for opencv_lib in "$FRAMEWORKS_DIR"/libopencv_*.dylib; do
    if [ -f "$opencv_lib" ]; then
        DEPS=$(otool -L "$opencv_lib" | grep "$HOMEBREW_PREFIX" | grep -v opencv | awk '{print $1}')
        for dep in $DEPS; do
            if [ -f "$dep" ]; then
                dep_name=$(basename "$dep")
                if [ ! -f "$FRAMEWORKS_DIR/$dep_name" ]; then
                    echo "    Copying dependency: $dep_name"
                    cp "$dep" "$FRAMEWORKS_DIR/"
                    chmod u+w "$FRAMEWORKS_DIR/$dep_name"
                fi
            fi
        done
    fi
done

# Additional dependency closure: copy any remaining Homebrew-linked dylibs referenced by
# binaries already in the bundle (Frameworks and PlugIns). This pulls in items like
# libdouble-conversion, fmt, tbb, openblas, etc., if they are needed.
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
            # If dependency is a Qt framework binary, copy the entire framework dir
            if [[ "$dep" == *"/Qt"*"framework/Versions/"*"/*" ]]; then
                fw_dir=$(echo "$dep" | sed -E 's#(Qt[^/]*\.framework)/.*#\1#')
                fw_name=$(basename "$fw_dir")
                if [ ! -d "$FRAMEWORKS_DIR/$fw_name" ] && [ -d "$QT_LIB_PATH/$fw_name" ]; then
                    echo "  Closure copy (Qt framework): $fw_name"
                    cp -R "$QT_LIB_PATH/$fw_name" "$FRAMEWORKS_DIR/"
                    chmod -R u+w "$FRAMEWORKS_DIR/$fw_name"
                    rm -rf "$FRAMEWORKS_DIR/$fw_name/Versions/A/Headers" "$FRAMEWORKS_DIR/$fw_name/Headers" 2>/dev/null || true
                    CHANGED=true
                fi
            else
                # Regular Homebrew dylib
                if [ ! -f "$FRAMEWORKS_DIR/$dep_name" ] && [ -f "$dep" ]; then
                    echo "  Closure copy: $dep_name (from $f)"
                    cp "$dep" "$FRAMEWORKS_DIR/"
                    chmod u+w "$FRAMEWORKS_DIR/$dep_name"
                    CHANGED=true
                fi
            fi
        done
    done
done

# Proactively include libdouble-conversion if present (used by Qt)
for dc in "$HOMEBREW_PREFIX"/lib/libdouble-conversion*.dylib; do
    [ -f "$dc" ] || continue
    dc_name=$(basename "$dc")
    if [ ! -f "$FRAMEWORKS_DIR/$dc_name" ]; then
        echo "  Copying $dc_name"
        cp "$dc" "$FRAMEWORKS_DIR/"
        chmod u+w "$FRAMEWORKS_DIR/$dc_name"
    fi
done

# If QtDBus is referenced by app or any plugin, ensure the framework is bundled
if otool -L "$APP_NAME.app/Contents/MacOS/$APP_NAME" | grep -q "$QT_LIB_PATH/QtDBus.framework"; then
    NEED_QTDBUS=1
elif find "$PLUGINS_DIR" -name "*.dylib" -maxdepth 2 -print0 2>/dev/null | xargs -0 otool -L 2>/dev/null | grep -q "$QT_LIB_PATH/QtDBus.framework"; then
    NEED_QTDBUS=1
else
    NEED_QTDBUS=0
fi
if [ "$NEED_QTDBUS" -eq 1 ] && [ -d "$QT_LIB_PATH/QtDBus.framework" ] && [ ! -d "$FRAMEWORKS_DIR/QtDBus.framework" ]; then
    echo "  Copying QtDBus.framework"
    cp -R "$QT_LIB_PATH/QtDBus.framework" "$FRAMEWORKS_DIR/"
    chmod -R u+w "$FRAMEWORKS_DIR/QtDBus.framework"
    rm -rf "$FRAMEWORKS_DIR/QtDBus.framework/Versions/A/Headers" "$FRAMEWORKS_DIR/QtDBus.framework/Headers" 2>/dev/null || true
fi

print_step "Step 4: Creating qt.conf"

cat > "$APP_NAME.app/Contents/Resources/qt.conf" << 'EOF'
[Paths]
Plugins = PlugIns
EOF

print_step "Step 5: Fixing Library Paths"

# Function to safely change library paths
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

# Fix main executable paths
echo "  Fixing main executable..."
# Fix Qt framework paths
for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
    old_path="$HOMEBREW_PREFIX/opt/qt/lib/$framework.framework/Versions/A/$framework"
    new_path="@executable_path/../Frameworks/$framework.framework/Versions/A/$framework"
    fix_lib_path "$old_path" "$new_path" "$APP_NAME.app/Contents/MacOS/$APP_NAME"
done

# Fix OpenCV library paths
for lib_path in $OPENCV_LIBS; do
    lib_name=$(basename "$lib_path")
    new_path="@executable_path/../Frameworks/$lib_name"
    fix_lib_path "$lib_path" "$new_path" "$APP_NAME.app/Contents/MacOS/$APP_NAME"
done

echo "  Fixing library IDs and dependencies..."
# Fix Qt framework IDs
for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
    if [ -f "$FRAMEWORKS_DIR/$framework.framework/Versions/A/$framework" ]; then
        install_name_tool -id "@executable_path/../Frameworks/$framework.framework/Versions/A/$framework" \
            "$FRAMEWORKS_DIR/$framework.framework/Versions/A/$framework" 2>/dev/null || true
    fi
done

# Fix OpenCV library IDs and their dependencies
for lib_file in "$FRAMEWORKS_DIR"/*.dylib; do
    if [ -f "$lib_file" ]; then
        lib_name=$(basename "$lib_file")

        # Fix library ID
        install_name_tool -id "@executable_path/../Frameworks/$lib_name" "$lib_file" 2>/dev/null || true

        # Fix dependencies on other homebrew libraries
        DEPS=$(otool -L "$lib_file" 2>/dev/null | grep "$HOMEBREW_PREFIX" | awk '{print $1}' || true)
        for dep_path in $DEPS; do
            dep_name=$(basename "$dep_path")
            if [ -f "$FRAMEWORKS_DIR/$dep_name" ]; then
                install_name_tool -change "$dep_path" "@executable_path/../Frameworks/$dep_name" "$lib_file" 2>/dev/null || true
            fi
        done

        # Fix Qt framework dependencies
        for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
            old_path="$HOMEBREW_PREFIX/opt/qt/lib/$framework.framework/Versions/A/$framework"
            new_path="@executable_path/../Frameworks/$framework.framework/Versions/A/$framework"
            fix_lib_path "$old_path" "$new_path" "$lib_file"
        done
    fi
done

# Fix Qt framework inter-dependencies
for framework1 in QtCore QtGui QtWidgets QtSvg QtDBus; do
    if [ -f "$FRAMEWORKS_DIR/$framework1.framework/Versions/A/$framework1" ]; then
        for framework2 in QtCore QtGui QtWidgets QtSvg QtDBus; do
            if [ "$framework1" != "$framework2" ]; then
                old_path="$HOMEBREW_PREFIX/opt/qt/lib/$framework2.framework/Versions/A/$framework2"
                new_path="@executable_path/../Frameworks/$framework2.framework/Versions/A/$framework2"
                fix_lib_path "$old_path" "$new_path" "$FRAMEWORKS_DIR/$framework1.framework/Versions/A/$framework1"
            fi
        done
    fi
done

# Fix Qt plugins
for plugin in "$PLUGINS_DIR"/*/*.dylib; do
    if [ -f "$plugin" ]; then
        for framework in QtCore QtGui QtWidgets QtSvg; do
            old_path="$HOMEBREW_PREFIX/opt/qt/lib/$framework.framework/Versions/A/$framework"
            new_path="@executable_path/../Frameworks/$framework.framework/Versions/A/$framework"
            fix_lib_path "$old_path" "$new_path" "$plugin"
        done
    fi
done

print_step "Step 6: Code Signing"

# Remove existing signatures
codesign --remove-signature "$APP_NAME.app" 2>/dev/null || true

# Sign all components
find "$FRAMEWORKS_DIR" -name "*.dylib" -exec codesign --force --sign - {} \; 2>/dev/null
find "$FRAMEWORKS_DIR" -name "Qt*" -type f -path "*/Versions/A/*" -exec codesign --force --sign - {} \; 2>/dev/null
find "$PLUGINS_DIR" -name "*.dylib" -exec codesign --force --sign - {} \; 2>/dev/null

# Sign main executable
codesign --force --sign - "$APP_NAME.app/Contents/MacOS/$APP_NAME"

# Sign entire bundle
codesign --force --sign - "$APP_NAME.app"

print_step "Step 7: Testing and Packaging"

# Basic verification
if codesign --verify "$APP_NAME.app" 2>/dev/null; then
    print_success "Code signature is valid"
else
    print_warning "Code signature verification failed, but app may still work"
fi

# Check for remaining external dependencies
EXTERNAL_DEPS=$(
    {
        otool -L "$APP_NAME.app/Contents/MacOS/$APP_NAME";
        for f in "$FRAMEWORKS_DIR"/*.dylib; do otool -L "$f"; done
        for f in "$FRAMEWORKS_DIR"/Qt*.framework/Versions/A/*; do [ -f "$f" ] && otool -L "$f"; done
        for p in "$PLUGINS_DIR"/*/*.dylib; do otool -L "$p"; done
    } 2>/dev/null | grep "$HOMEBREW_PREFIX" | grep -v '@executable_path' || true
)
if [ -z "$EXTERNAL_DEPS" ]; then
    print_success "All dependencies are bundled"
else
    print_warning "Some external dependencies remain"
fi

# Remove quarantine attribute
xattr -dr com.apple.quarantine "$APP_NAME.app" 2>/dev/null || true

# Create distribution archive
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
ARCHIVE_NAME="$APP_NAME-deployable-$TIMESTAMP.zip"
zip -r "$ARCHIVE_NAME" "$APP_NAME.app" >/dev/null

# Final summary
echo
print_success "Deployment Complete!"
echo "  App bundle: $PWD/$APP_NAME.app"
echo "  Archive: $PWD/$ARCHIVE_NAME"
echo "  Bundle size: $(du -sh "$APP_NAME.app" | cut -f1)"
echo "  Archive size: $(du -sh "$ARCHIVE_NAME" | cut -f1)"
echo
echo "To test locally:"
echo "  open $APP_NAME.app"
echo
echo "To distribute:"
echo "  1. Share the $ARCHIVE_NAME file"
echo "  2. Recipients should unzip and run:"
echo "     xattr -dr com.apple.quarantine $APP_NAME.app"
echo "     open $APP_NAME.app"
echo
print_success "Your YAWT app is ready for distribution!"
