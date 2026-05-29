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
USE_MACDEPLOYQT=true
MACDEPLOYQT_VERBOSE=2
CREATE_ARCHIVE=true

# Ensure common Homebrew paths are available when running non-interactive.
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_step() { echo -e "${BLUE}=== $1 ===${NC}"; }
print_success() { echo -e "${GREEN}✓ $1${NC}"; }
print_warning() { echo -e "${YELLOW}⚠ $1${NC}"; }
print_error() { echo -e "${RED}✗ $1${NC}"; }
print_info() { echo -e "[`date +%H:%M:%S`] $1"; }
trap 'print_error "Failed at line $LINENO: $BASH_COMMAND"' ERR

run_timed() {
    local label="$1"
    shift
    local start_ts end_ts elapsed
    start_ts=$(date +%s)
    print_info "START: $label"
    "$@"
    end_ts=$(date +%s)
    elapsed=$((end_ts - start_ts))
    print_info "DONE: $label (${elapsed}s)"
}

usage() {
    cat << EOF
Usage: $0 [options]

Options:
  -d, --debug          Build Debug (default: Release)
  -c, --clean          Clean build directory before building
  -n, --no-build       Skip build step (assumes build/yawt.app exists)
  --developer-id       Sign with Apple Developer ID if available
  --no-macdeployqt     Skip macdeployqt (use manual Qt bundling)
  --mdqt-verbose N     macdeployqt verbosity (0-3, default: 2)
  --no-archive         Do not create a timestamped zip archive
  -h, --help           Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--debug) BUILD_TYPE="Debug"; shift ;;
        -c|--clean) CLEAN_BUILD=true; shift ;;
        -n|--no-build) SKIP_BUILD=true; shift ;;
        --developer-id) USE_DEVELOPER_ID=true; shift ;;
        --no-macdeployqt) USE_MACDEPLOYQT=false; shift ;;
        --no-archive) CREATE_ARCHIVE=false; shift ;;
        --mdqt-verbose)
            if [ $# -lt 2 ] || ! [[ "$2" =~ ^[0-3]$ ]]; then
                print_error "--mdqt-verbose requires a value 0..3"
                exit 1
            fi
            MACDEPLOYQT_VERBOSE="$2"
            shift 2
            ;;
        -h|--help) usage; exit 0 ;;
        *) print_error "Unknown option: $1"; usage; exit 1 ;;
    esac
done

print_step "YAWT macOS Packaging"
echo "Build Type: $BUILD_TYPE"
echo "Clean Build: $CLEAN_BUILD"
echo "Skip Build: $SKIP_BUILD"
echo "Developer ID Signing: $USE_DEVELOPER_ID"
echo "Use macdeployqt: $USE_MACDEPLOYQT"
echo "macdeployqt verbosity: $MACDEPLOYQT_VERBOSE"
echo "Create archive: $CREATE_ARCHIVE"
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
    run_timed "cmake configure" cmake .. -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DCMAKE_PREFIX_PATH="$QT_PREFIX"
    run_timed "build (make)" make -j"$(sysctl -n hw.logicalcpu)"
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

if $USE_MACDEPLOYQT; then
    print_step "Running macdeployqt"
    if [ -n "${MACDEPLOYQT_BIN:-}" ] && [ -x "$MACDEPLOYQT_BIN" ]; then
        true
    elif [ -x "/opt/homebrew/bin/macdeployqt" ]; then
        MACDEPLOYQT_BIN="/opt/homebrew/bin/macdeployqt"
    elif [ -x "/usr/local/bin/macdeployqt" ]; then
        MACDEPLOYQT_BIN="/usr/local/bin/macdeployqt"
    elif command -v macdeployqt >/dev/null 2>&1; then
        MACDEPLOYQT_BIN="macdeployqt"
    else
        if command -v qtpaths >/dev/null 2>&1; then
            QT_PREFIX="$(qtpaths --install-prefix)"
        elif command -v qtpaths6 >/dev/null 2>&1; then
            QT_PREFIX="$(qtpaths6 --install-prefix)"
        else
            QT_PREFIX="$HOMEBREW_PREFIX/opt/qt"
        fi
        MACDEPLOYQT_BIN="$QT_PREFIX/bin/macdeployqt"
    fi
    if [ ! -x "$MACDEPLOYQT_BIN" ]; then
        print_error "macdeployqt not found (tried: $MACDEPLOYQT_BIN)"
        exit 1
    fi
    echo "Using macdeployqt: $MACDEPLOYQT_BIN"
    if command -v stdbuf >/dev/null 2>&1; then
        stdbuf -oL -eL "$MACDEPLOYQT_BIN" "$BUILD_DIR/$APP_NAME.app" -verbose="$MACDEPLOYQT_VERBOSE" 2>&1 | \
            while IFS= read -r line; do
                printf '[%s] [macdeployqt] %s\n' "$(date +%H:%M:%S)" "$line"
            done
    else
        "$MACDEPLOYQT_BIN" "$BUILD_DIR/$APP_NAME.app" -verbose="$MACDEPLOYQT_VERBOSE" 2>&1 | \
            while IFS= read -r line; do
                printf '[%s] [macdeployqt] %s\n' "$(date +%H:%M:%S)" "$line"
            done
    fi
fi

print_step "Bundling Dependencies"
print_info "Preparing Frameworks and PlugIns directories"

pushd "$BUILD_DIR" >/dev/null

FRAMEWORKS_DIR="$APP_NAME.app/Contents/Frameworks"
PLUGINS_DIR="$APP_NAME.app/Contents/PlugIns"
if ! $USE_MACDEPLOYQT; then
    print_info "Cleaning stale bundled Frameworks and PlugIns before manual deployment"
    chmod -R u+w "$FRAMEWORKS_DIR" "$PLUGINS_DIR" 2>/dev/null || true
    xattr -rc "$FRAMEWORKS_DIR" "$PLUGINS_DIR" 2>/dev/null || true
    rm -rf "$FRAMEWORKS_DIR" "$PLUGINS_DIR"
fi
mkdir -p "$FRAMEWORKS_DIR" "$PLUGINS_DIR/platforms" "$PLUGINS_DIR/imageformats"

if command -v qtpaths >/dev/null 2>&1; then
    QT_PREFIX="$(qtpaths --install-prefix)"
elif command -v qtpaths6 >/dev/null 2>&1; then
    QT_PREFIX="$(qtpaths6 --install-prefix)"
else
    QT_PREFIX="$HOMEBREW_PREFIX/opt/qt"
fi

QT_BASE_PREFIX=""
if command -v brew >/dev/null 2>&1; then
    QT_BASE_PREFIX="$(brew --prefix qtbase 2>/dev/null || true)"
fi
if [ -z "$QT_BASE_PREFIX" ]; then
    if [ -d "$HOMEBREW_PREFIX/opt/qtbase" ]; then
        QT_BASE_PREFIX="$HOMEBREW_PREFIX/opt/qtbase"
    fi
fi

QT_LIB_PATH="$QT_PREFIX/lib"
if ! $USE_MACDEPLOYQT; then
    print_info "Manual Qt framework copy enabled"
    for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
        if [ -d "$QT_LIB_PATH/$framework.framework" ]; then
            print_info "Copy framework: $framework.framework"
            framework_source="$QT_LIB_PATH/$framework.framework"
            if command -v realpath >/dev/null 2>&1; then
                framework_source=$(realpath "$framework_source")
            fi
            cp -R "$framework_source" "$FRAMEWORKS_DIR/"
            chmod -R u+w "$FRAMEWORKS_DIR/$framework.framework"
            rm -rf "$FRAMEWORKS_DIR/$framework.framework/Versions/A/Headers" 2>/dev/null || true
            rm -rf "$FRAMEWORKS_DIR/$framework.framework/Headers" 2>/dev/null || true
            rm -f "$FRAMEWORKS_DIR/$framework.framework/Versions/A"/*.prl 2>/dev/null || true
        else
            print_warning "$framework.framework not found"
        fi
    done
fi

QT_PLUGINS_PATH=""
for candidate in \
    "$QT_PREFIX/plugins" \
    "$QT_PREFIX/lib/qt6/plugins" \
    "$QT_PREFIX/share/qt/plugins" \
    "$HOMEBREW_PREFIX/lib/qt6/plugins" \
    "$HOMEBREW_PREFIX/share/qt/plugins"; do
    if [ -d "$candidate" ]; then
        QT_PLUGINS_PATH="$candidate"
        break
    fi
done
if ! $USE_MACDEPLOYQT; then
    if [ -f "$QT_PLUGINS_PATH/platforms/libqcocoa.dylib" ]; then
        print_info "Copy plugin: platforms/libqcocoa.dylib"
        cp "$QT_PLUGINS_PATH/platforms/libqcocoa.dylib" "$PLUGINS_DIR/platforms/"
    fi
    for plugin in libqjpeg.dylib libqpng.dylib libqgif.dylib libqico.dylib libqsvg.dylib; do
        if [ -f "$QT_PLUGINS_PATH/imageformats/$plugin" ]; then
            print_info "Copy plugin: imageformats/$plugin"
            cp "$QT_PLUGINS_PATH/imageformats/$plugin" "$PLUGINS_DIR/imageformats/"
        fi
    done
fi

OPENCV_LIBS=$(otool -L "$APP_NAME.app/Contents/MacOS/$APP_NAME" | grep "$HOMEBREW_PREFIX.*opencv" | awk '{print $1}' || true)
print_info "Copying OpenCV libraries directly referenced by executable"
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

RPATH_OPENCV_LIBS=$(otool -L "$APP_NAME.app/Contents/MacOS/$APP_NAME" | awk '{print $1}' | grep '^@rpath/libopencv_.*\.dylib$' || true)
if [ -n "$RPATH_OPENCV_LIBS" ]; then
    print_info "Resolving @rpath OpenCV libraries referenced by executable"
    for rpath_dep in $RPATH_OPENCV_LIBS; do
        dep_name=$(basename "$rpath_dep")
        dep_source=""
        if [ -n "$OPENCV_LIB_DIR" ] && [ -f "$OPENCV_LIB_DIR/$dep_name" ]; then
            dep_source="$OPENCV_LIB_DIR/$dep_name"
        elif [ -f "$HOMEBREW_PREFIX/lib/$dep_name" ]; then
            dep_source="$HOMEBREW_PREFIX/lib/$dep_name"
        fi
        if [ -n "$dep_source" ] && [ ! -f "$FRAMEWORKS_DIR/$dep_name" ]; then
            cp "$dep_source" "$FRAMEWORKS_DIR/"
            chmod u+w "$FRAMEWORKS_DIR/$dep_name"
            print_info "Copied resolved OpenCV lib: $dep_name"
        elif [ ! -f "$FRAMEWORKS_DIR/$dep_name" ]; then
            print_warning "Could not resolve OpenCV dependency: $dep_name"
        fi
    done
fi

BUNDLED_PATH_OPENCV_LIBS=$(otool -L "$APP_NAME.app/Contents/MacOS/$APP_NAME" | awk '{print $1}' | grep '^@executable_path/../Frameworks/libopencv_.*\.dylib$' || true)
if [ -n "$BUNDLED_PATH_OPENCV_LIBS" ]; then
    print_info "Refreshing OpenCV libraries already referenced through bundled paths"
    for bundled_dep in $BUNDLED_PATH_OPENCV_LIBS; do
        dep_name=$(basename "$bundled_dep")
        dep_source=""
        if [ -n "$OPENCV_LIB_DIR" ] && [ -f "$OPENCV_LIB_DIR/$dep_name" ]; then
            dep_source="$OPENCV_LIB_DIR/$dep_name"
        elif [ -f "$HOMEBREW_PREFIX/lib/$dep_name" ]; then
            dep_source="$HOMEBREW_PREFIX/lib/$dep_name"
        fi
        if [ -n "$dep_source" ] && [ ! -f "$FRAMEWORKS_DIR/$dep_name" ]; then
            cp "$dep_source" "$FRAMEWORKS_DIR/"
            chmod u+w "$FRAMEWORKS_DIR/$dep_name"
            print_info "Copied bundled-path OpenCV lib: $dep_name"
        elif [ ! -f "$FRAMEWORKS_DIR/$dep_name" ]; then
            print_warning "Could not refresh bundled-path OpenCV dependency: $dep_name"
        fi
    done
fi

CHANGED=true
PASS=0
while $CHANGED && [ $PASS -lt 5 ]; do
    CHANGED=false
    PASS=$((PASS+1))
    print_info "Dependency closure pass $PASS"
    for f in "$APP_NAME.app/Contents/MacOS/$APP_NAME" "$FRAMEWORKS_DIR"/*.dylib "$FRAMEWORKS_DIR"/Qt*.framework/Versions/A/* "$PLUGINS_DIR"/*/*.dylib; do
        [ -f "$f" ] || continue
        DEPS=$(otool -L "$f" 2>/dev/null | awk '{print $1}' | grep -E "^($HOMEBREW_PREFIX|$QT_PREFIX|$QT_BASE_PREFIX)/" || true)
        for dep in $DEPS; do
            dep_name=$(basename "$dep")
            if [[ "$dep" == *"/Qt"*"framework/Versions/"*"/*" ]]; then
                fw_dir=$(echo "$dep" | sed -E 's#(Qt[^/]*\.framework)/.*#\1#')
                fw_name=$(basename "$fw_dir")
                if [ ! -d "$FRAMEWORKS_DIR/$fw_name" ] && [ -d "$QT_LIB_PATH/$fw_name" ]; then
                    framework_source="$QT_LIB_PATH/$fw_name"
                    if command -v realpath >/dev/null 2>&1; then
                        framework_source=$(realpath "$framework_source")
                    fi
                    cp -R "$framework_source" "$FRAMEWORKS_DIR/"
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

cat > "$APP_NAME.app/Contents/Resources/qt.conf" << 'EOF'
[Paths]
Plugins = PlugIns
EOF

print_step "Fixing Library Paths"
print_info "Rewriting install names and rpaths"
fix_start_ts=$(date +%s)

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

rewrite_dependency_to_bundle() {
    local dep_path="$1"
    local target="$2"

    [ -f "$target" ] && [ -w "$target" ] || return

    if [[ "$dep_path" == @rpath/Qt*.framework/Versions/*/* ]]; then
        local framework_name framework_binary
        framework_name=$(echo "$dep_path" | sed -E 's#@rpath/(Qt[^/]+)\.framework/Versions/[^/]+/.*#\1#')
        framework_binary="$FRAMEWORKS_DIR/$framework_name.framework/Versions/A/$framework_name"
        if [ -f "$framework_binary" ]; then
            install_name_tool -change "$dep_path" "@executable_path/../Frameworks/$framework_name.framework/Versions/A/$framework_name" "$target" 2>/dev/null || true
        fi
        return 0
    fi

    if [[ "$dep_path" == @executable_path/../Frameworks/Qt* ]] && [[ "$dep_path" != *".framework/"* ]]; then
        local framework_name framework_binary
        framework_name=$(basename "$dep_path")
        framework_binary="$FRAMEWORKS_DIR/$framework_name.framework/Versions/A/$framework_name"
        if [ -f "$framework_binary" ]; then
            install_name_tool -change "$dep_path" "@executable_path/../Frameworks/$framework_name.framework/Versions/A/$framework_name" "$target" 2>/dev/null || true
        fi
        return 0
    fi

    if [[ "$dep_path" == @rpath/* ]]; then
        local dep_name
        dep_name=$(basename "$dep_path")
        if [ -f "$FRAMEWORKS_DIR/$dep_name" ]; then
            install_name_tool -change "$dep_path" "@executable_path/../Frameworks/$dep_name" "$target" 2>/dev/null || true
        fi
        return
    fi

    [[ "$dep_path" == /* ]] || return 0

    local framework_path
    for framework_path in "$FRAMEWORKS_DIR"/*.framework; do
        [ -d "$framework_path" ] || continue
        local framework_name framework_binary
        framework_name=$(basename "$framework_path" .framework)
        framework_binary="$FRAMEWORKS_DIR/$framework_name.framework/Versions/A/$framework_name"
        if [[ "$dep_path" == *"/$framework_name.framework/"* ]] && [ -f "$framework_binary" ]; then
            install_name_tool -change "$dep_path" "@executable_path/../Frameworks/$framework_name.framework/Versions/A/$framework_name" "$target" 2>/dev/null || true
            return
        fi
    done

    local dep_name
    dep_name=$(basename "$dep_path")
    if [ -f "$FRAMEWORKS_DIR/$dep_name" ]; then
        install_name_tool -change "$dep_path" "@executable_path/../Frameworks/$dep_name" "$target" 2>/dev/null || true
    fi

    return 0
}

remove_top_level_qt_binaries() {
    local framework_name
    for framework_name in QtCore QtGui QtWidgets QtSvg QtDBus; do
        rm -f "$FRAMEWORKS_DIR/$framework_name"
    done
}

patch_target_to_bundled_deps() {
    local target="$1"
    [ -f "$target" ] || return

    local deps dep_path
    deps=$(otool -L "$target" 2>/dev/null | awk 'NR > 1 {print $1}' || true)
    for dep_path in $deps; do
        rewrite_dependency_to_bundle "$dep_path" "$target"
    done

    return 0
}

verify_no_external_qt_links() {
    local offenders=""
    local target refs
    for target in "$APP_NAME.app/Contents/MacOS/$APP_NAME" "$FRAMEWORKS_DIR"/*.dylib "$FRAMEWORKS_DIR"/*.framework/Versions/A/* "$PLUGINS_DIR"/*/*.dylib; do
        [ -f "$target" ] || continue
        refs=$(otool -L "$target" 2>/dev/null | awk 'NR > 1 {print $1}' | grep -E '((/opt/homebrew|/usr/local).*/Qt[^/]*\.framework|@executable_path/\.\./Frameworks/Qt[^/]+$)' || true)
        if [ -n "$refs" ]; then
            offenders="${offenders}${target}\n${refs}\n"
        fi
    done

    if [ -n "$offenders" ]; then
        print_error "Bundled app still references external Homebrew Qt frameworks"
        printf "%b" "$offenders"
        exit 1
    fi

    return 0
}

install_name_tool -add_rpath "@executable_path/../Frameworks" "$APP_NAME.app/Contents/MacOS/$APP_NAME" 2>/dev/null || true
print_info "Patching executable link paths"
EXECUTABLE_PATH="$APP_NAME.app/Contents/MacOS/$APP_NAME"
EXECUTABLE_DEPS=$(otool -L "$EXECUTABLE_PATH" 2>/dev/null | awk 'NR > 1 {print $1}' || true)
for dep_path in $EXECUTABLE_DEPS; do
    rewrite_dependency_to_bundle "$dep_path" "$EXECUTABLE_PATH"
done
for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
    old_path="$QT_LIB_PATH/$framework.framework/Versions/A/$framework"
    new_path="@executable_path/../Frameworks/$framework.framework/Versions/A/$framework"
    fix_lib_path "$old_path" "$new_path" "$APP_NAME.app/Contents/MacOS/$APP_NAME"
    if [ -n "$QT_BASE_PREFIX" ]; then
        old_path="$QT_BASE_PREFIX/lib/$framework.framework/Versions/A/$framework"
        fix_lib_path "$old_path" "$new_path" "$APP_NAME.app/Contents/MacOS/$APP_NAME"
    fi
done
for lib_path in $OPENCV_LIBS; do
    lib_name=$(basename "$lib_path")
    fix_lib_path "$lib_path" "@executable_path/../Frameworks/$lib_name" "$APP_NAME.app/Contents/MacOS/$APP_NAME"
done

framework_bin_total=0
for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
    if [ -f "$FRAMEWORKS_DIR/$framework.framework/Versions/A/$framework" ]; then
        framework_bin_total=$((framework_bin_total + 1))
    fi
done
print_info "Setting install ids for $framework_bin_total Qt framework binaries"
for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
    if [ -f "$FRAMEWORKS_DIR/$framework.framework/Versions/A/$framework" ]; then
        install_name_tool -id "@executable_path/../Frameworks/$framework.framework/Versions/A/$framework" \
            "$FRAMEWORKS_DIR/$framework.framework/Versions/A/$framework" 2>/dev/null || true
    fi
done

print_info "Setting install ids for all bundled framework binaries"
for framework_path in "$FRAMEWORKS_DIR"/*.framework; do
    [ -d "$framework_path" ] || continue
    framework_name=$(basename "$framework_path" .framework)
    framework_binary="$FRAMEWORKS_DIR/$framework_name.framework/Versions/A/$framework_name"
    if [ -f "$framework_binary" ]; then
        install_name_tool -id "@executable_path/../Frameworks/$framework_name.framework/Versions/A/$framework_name" \
            "$framework_binary" 2>/dev/null || true
    fi
done

lib_total=$(find "$FRAMEWORKS_DIR" -maxdepth 1 -name "*.dylib" | wc -l | xargs)
lib_idx=0
print_info "Patching $lib_total framework dylibs"
for lib_file in "$FRAMEWORKS_DIR"/*.dylib; do
    [ -f "$lib_file" ] || continue
    lib_idx=$((lib_idx + 1))
    if [ $((lib_idx % 10)) -eq 0 ] || [ "$lib_idx" -eq 1 ] || [ "$lib_idx" -eq "$lib_total" ]; then
        print_info "Framework dylib progress: $lib_idx/$lib_total ($(basename "$lib_file"))"
    fi
    lib_name=$(basename "$lib_file")
    install_name_tool -id "@executable_path/../Frameworks/$lib_name" "$lib_file" 2>/dev/null || true
    DEPS=$(otool -L "$lib_file" 2>/dev/null | awk '{print $1}' | grep -E "^($HOMEBREW_PREFIX|$QT_PREFIX|$QT_BASE_PREFIX)/" || true)
    for dep_path in $DEPS; do
        dep_name=$(basename "$dep_path")
        if [ -f "$FRAMEWORKS_DIR/$dep_name" ]; then
            install_name_tool -change "$dep_path" "@executable_path/../Frameworks/$dep_name" "$lib_file" 2>/dev/null || true
        fi
    done
    for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
        old_path="$QT_LIB_PATH/$framework.framework/Versions/A/$framework"
        new_path="@executable_path/../Frameworks/$framework.framework/Versions/A/$framework"
        fix_lib_path "$old_path" "$new_path" "$lib_file"
        if [ -n "$QT_BASE_PREFIX" ]; then
            old_path="$QT_BASE_PREFIX/lib/$framework.framework/Versions/A/$framework"
            fix_lib_path "$old_path" "$new_path" "$lib_file"
        fi
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

plugin_total=$(find "$PLUGINS_DIR" -name "*.dylib" | wc -l | xargs)
plugin_idx=0
print_info "Patching $plugin_total Qt plugins"
for plugin in "$PLUGINS_DIR"/*/*.dylib; do
    [ -f "$plugin" ] || continue
    plugin_idx=$((plugin_idx + 1))
    if [ $((plugin_idx % 5)) -eq 0 ] || [ "$plugin_idx" -eq 1 ] || [ "$plugin_idx" -eq "$plugin_total" ]; then
        print_info "Plugin progress: $plugin_idx/$plugin_total ($(basename "$plugin"))"
    fi
    for framework in QtCore QtGui QtWidgets QtSvg QtDBus; do
        old_path="$QT_LIB_PATH/$framework.framework/Versions/A/$framework"
        new_path="@executable_path/../Frameworks/$framework.framework/Versions/A/$framework"
        fix_lib_path "$old_path" "$new_path" "$plugin"
        if [ -n "$QT_BASE_PREFIX" ]; then
            old_path="$QT_BASE_PREFIX/lib/$framework.framework/Versions/A/$framework"
            fix_lib_path "$old_path" "$new_path" "$plugin"
        fi
    done
    DEPS=$(otool -L "$plugin" 2>/dev/null | awk '{print $1}' | grep -E "^($HOMEBREW_PREFIX|$QT_PREFIX|$QT_BASE_PREFIX)/" || true)
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
fix_end_ts=$(date +%s)
print_info "Completed library path fixes in $((fix_end_ts - fix_start_ts))s"

print_step "Verifying Bundled Qt Links"
print_info "Rewriting any remaining absolute or @rpath dependency references"
for target in "$APP_NAME.app/Contents/MacOS/$APP_NAME" "$FRAMEWORKS_DIR"/*.dylib "$FRAMEWORKS_DIR"/*.framework/Versions/A/* "$PLUGINS_DIR"/*/*.dylib; do
    [ -f "$target" ] || continue
    patch_target_to_bundled_deps "$target"
done
remove_top_level_qt_binaries
verify_no_external_qt_links

print_step "Signing"
print_info "Removing extended attributes from app bundle"
xattr -rc "$APP_NAME.app" 2>/dev/null || true

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
    dylib_count=$(find "$FRAMEWORKS_DIR" -name "*.dylib" | wc -l | xargs)
    qt_bin_count=$(find "$FRAMEWORKS_DIR" -name "Qt*" -type f -path "*/Versions/A/*" | wc -l | xargs)
    plugin_count=$(find "$PLUGINS_DIR" -name "*.dylib" | wc -l | xargs)
    print_info "Ad-hoc signing $dylib_count framework dylibs"
    find "$FRAMEWORKS_DIR" -name "*.dylib" -exec codesign --force --sign - {} \; 2>/dev/null
    print_info "Ad-hoc signing $qt_bin_count Qt framework binaries"
    find "$FRAMEWORKS_DIR" -name "Qt*" -type f -path "*/Versions/A/*" -exec codesign --force --sign - {} \; 2>/dev/null
    print_info "Ad-hoc signing $plugin_count plugin dylibs"
    find "$PLUGINS_DIR" -name "*.dylib" -exec codesign --force --sign - {} \; 2>/dev/null
    run_timed "codesign app executable" codesign --force --sign - "$APP_NAME.app/Contents/MacOS/$APP_NAME"
    run_timed "codesign app bundle" codesign --force --sign - "$APP_NAME.app"
    run_timed "deep codesign app bundle" codesign --force --sign - --deep "$APP_NAME.app"
else
    dylib_count=$(find "$APP_NAME.app" -name "*.dylib" | wc -l | xargs)
    framework_count=$(find "$APP_NAME.app" -name "*.framework" | wc -l | xargs)
    print_info "Developer ID signing $dylib_count dylibs"
    find "$APP_NAME.app" -name "*.dylib" -exec codesign --force --sign "$SIGN_ID" --timestamp --options runtime {} \;
    print_info "Developer ID signing $framework_count frameworks"
    find "$APP_NAME.app" -name "*.framework" -exec codesign --force --sign "$SIGN_ID" --timestamp --options runtime {} \;
    if [ -n "$ENTITLEMENTS_FILE" ]; then
        run_timed "codesign app executable (entitlements)" codesign --force --sign "$SIGN_ID" --timestamp --options runtime --entitlements "$ENTITLEMENTS_FILE" "$APP_NAME.app/Contents/MacOS/$APP_NAME"
        run_timed "codesign app bundle (entitlements)" codesign --force --sign "$SIGN_ID" --timestamp --options runtime --entitlements "$ENTITLEMENTS_FILE" "$APP_NAME.app"
    else
        run_timed "codesign app executable" codesign --force --sign "$SIGN_ID" --timestamp --options runtime "$APP_NAME.app/Contents/MacOS/$APP_NAME"
        run_timed "codesign app bundle" codesign --force --sign "$SIGN_ID" --timestamp --options runtime "$APP_NAME.app"
    fi
fi

print_step "Verifying Signature"
print_info "Running strict signature verification"
xattr -rc "$APP_NAME.app" 2>/dev/null || true
if ! codesign --verify --deep --strict --verbose=2 "$APP_NAME.app"; then
    print_error "Code signing verification failed"
    exit 1
fi

print_step "Packaging"
xattr -rc "$APP_NAME.app" 2>/dev/null || true
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
ARCHIVE_NAME="$APP_NAME-macos-$TIMESTAMP.zip"
if $CREATE_ARCHIVE; then
    if command -v ditto >/dev/null 2>&1; then
        # Preserve symlinks and bundle metadata so signatures remain valid after transfer.
        run_timed "archive app (ditto)" ditto -c -k --sequesterRsrc --keepParent "$APP_NAME.app" "$ARCHIVE_NAME"
    else
        # Fallback: preserve symlinks if ditto is unavailable.
        run_timed "archive app (zip fallback)" zip -yr "$ARCHIVE_NAME" "$APP_NAME.app" >/dev/null
    fi
else
    print_info "Archive creation skipped"
fi

if [ -n "$ENTITLEMENTS_FILE" ]; then
    rm -f "$ENTITLEMENTS_FILE"
fi

print_success "Done"
echo "App bundle: $PWD/$APP_NAME.app"
if $CREATE_ARCHIVE; then
    echo "Archive: $PWD/$ARCHIVE_NAME"
fi

popd >/dev/null
