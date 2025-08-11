#!/bin/bash

# Complete Build and Deploy Script for YAWT
# Builds the project and creates a distributable macOS app bundle

set -e  # Exit on any error

# Configuration
PROJECT_NAME="yawt"
BUILD_DIR="build"
DEPLOY_DIR="deploy"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
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

# Parse command line arguments
BUILD_TYPE="Release"
CLEAN_BUILD=false
SKIP_DEPLOY=false
SELF_SIGN=false
DEVELOPER_ID_SIGN=false
NOTARIZE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -n|--no-deploy)
            SKIP_DEPLOY=true
            shift
            ;;
        -s|--self-sign)
            SELF_SIGN=true
            shift
            ;;
        --developer-id)
            DEVELOPER_ID_SIGN=true
            shift
            ;;
        --notarize)
            NOTARIZE=true
            DEVELOPER_ID_SIGN=true  # Notarization requires Developer ID
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -d, --debug         Build in Debug mode (default: Release)"
            echo "  -c, --clean         Clean build directory before building"
            echo "  -n, --no-deploy     Skip deployment step"
            echo "  -s, --self-sign     Sign with self-signed certificate"
            echo "  --developer-id      Sign with Apple Developer ID"
            echo "  --notarize          Sign and notarize with Apple Developer ID"
            echo "  -h, --help          Show this help message"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

print_status "YAWT Build and Deploy Script"
echo "Build Type: $BUILD_TYPE"
echo "Clean Build: $CLEAN_BUILD"
echo "Skip Deploy: $SKIP_DEPLOY"
echo "Self Sign: $SELF_SIGN"
echo "Developer ID: $DEVELOPER_ID_SIGN"
echo "Notarize: $NOTARIZE"
echo

# Check prerequisites
print_status "Checking Prerequisites"

# Check if Qt is available
if ! command -v qtpaths &> /dev/null; then
    print_error "Qt not found. Please install Qt6 and ensure qtpaths is in PATH."
    exit 1
fi

QT_PREFIX=$(qtpaths --install-prefix)
print_success "Qt6 found at: $QT_PREFIX"

# Check if OpenCV is available
if [ ! -d "/opt/homebrew/lib" ] || [ ! -f "/opt/homebrew/lib/libopencv_core.dylib" ]; then
    print_error "OpenCV not found at /opt/homebrew. Please install OpenCV via Homebrew."
    exit 1
fi
print_success "OpenCV found at: /opt/homebrew"

# Check if macdeployqt is available
if ! command -v macdeployqt &> /dev/null; then
    print_error "macdeployqt not found. This is required for deployment."
    exit 1
fi
print_success "macdeployqt found"

# Clean build if requested
if [ "$CLEAN_BUILD" = true ]; then
    print_status "Cleaning Build Directory"
    rm -rf "$BUILD_DIR"
    print_success "Build directory cleaned"
fi

# Create build directory
print_status "Setting Up Build Directory"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake
print_status "Configuring with CMake ($BUILD_TYPE)"
cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_PREFIX_PATH="$QT_PREFIX" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

print_success "CMake configuration complete"

# Build the project
print_status "Building Project"
CPU_COUNT=$(sysctl -n hw.logicalcpu)
make -j"$CPU_COUNT"
print_success "Build complete"

# Verify the app was created
if [ ! -d "$PROJECT_NAME.app" ]; then
    print_error "$PROJECT_NAME.app not found in build directory"
    exit 1
fi

print_success "$PROJECT_NAME.app created successfully"

# Skip deployment if requested
if [ "$SKIP_DEPLOY" = true ]; then
    print_status "Skipping Deployment (--no-deploy specified)"
    print_success "Build complete. App available at: build/$PROJECT_NAME.app"
    exit 0
fi

# Deploy the application
print_status "Deploying Application"
cd ..

# Check if deployment script exists
if [ ! -f "deploy_mac.sh" ]; then
    print_error "deploy_mac.sh not found. Cannot proceed with deployment."
    exit 1
fi

# Run deployment
./deploy_mac.sh

print_success "Deployment complete"

# Create deploy directory and copy final app
print_status "Preparing Final Distribution"
mkdir -p "$DEPLOY_DIR"
cp -R "$BUILD_DIR/$PROJECT_NAME.app" "$DEPLOY_DIR/"

# Remove quarantine attribute from the deployed app
xattr -dr com.apple.quarantine "$DEPLOY_DIR/$PROJECT_NAME.app" 2>/dev/null || true

print_success "Distribution ready in $DEPLOY_DIR/"

# Code signing (if requested)
if [ "$SELF_SIGN" = true ] || [ "$DEVELOPER_ID_SIGN" = true ]; then
    print_status "Code Signing"

    if [ "$DEVELOPER_ID_SIGN" = true ]; then
        print_info "Using Apple Developer ID signing..."
        NOTARIZE_FLAG=""
        if [ "$NOTARIZE" = true ]; then
            NOTARIZE_FLAG="--notarize"
        fi
        ./sign_with_developer_id.sh $NOTARIZE_FLAG

        # Copy signed app to deploy directory
        cp -R "$BUILD_DIR/$PROJECT_NAME.app" "$DEPLOY_DIR/"
        print_success "Signed app copied to $DEPLOY_DIR/"

    elif [ "$SELF_SIGN" = true ]; then
        print_info "Using self-signed certificate..."
        ./create_self_signed_cert.sh

        # Copy signed app to deploy directory
        cp -R "$BUILD_DIR/$PROJECT_NAME.app" "$DEPLOY_DIR/"
        print_success "Self-signed app copied to $DEPLOY_DIR/"
    fi
fi

# Final summary
echo
print_status "Build and Deploy Summary"
echo "Build Type: $BUILD_TYPE"
echo "App Location: $DEPLOY_DIR/$PROJECT_NAME.app"
echo "Archive Location: $BUILD_DIR/$PROJECT_NAME-*.zip"
if [ "$SELF_SIGN" = true ]; then
    echo "Signing: Self-signed certificate"
elif [ "$DEVELOPER_ID_SIGN" = true ]; then
    echo "Signing: Apple Developer ID"
    if [ "$NOTARIZE" = true ]; then
        echo "Notarization: Enabled"
    fi
fi
echo
print_success "YAWT is ready for distribution!"
echo

if [ "$DEVELOPER_ID_SIGN" = true ]; then
    echo "Distribution (Developer ID signed):"
    echo "  • Share the .zip file from the build directory"
    echo "  • Users can double-click to run (no warnings)"
    echo "  • Ready for distribution outside App Store"
elif [ "$SELF_SIGN" = true ]; then
    echo "Distribution (Self-signed):"
    echo "  • Share the .zip file from the build directory"
    echo "  • Users should right-click → Open on first launch"
    echo "  • Some security warnings may appear"
else
    echo "To run locally:"
    echo "  open $DEPLOY_DIR/$PROJECT_NAME.app"
    echo
    echo "To distribute:"
    echo "  1. Share the .zip file from the build directory"
    echo "  2. Recipients should unzip and run:"
    echo "     xattr -dr com.apple.quarantine $PROJECT_NAME.app"
    echo
    echo "For better distribution, consider signing:"
    echo "  ./build_and_deploy.sh --self-sign        (free, some warnings)"
    echo "  ./build_and_deploy.sh --developer-id     (requires Apple Developer account)"
    echo "  ./build_and_deploy.sh --notarize         (best user experience)"
fi
