#!/bin/bash

# Apple Developer ID Signing Script for YAWT
# Signs the app bundle with a proper Apple Developer ID certificate
# Requires: Apple Developer Program membership ($99/year)

set -e

# Configuration
APP_NAME="yawt"
BUILD_DIR="build"
NOTARIZE=false

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

print_info() {
    echo -e "${NC}$1${NC}"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--notarize)
            NOTARIZE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -n, --notarize   Also notarize the app (requires Apple ID app password)"
            echo "  -h, --help       Show this help message"
            echo
            echo "Prerequisites:"
            echo "  • Apple Developer Program membership"
            echo "  • Developer ID Application certificate in Keychain"
            echo "  • Built app at $BUILD_DIR/$APP_NAME.app"
            echo
            echo "For notarization, you also need:"
            echo "  • App-specific password stored in Keychain"
            echo "  • Run: xcrun notarytool store-credentials"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

print_status "Apple Developer ID Signing for YAWT"
echo "Notarization: $([ "$NOTARIZE" = true ] && echo "Enabled" || echo "Disabled")"
echo

# Check if app bundle exists
if [ ! -d "$BUILD_DIR/$APP_NAME.app" ]; then
    print_error "App bundle not found at $BUILD_DIR/$APP_NAME.app"
    echo "Please build the app first using:"
    echo "  ./build_and_deploy.sh --no-deploy"
    exit 1
fi

# Find Developer ID Application certificates
print_status "Finding Developer ID Certificates"

CERT_LIST=$(security find-identity -v -p codesigning | grep "Developer ID Application" | head -10)

if [ -z "$CERT_LIST" ]; then
    print_error "No Developer ID Application certificates found in Keychain"
    echo
    echo "To obtain a Developer ID certificate:"
    echo "1. Join Apple Developer Program: https://developer.apple.com/programs/"
    echo "2. In Xcode → Preferences → Accounts → Manage Certificates → Create Certificate"
    echo "3. Or download from developer.apple.com → Certificates"
    echo
    echo "For development/testing only, use:"
    echo "  ./create_self_signed_cert.sh"
    exit 1
fi

echo "Available Developer ID certificates:"
echo "$CERT_LIST"
echo

# Auto-select certificate or let user choose
CERT_COUNT=$(echo "$CERT_LIST" | wc -l | xargs)
if [ "$CERT_COUNT" -eq 1 ]; then
    CERT_HASH=$(echo "$CERT_LIST" | awk '{print $2}')
    CERT_NAME=$(echo "$CERT_LIST" | sed 's/.*) \(.*\) \([A-Z0-9]*\)$/\1/')
    print_success "Auto-selected certificate: $CERT_NAME"
else
    echo "Multiple certificates found. Please select one:"
    echo "$CERT_LIST" | nl
    echo
    read -p "Enter selection (1-$CERT_COUNT): " CERT_SELECTION

    if ! [[ "$CERT_SELECTION" =~ ^[1-9][0-9]*$ ]] || [ "$CERT_SELECTION" -gt "$CERT_COUNT" ]; then
        print_error "Invalid selection"
        exit 1
    fi

    SELECTED_LINE=$(echo "$CERT_LIST" | sed -n "${CERT_SELECTION}p")
    CERT_HASH=$(echo "$SELECTED_LINE" | awk '{print $2}')
    CERT_NAME=$(echo "$SELECTED_LINE" | sed 's/.*) \(.*\) \([A-Z0-9]*\)$/\1/')
    print_success "Selected certificate: $CERT_NAME"
fi

echo
print_status "Preparing for Code Signing"

# Remove any existing signatures
print_info "Removing existing signatures..."
codesign --remove-signature "$BUILD_DIR/$APP_NAME.app" 2>/dev/null || true

# Create entitlements file for hardened runtime
ENTITLEMENTS_FILE="$BUILD_DIR/entitlements.plist"
cat > "$ENTITLEMENTS_FILE" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <!-- Allow JIT compilation (needed for some Qt components) -->
    <key>com.apple.security.cs.allow-jit</key>
    <true/>

    <!-- Allow unsigned executable memory (needed for some frameworks) -->
    <key>com.apple.security.cs.allow-unsigned-executable-memory</key>
    <true/>

    <!-- Allow DYLD environment variables (for development) -->
    <key>com.apple.security.cs.allow-dyld-environment-variables</key>
    <true/>

    <!-- Disable library validation (needed for bundled libraries) -->
    <key>com.apple.security.cs.disable-library-validation</key>
    <true/>

    <!-- Allow loading of unsigned plugins -->
    <key>com.apple.security.cs.disable-executable-page-protection</key>
    <true/>
</dict>
</plist>
EOF

print_success "Entitlements file created"

# Sign all components in the correct order
print_status "Code Signing Process"

print_info "Step 1: Signing frameworks and libraries..."
# Sign all .dylib files
find "$BUILD_DIR/$APP_NAME.app" -name "*.dylib" -exec codesign \
    --force \
    --sign "$CERT_HASH" \
    --timestamp \
    --options runtime \
    {} \;

# Sign all .framework directories
find "$BUILD_DIR/$APP_NAME.app" -name "*.framework" -exec codesign \
    --force \
    --sign "$CERT_HASH" \
    --timestamp \
    --options runtime \
    {} \;

print_info "Step 2: Signing Qt plugins..."
# Sign Qt plugins if they exist
if [ -d "$BUILD_DIR/$APP_NAME.app/Contents/PlugIns" ]; then
    find "$BUILD_DIR/$APP_NAME.app/Contents/PlugIns" -name "*.dylib" -exec codesign \
        --force \
        --sign "$CERT_HASH" \
        --timestamp \
        --options runtime \
        {} \;
fi

print_info "Step 3: Signing main executable..."
codesign \
    --force \
    --sign "$CERT_HASH" \
    --timestamp \
    --options runtime \
    --entitlements "$ENTITLEMENTS_FILE" \
    "$BUILD_DIR/$APP_NAME.app/Contents/MacOS/$APP_NAME"

print_info "Step 4: Signing app bundle..."
codesign \
    --force \
    --sign "$CERT_HASH" \
    --timestamp \
    --options runtime \
    --entitlements "$ENTITLEMENTS_FILE" \
    "$BUILD_DIR/$APP_NAME.app"

print_success "Code signing complete"

# Verify signature
print_status "Verifying Signature"
if codesign --verify --deep --strict "$BUILD_DIR/$APP_NAME.app"; then
    print_success "Signature verification passed"
else
    print_error "Signature verification failed"
    exit 1
fi

# Display signature information
print_info "Signature details:"
codesign -dv --verbose=2 "$BUILD_DIR/$APP_NAME.app"

# Create signed archive
print_status "Creating Distribution Archive"
cd "$BUILD_DIR"
ARCHIVE_NAME="$APP_NAME-signed-$(date +%Y%m%d-%H%M%S).zip"
zip -r "$ARCHIVE_NAME" "$APP_NAME.app"
cd ..

print_success "Signed archive created: $BUILD_DIR/$ARCHIVE_NAME"

# Notarization (optional)
if [ "$NOTARIZE" = true ]; then
    print_status "Starting Notarization Process"

    print_warning "Notarization requires:"
    print_warning "1. App-specific password stored in Keychain"
    print_warning "2. Prior setup with: xcrun notarytool store-credentials"
    echo

    read -p "Do you want to continue with notarization? (y/N): " -n 1 -r
    echo

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Submitting app for notarization..."

        # Submit for notarization
        if xcrun notarytool submit "$BUILD_DIR/$ARCHIVE_NAME" \
            --keychain-profile "notarytool-profile" \
            --wait; then

            print_success "Notarization successful!"

            # Staple the notarization
            print_info "Stapling notarization to app bundle..."
            xcrun stapler staple "$BUILD_DIR/$APP_NAME.app"

            # Create final notarized archive
            cd "$BUILD_DIR"
            NOTARIZED_ARCHIVE="$APP_NAME-notarized-$(date +%Y%m%d-%H%M%S).zip"
            zip -r "$NOTARIZED_ARCHIVE" "$APP_NAME.app"
            cd ..

            print_success "Notarized archive created: $BUILD_DIR/$NOTARIZED_ARCHIVE"
        else
            print_error "Notarization failed"
            print_warning "The signed (but not notarized) app is still available"
        fi
    else
        print_info "Skipping notarization"
    fi
fi

# Final summary
echo
print_status "Code Signing Summary"
echo "Certificate: $CERT_NAME"
echo "Signed App: $BUILD_DIR/$APP_NAME.app"
echo "Archive: $BUILD_DIR/$ARCHIVE_NAME"
if [ "$NOTARIZE" = true ] && [ -n "$NOTARIZED_ARCHIVE" ]; then
    echo "Notarized Archive: $BUILD_DIR/$NOTARIZED_ARCHIVE"
fi
echo

print_success "YAWT is ready for distribution!"
echo
echo "Distribution options:"
echo "• Share the .zip archive with users"
echo "• Users can simply double-click to run (no security warnings)"
echo "• Suitable for distribution outside the App Store"
echo
echo "Next steps:"
echo "• Test on different macOS versions"
echo "• Consider App Store distribution for wider reach"
echo "• Set up automatic builds with GitHub Actions"

# Cleanup
rm -f "$ENTITLEMENTS_FILE"
