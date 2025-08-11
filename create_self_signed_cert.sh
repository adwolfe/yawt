#!/bin/bash

# Self-Signed Certificate Creation and Signing Script for YAWT
# Creates a self-signed certificate and signs the app bundle

set -e

# Configuration
APP_NAME="yawt"
CERT_NAME="YAWT Developer"
KEYCHAIN="login"  # or "System" for system keychain

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

print_status "Self-Signed Certificate Setup for YAWT"
echo
print_warning "This creates a self-signed certificate for development/testing only."
print_warning "Users will still see security warnings when running the app."
print_warning "For production distribution, use an Apple Developer certificate."
echo

# Check if certificate already exists
if security find-certificate -c "$CERT_NAME" >/dev/null 2>&1; then
    print_status "Certificate '$CERT_NAME' already exists"
    read -p "Do you want to recreate it? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_status "Deleting existing certificate..."
        security delete-certificate -c "$CERT_NAME" || true
        print_success "Existing certificate deleted"
    else
        print_status "Using existing certificate"
        CERT_EXISTS=true
    fi
fi

# Create self-signed certificate if it doesn't exist or was deleted
if [ "$CERT_EXISTS" != "true" ]; then
    print_status "Creating Self-Signed Certificate"

    # Create a temporary config file for the certificate
    TEMP_CONFIG=$(mktemp)
    cat > "$TEMP_CONFIG" << EOF
[ req ]
default_bits = 2048
distinguished_name = req_distinguished_name
req_extensions = v3_req
prompt = no

[ req_distinguished_name ]
C = US
ST = State
L = City
O = YAWT Development
OU = Development Team
CN = $CERT_NAME

[ v3_req ]
keyUsage = keyEncipherment, dataEncipherment, digitalSignature
extendedKeyUsage = codeSigning
EOF

    # Generate the certificate
    TEMP_CERT=$(mktemp)
    TEMP_KEY=$(mktemp)

    print_status "Generating certificate and key..."
    openssl req -new -x509 -days 365 -nodes \
        -config "$TEMP_CONFIG" \
        -keyout "$TEMP_KEY" \
        -out "$TEMP_CERT"

    # Create a p12 file to import into Keychain
    TEMP_P12=$(mktemp)
    openssl pkcs12 -export -nodes -passout pass: \
        -inkey "$TEMP_KEY" \
        -in "$TEMP_CERT" \
        -out "$TEMP_P12"

    # Import into Keychain
    print_status "Importing certificate into Keychain..."
    security import "$TEMP_P12" -k "$KEYCHAIN.keychain" -T /usr/bin/codesign

    # Clean up temporary files
    rm -f "$TEMP_CONFIG" "$TEMP_CERT" "$TEMP_KEY" "$TEMP_P12"

    print_success "Self-signed certificate '$CERT_NAME' created and imported"
fi

# Trust the certificate for code signing (required for self-signed certs)
print_status "Setting certificate trust policy..."
security add-trusted-cert -d -r trustRoot -k "$KEYCHAIN.keychain" -p codeSign \
    <(security find-certificate -c "$CERT_NAME" -p) 2>/dev/null || true

print_success "Certificate trust policy updated"

# Check if app bundle exists
if [ ! -d "build/$APP_NAME.app" ]; then
    print_error "App bundle not found at build/$APP_NAME.app"
    echo "Please build the app first using:"
    echo "  ./build_and_deploy.sh --no-deploy"
    exit 1
fi

# Sign the application
print_status "Signing Application Bundle"

# Remove any existing signatures first
print_status "Removing existing signatures..."
codesign --remove-signature "build/$APP_NAME.app" 2>/dev/null || true

# Sign all executables and libraries in the bundle
print_status "Signing bundle contents..."

# Sign frameworks and dylibs first
find "build/$APP_NAME.app" -name "*.dylib" -exec codesign --force --sign "$CERT_NAME" {} \;
find "build/$APP_NAME.app" -name "*.framework" -exec codesign --force --sign "$CERT_NAME" {} \;

# Sign Qt plugins
find "build/$APP_NAME.app/Contents/PlugIns" -name "*.dylib" -exec codesign --force --sign "$CERT_NAME" {} \; 2>/dev/null || true

# Sign the main executable
codesign --force --sign "$CERT_NAME" "build/$APP_NAME.app/Contents/MacOS/$APP_NAME"

# Sign the entire bundle with entitlements and hardened runtime
print_status "Signing main bundle..."

# Create entitlements file
ENTITLEMENTS_FILE="build/entitlements.plist"
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
</dict>
</plist>
EOF

# Final bundle signing
codesign --force --deep --sign "$CERT_NAME" \
    --entitlements "$ENTITLEMENTS_FILE" \
    --options runtime \
    "build/$APP_NAME.app"

print_success "Application bundle signed successfully"

# Verify the signature
print_status "Verifying Signature"
if codesign --verify --deep --strict "build/$APP_NAME.app"; then
    print_success "Signature verification passed"
else
    print_error "Signature verification failed"
    exit 1
fi

# Display signature information
print_status "Signature Information"
codesign -dv --verbose=4 "build/$APP_NAME.app"

echo
print_success "Self-Signed Code Signing Complete!"
echo
echo "Your app is now signed with a self-signed certificate."
echo
print_warning "Important Notes:"
echo "• Users will still see security warnings on first launch"
echo "• They need to right-click → Open (or go to System Preferences → Security)"
echo "• This is NOT suitable for App Store distribution"
echo "• For production, consider getting an Apple Developer certificate"
echo
echo "The signed app is ready at: build/$APP_NAME.app"
echo
echo "To distribute:"
echo "1. Create archive: cd build && zip -r $APP_NAME-signed.zip $APP_NAME.app"
echo "2. Share the zip file"
echo "3. Recipients should right-click the app and select 'Open'"
