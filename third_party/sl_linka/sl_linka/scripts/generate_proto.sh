#!/bin/bash
# Generate protobuf code for all platforms
# Usage: ./scripts/generate_proto.sh [c|python|kotlin|all]
#
# Generated files go to each SDK's message_gen/ directory:
#   - sdk/embedded/message_gen/     (C/nanopb)
#   - sdk/python/sl_link/message_gen/ (Python)
#   - sdk/android/src/message_gen/  (Kotlin)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source files
PROTO_FILE="$PROJECT_ROOT/proto/sl_link.proto"
NANOPB_DIR="$PROJECT_ROOT/nanopb"

# Protoc binary (v25.1)
# We use protoc 25.x specifically because:
#   - Generates Python code compatible with both protobuf 4.x and 6.x
#   - Later versions (26+) generate code that requires protobuf 6.x runtime
# Download: https://github.com/protocolbuffers/protobuf/releases/tag/v25.1
#   - Linux: protoc-25.1-linux-x86_64.zip
#   - macOS: protoc-25.1-osx-universal_binary.zip
# Extract to: tools/protoc/
PROTOC="$PROJECT_ROOT/tools/protoc/bin/protoc"

# Output directories (inside each SDK's message_gen/)
C_OUT="$PROJECT_ROOT/sdk/embedded/message_gen"
PYTHON_OUT="$PROJECT_ROOT/sdk/python/sl_link/message_gen"
KOTLIN_OUT="$PROJECT_ROOT/sdk/android/src/message_gen"

# Check if local protoc exists, fallback to system protoc
if [ ! -x "$PROTOC" ]; then
    echo "Warning: tools/protoc/bin/protoc not found, using system protoc"
    PROTOC="protoc"
fi

generate_c() {
    echo "=== Generating C/nanopb code ==="
    mkdir -p "$C_OUT"
    
    python3 "$NANOPB_DIR/generator/nanopb_generator.py" \
        -I"$PROJECT_ROOT/proto" \
        -D"$C_OUT" \
        "$PROTO_FILE"
    
    echo "Generated: sdk/embedded/message_gen/sl_link.pb.c, sl_link.pb.h"
}

generate_python() {
    echo "=== Generating Python code ==="
    echo "Using protoc: $PROTOC"
    mkdir -p "$PYTHON_OUT"
    
    # Use protoc 25.x for compatibility with protobuf 4.x and 6.x
    "$PROTOC" \
        -I"$PROJECT_ROOT/proto" \
        -I"$PROJECT_ROOT/tools/protoc/include" \
        --python_out="$PYTHON_OUT" \
        "$PROTO_FILE"

    echo "Generated: sdk/python/sl_link/message_gen/sl_link_pb2.py"
}

generate_kotlin() {
    echo "=== Generating Kotlin/Android code ==="
    echo "Using protoc: $PROTOC"
    
    # Clean and create output directory
    rm -rf "$KOTLIN_OUT/sl_link"
    mkdir -p "$KOTLIN_OUT"
    
    # protoc 3.6.x does not support '--java_out=lite' or '--kotlin_out'.
    # Fallback to Java generation only, which is sufficient for current Android SDK usage.
    if "$PROTOC" --version | grep -q "libprotoc 3.6"; then
        "$PROTOC" \
            -I"$PROJECT_ROOT/proto" \
            -I"$PROJECT_ROOT/tools/protoc/include" \
            --java_out="$KOTLIN_OUT" \
            "$PROTO_FILE"
        echo "Generated: sdk/android/src/message_gen/sl_link/*.java"
    else
        "$PROTOC" \
            -I"$PROJECT_ROOT/proto" \
            -I"$PROJECT_ROOT/tools/protoc/include" \
            --java_out=lite:"$KOTLIN_OUT" \
            --kotlin_out=lite:"$KOTLIN_OUT" \
            "$PROTO_FILE"
        echo "Generated: sdk/android/src/message_gen/sl_link/*.kt, *.java"
    fi
}

# Parse arguments
case "${1:-all}" in
    c)
        generate_c
        ;;
    python)
        generate_python
        ;;
    kotlin|android)
        generate_kotlin
        ;;
    all)
        generate_c
        generate_python
        generate_kotlin
        ;;
    *)
        echo "Usage: $0 [c|python|kotlin|all]"
        echo "  c       - Generate C/nanopb code to sdk/embedded/message_gen/"
        echo "  python  - Generate Python code to sdk/python/sl_link/message_gen/"
        echo "  kotlin  - Generate Kotlin code to sdk/android/src/message_gen/"
        echo "  all     - Generate all (default)"
        exit 1
        ;;
esac

echo ""
echo "=== Generation complete ==="
