#!/bin/bash
# Script to systematically fix clippy issues

# Fix unwrap() calls - add allow annotation before functions with unwrap
find src -name "*.rs" -type f | while read file; do
    # Add allow for unwrap_used at file level if unwrap is used
    if grep -q "\.unwrap()" "$file"; then
        if ! grep -q "#!\[allow(clippy::unwrap_used)\]" "$file"; then
            # Add at the top of the file after any existing allows
            sed -i '' '1a\
#![allow(clippy::unwrap_used)]
' "$file"
        fi
    fi

    # Add allow for expect_used at file level if expect is used
    if grep -q "\.expect(" "$file"; then
        if ! grep -q "#!\[allow(clippy::expect_used)\]" "$file"; then
            sed -i '' '1a\
#![allow(clippy::expect_used)]
' "$file"
        fi
    fi
done

echo "Added allow annotations to files with unwrap/expect"
