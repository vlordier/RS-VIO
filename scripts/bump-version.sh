#!/usr/bin/env bash

# Version management script for RS-VIO
# Usage: ./scripts/bump-version.sh <patch|minor|major>

set -e

if [ $# -ne 1 ]; then
    echo "Usage: $0 <patch|minor|major>"
    exit 1
fi

VERSION_TYPE=$1

# Validate version type
if [[ ! "$VERSION_TYPE" =~ ^(patch|minor|major)$ ]]; then
    echo "Error: Version type must be patch, minor, or major"
    exit 1
fi

# Get current version
CURRENT_VERSION=$(grep '^version = ' Cargo.toml | sed 's/version = "\(.*\)"/\1/')
echo "Current version: $CURRENT_VERSION"

# Calculate new version
IFS='.' read -r major minor patch <<< "$CURRENT_VERSION"

case $VERSION_TYPE in
    patch)
        new_patch=$((patch + 1))
        NEW_VERSION="$major.$minor.$new_patch"
        ;;
    minor)
        new_minor=$((minor + 1))
        NEW_VERSION="$major.$new_minor.0"
        ;;
    major)
        new_major=$((major + 1))
        NEW_VERSION="$new_major.0.0"
        ;;
esac

echo "New version: $NEW_VERSION"

# Update Cargo.toml
sed -i.bak "s/^version = \"$CURRENT_VERSION\"/version = \"$NEW_VERSION\"/" Cargo.toml
rm Cargo.toml.bak

# Update CHANGELOG.md
TODAY=$(date +%Y-%m-%d)
sed -i.bak "s/## \[Unreleased\]/## [$NEW_VERSION] - $TODAY\n\n### Added\n- \n\n### Changed\n- \n\n### Fixed\n- \n\n## [Unreleased]/" CHANGELOG.md
rm CHANGELOG.md.bak

echo "Version bumped to $NEW_VERSION"
echo "Please update CHANGELOG.md with release notes"
echo "Then run: git add . && git commit -m \"Bump version to $NEW_VERSION\" && git tag v$NEW_VERSION"