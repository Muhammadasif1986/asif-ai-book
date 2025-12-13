#!/bin/bash

# Script to fix Docusaurus GitHub Pages deployment issues

echo "Fixing Docusaurus GitHub Pages deployment..."

# 1. Remove duplicate docs directory
echo "Removing duplicate docs directory..."
if [ -d "my-website/docs (2)" ]; then
    rm -rf "my-website/docs (2)"
    echo "✓ Removed duplicate docs directory"
else
    echo "✓ No duplicate docs directory found"
fi

# 2. Clean install dependencies
echo "Cleaning and reinstalling dependencies..."
cd my-website

# Remove node_modules if they exist
if [ -d "node_modules" ]; then
    rm -rf node_modules
    echo "✓ Removed node_modules"
fi

# Install dependencies
npm install
echo "✓ Dependencies installed"

# 3. Build the site
echo "Building Docusaurus site..."
npm run build
if [ $? -eq 0 ]; then
    echo "✓ Site built successfully"
else
    echo "✗ Site build failed"
    exit 1
fi

# 4. Verify the build output
if [ -d "build" ]; then
    echo "✓ Build directory created successfully"
else
    echo "✗ Build directory not found"
    exit 1
fi

echo "Fixes applied successfully! Your Docusaurus site should now work properly with GitHub Pages."