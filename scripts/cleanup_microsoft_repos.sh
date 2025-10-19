#!/bin/bash

# Microsoft Repository Cleanup Script
# Use this if you encounter conflicts with existing Microsoft repositories

echo "🧹 Cleaning up Microsoft repository conflicts..."

# Remove any existing Microsoft VS Code repositories and keys
echo "Removing existing Microsoft repositories and keys..."

sudo rm -f /etc/apt/sources.list.d/vscode.list
sudo rm -f /etc/apt/sources.list.d/microsoft*.list
sudo rm -f /etc/apt/trusted.gpg.d/packages.microsoft.gpg
sudo rm -f /etc/apt/trusted.gpg.d/microsoft.gpg
sudo rm -f /usr/share/keyrings/microsoft.gpg
sudo rm -f /usr/share/keyrings/packages.microsoft.gpg

# Clean up any temporary GPG files
rm -f packages.microsoft.gpg

echo "✅ Cleanup complete!"
echo ""
echo "Now you can safely run your setup script:"
echo "  ./scripts/setup_desktop.sh"
echo "  ./scripts/setup_laptop.sh"
echo "  ./scripts/setup_pi.sh"
echo ""
echo "The setup scripts will properly install VS Code with correct repository configuration."
