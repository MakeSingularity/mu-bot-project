#!/bin/bash
# Git commit script for Gazebo Garden migration
# This script commits all the changes related to the migration

echo "🚀 Committing Gazebo Garden Migration Changes"
echo "============================================"

# Add all the changed files
git add README.md
git add sim/launch/emu_gazebo_garden.launch.py
git add sim/worlds/emu_testing_world.sdf
git add scripts/setup_desktop.sh
git add scripts/setup_laptop.sh
git add scripts/test_gazebo_display.sh
git add docs/gazebo_garden_migration.md
git add docs/laptop_setup_guide.md

# Check what we're committing
echo "📋 Files to be committed:"
git status --porcelain

echo ""
echo "📝 Commit message:"
echo "feat: Complete migration to Gazebo Garden simulation environment

🎯 Major Changes:
- ✅ Migrated from Gazebo Classic to Gazebo Garden for future-proofing
- ✅ Created new launch file: sim/launch/emu_gazebo_garden.launch.py
- ✅ Added SDF world file: sim/worlds/emu_testing_world.sdf
- ✅ Updated setup scripts for automated Gazebo Garden installation
- ✅ Added display compatibility test: scripts/test_gazebo_display.sh

🔧 Infrastructure Updates:
- Updated README with Gazebo Garden migration information
- Modified setup_desktop.sh and setup_laptop.sh for new packages
- Created comprehensive migration guide documentation
- Added laptop-specific setup instructions

🐛 Fixes:
- Resolved WSL GUI compatibility issues with fallback options
- Fixed launch file path resolution and world file loading
- Added software rendering support for WSL environments

📚 Documentation:
- Added docs/gazebo_garden_migration.md with detailed migration steps
- Created docs/laptop_setup_guide.md for portable development
- Updated all example commands to use Gazebo Garden
- Added troubleshooting section for common WSL issues

🎉 Benefits:
- Long-term support (Gazebo Classic EOL January 2025)
- Better WSL compatibility with headless mode support
- Modern simulation architecture for future development
- Automated setup process for all environments

Breaking Changes: None - legacy files preserved for reference
Tested: ✅ Desktop WSL environment, headless and GUI modes working"

echo ""
read -p "Proceed with commit? (y/N): " confirm

if [[ $confirm =~ ^[Yy]$ ]]; then
    git commit -m "feat: Complete migration to Gazebo Garden simulation environment

🎯 Major Changes:
- ✅ Migrated from Gazebo Classic to Gazebo Garden for future-proofing
- ✅ Created new launch file: sim/launch/emu_gazebo_garden.launch.py
- ✅ Added SDF world file: sim/worlds/emu_testing_world.sdf
- ✅ Updated setup scripts for automated Gazebo Garden installation
- ✅ Added display compatibility test: scripts/test_gazebo_display.sh

🔧 Infrastructure Updates:
- Updated README with Gazebo Garden migration information
- Modified setup_desktop.sh and setup_laptop.sh for new packages
- Created comprehensive migration guide documentation
- Added laptop-specific setup instructions

🐛 Fixes:
- Resolved WSL GUI compatibility issues with fallback options
- Fixed launch file path resolution and world file loading
- Added software rendering support for WSL environments

📚 Documentation:
- Added docs/gazebo_garden_migration.md with detailed migration steps
- Created docs/laptop_setup_guide.md for portable development
- Updated all example commands to use Gazebo Garden
- Added troubleshooting section for common WSL issues

🎉 Benefits:
- Long-term support (Gazebo Classic EOL January 2025)
- Better WSL compatibility with headless mode support
- Modern simulation architecture for future development
- Automated setup process for all environments

Breaking Changes: None - legacy files preserved for reference
Tested: ✅ Desktop WSL environment, headless and GUI modes working"

    echo ""
    echo "✅ Committed successfully!"
    echo ""
    echo "📤 Ready to push to GitHub:"
    echo "   git push origin main"
    echo ""
    echo "🚀 For laptop setup, run on your laptop:"
    echo "   git pull origin main"
    echo "   ./scripts/setup_laptop.sh"
    echo "   ./scripts/test_gazebo_display.sh"
else
    echo "❌ Commit cancelled."
fi
