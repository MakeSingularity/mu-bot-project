# VS Code Configuration for MU-Bot

This directory contains VS Code configuration files for a unified development environment across desktop, laptop, and Raspberry Pi platforms.

## Files

- **`settings.json`** - Workspace settings configured for ROS 2 Jazzy and Ubuntu 24.04
- **`launch.json`** - Debug configurations for Python ROS nodes and AI components
- **`mu-bot.code-workspace`** - Workspace file with tasks and extension recommendations

## Key Features

### GitHub Copilot Integration
- **GitHub Copilot** for AI-powered code completion
- **GitHub Copilot Chat** for conversational coding assistance
- Configured for Python, C++, YAML, and ROS launch files

### ROS 2 Jazzy Support
- Python paths configured for ROS 2 Jazzy packages
- Environment variables set for ROS development
- File associations for ROS file types (launch.py, urdf, xacro, sdf)

### Cross-Platform Development
- **Remote SSH** extensions for Pi development from desktop/laptop
- Consistent configuration across all environments
- Unified task definitions for building and testing

### Development Tools
- **Python** support with Black formatting and Flake8 linting
- **C++** support for ROS nodes and hardware interfaces
- **CMake** and **YAML** support
- **Jupyter** notebooks for AI development

## Usage

### Open Workspace
```bash
# From project root
code mu-bot.code-workspace
```

### Remote Development
1. Install VS Code on all platforms (done by setup scripts)
2. Use Remote-SSH to connect from desktop/laptop to Pi
3. All configurations sync automatically

### Debugging ROS Nodes
- Use "Python: ROS 2 Node" configuration for ROS nodes
- Environment variables automatically set for ROS
- Integrated terminal with proper Python paths

### Extensions
All recommended extensions are automatically installed by setup scripts:
- GitHub Copilot & Chat
- Python tools (Black, Flake8, Pylint)
- C++ tools
- Remote development tools
- ROS file type support

## Customization

### Adding Extensions
Edit `extensions.recommendations` in `settings.json` or the workspace file.

### Modifying Tasks
Edit the `tasks` section in `mu-bot.code-workspace` to add custom build/test tasks.

### Debug Configurations
Modify `launch.json` to add new debug configurations for specific nodes or tests.

## Network Development

When developing across devices (desktop ↔ laptop ↔ Pi):

1. Use Remote-SSH for code editing on Pi
2. ROS topics work transparently across network (with proper ROS_DOMAIN_ID)
3. VS Code tasks can run on any connected device
4. GitHub Copilot suggestions work on all platforms

## Troubleshooting

### Python Extension Issues
If Python debugging doesn't work:
```bash
# Ensure Python extension is installed
code --install-extension ms-python.python --force
```

### Remote SSH Connection Issues
```bash
# Generate SSH key if needed
ssh-keygen -t ed25519 -C "your_email@example.com"
ssh-copy-id pi@<pi_ip_address>
```

### GitHub Copilot Not Working
1. Ensure you have a GitHub Copilot subscription
2. Sign in to GitHub in VS Code: `Ctrl+Shift+P` → "GitHub: Sign in"
3. Authorize VS Code access to your GitHub account
