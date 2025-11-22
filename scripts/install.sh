#!/bin/bash
# Installation script for Dobot CR Controller

set -e

echo "🤖 Dobot CR Controller - Installation Script"
echo "=============================================="
echo ""

# Check Python version
echo "Checking Python version..."
python3 --version

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.7 or higher."
    exit 1
fi

# Create virtual environment
echo ""
echo "Creating virtual environment..."
python3 -m venv venv

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo ""
echo "Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo ""
echo "Installing dependencies..."
pip install -r requirements.txt

# Install package in editable mode
echo ""
echo "Installing dobot-cr-controller..."
pip install -e .

# Verify installation
echo ""
echo "Verifying installation..."
dobot-cr --version

echo ""
echo "✅ Installation complete!"
echo ""
echo "Next steps:"
echo "1. Activate the virtual environment: source venv/bin/activate"
echo "2. Edit dobot_config.yaml with your robot's IP address"
echo "3. Test connection: dobot-cr connect"
echo "4. Get position: dobot-cr position"
echo ""
echo "For help: dobot-cr --help"
