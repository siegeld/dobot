#!/bin/bash
# Development environment setup script for Dobot CR Controller

set -e

echo "🔧 Dobot CR Controller - Development Setup"
echo "==========================================="
echo ""

# Check Python version
echo "Checking Python version..."
python3 --version

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.7 or higher."
    exit 1
fi

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo ""
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo ""
echo "Upgrading pip..."
pip install --upgrade pip

# Install development dependencies
echo ""
echo "Installing development dependencies..."
pip install -r requirements-dev.txt

# Install package in editable mode
echo ""
echo "Installing dobot-cr-controller in development mode..."
pip install -e .

# Install pre-commit hooks (if available)
if command -v pre-commit &> /dev/null; then
    echo ""
    echo "Setting up pre-commit hooks..."
    pre-commit install
fi

# Verify installation
echo ""
echo "Verifying installation..."
dobot-cr --version

# Run basic checks
echo ""
echo "Running code quality checks..."

echo "  → Black (code formatting)..."
black --check dobot_cr/ || echo "    ⚠️  Run 'black dobot_cr/' to format code"

echo "  → Flake8 (linting)..."
flake8 dobot_cr/ || echo "    ⚠️  Fix linting issues"

echo "  → MyPy (type checking)..."
mypy dobot_cr/ || echo "    ⚠️  Fix type errors"

echo ""
echo "✅ Development environment setup complete!"
echo ""
echo "Development commands:"
echo "  black dobot_cr/          # Format code"
echo "  flake8 dobot_cr/         # Lint code"
echo "  mypy dobot_cr/           # Type check"
echo "  pytest                   # Run tests (when available)"
echo ""
echo "To activate the environment: source venv/bin/activate"
