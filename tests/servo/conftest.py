"""Fixtures for servo-tester tests. Reuses the MockRosClient from VLA tests."""

import sys
from pathlib import Path

import pytest

# Make the VLA conftest's MockRosClient available without a package marker.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vla"))

from conftest import MockRosClient  # noqa: E402


@pytest.fixture
def mock_ros():
    return MockRosClient()
