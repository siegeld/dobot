"""
Configuration management for Dobot CR Controller.
"""

import os
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


class Config:
    """Manages configuration loading and access."""

    DEFAULT_CONFIG_NAME = "dobot_config.yaml"
    LOCAL_CONFIG_NAME = "dobot_config.local.yaml"

    def __init__(self, config_path: Optional[Path] = None) -> None:
        """
        Initialize configuration.

        Args:
            config_path: Optional path to config file. If not provided, searches in:
                         1. dobot_config.local.yaml (gitignored, user-specific)
                         2. dobot_config.yaml (default, version controlled)
        """
        self._config: Dict[str, Any] = {}
        self._load_config(config_path)

    def _find_config_file(self) -> Path:
        """Find the configuration file to use."""
        cwd = Path.cwd()

        # First, check for local config (takes precedence)
        local_config = cwd / self.LOCAL_CONFIG_NAME
        if local_config.exists():
            return local_config

        # Fall back to default config
        default_config = cwd / self.DEFAULT_CONFIG_NAME
        if default_config.exists():
            return default_config

        raise FileNotFoundError(
            f"No configuration file found. Expected '{self.DEFAULT_CONFIG_NAME}' "
            f"or '{self.LOCAL_CONFIG_NAME}' in current directory."
        )

    def _load_config(self, config_path: Optional[Path]) -> None:
        """Load configuration from YAML file."""
        if config_path is None:
            config_path = self._find_config_file()

        with open(config_path, "r") as f:
            self._config = yaml.safe_load(f) or {}

    def get(self, key: str, default: Any = None) -> Any:
        """
        Get configuration value using dot notation.

        Args:
            key: Configuration key (e.g., 'robot.ip')
            default: Default value if key not found

        Returns:
            Configuration value or default
        """
        keys = key.split(".")
        value = self._config

        for k in keys:
            if isinstance(value, dict):
                value = value.get(k)
            else:
                return default

            if value is None:
                return default

        return value

    @property
    def robot_ip(self) -> str:
        """Get robot IP address."""
        ip = self.get("robot.ip")
        if not ip:
            raise ValueError("Robot IP address not configured in dobot_config.yaml")
        return ip

    @property
    def control_port(self) -> int:
        """Get control port."""
        return self.get("robot.control_port", 29999)

    @property
    def feedback_port(self) -> int:
        """Get feedback port."""
        return self.get("robot.feedback_port", 30004)

    @property
    def timeout(self) -> int:
        """Get connection timeout."""
        return self.get("robot.timeout", 5)

    @property
    def use_color(self) -> bool:
        """Get color output preference."""
        return self.get("display.color", True)

    @property
    def output_format(self) -> str:
        """Get output format."""
        return self.get("display.format", "table")

    @property
    def precision(self) -> int:
        """Get coordinate precision."""
        return self.get("display.precision", 2)

    @property
    def log_level(self) -> str:
        """Get logging level."""
        return self.get("logging.level", "INFO")

    @property
    def jog_default_distance(self) -> float:
        """Get default jog distance for linear movements (mm)."""
        return self.get("jog.default_distance_mm", 10.0)

    @property
    def jog_default_rotation(self) -> float:
        """Get default jog distance for rotations (degrees)."""
        return self.get("jog.default_rotation_deg", 5.0)

    @property
    def jog_speed(self) -> int:
        """Get jog movement speed (percentage 1-100)."""
        return self.get("jog.speed_percent", 50)

    @property
    def jog_coordinate_mode(self) -> str:
        """Get default jog coordinate mode ('user' or 'tool')."""
        mode = self.get("jog.coordinate_mode", "user")
        if mode not in ("user", "tool"):
            return "user"
        return mode
