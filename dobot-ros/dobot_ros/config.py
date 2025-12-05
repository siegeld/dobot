"""
Configuration management for Dobot ROS2 CLI.
"""

import os
from pathlib import Path
from typing import Any, Optional

import yaml


class Config:
    """Configuration manager for Dobot ROS2 CLI."""

    DEFAULT_CONFIG_FILES = [
        'dobot_config.local.yaml',
        'dobot_config.yaml',
    ]

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize configuration.

        Args:
            config_path: Optional path to config file. If not provided,
                        searches for default config files.
        """
        self._config: dict = {}
        self._config_file: Optional[Path] = None

        if config_path:
            self._load_config(Path(config_path))
        else:
            self._find_and_load_config()

    def _find_and_load_config(self) -> None:
        """Find and load configuration file."""
        search_paths = [
            Path.cwd(),
            Path.home() / '.config' / 'dobot-ros',
            Path(__file__).parent.parent / 'config',
        ]

        for base_path in search_paths:
            for config_name in self.DEFAULT_CONFIG_FILES:
                config_file = base_path / config_name
                if config_file.exists():
                    self._load_config(config_file)
                    return

        # Use defaults if no config found
        self._config = self._get_defaults()

    def _load_config(self, path: Path) -> None:
        """Load configuration from file."""
        with open(path, 'r') as f:
            self._config = yaml.safe_load(f) or {}
        self._config_file = path

    def _get_defaults(self) -> dict:
        """Get default configuration."""
        return {
            'robot': {
                'ip': '192.168.1.6',
            },
            'ros': {
                'namespace': '',
                'service_timeout': 5.0,
            },
            'display': {
                'color': True,
                'format': 'table',
                'precision': 2,
            },
            'jog': {
                'default_distance_mm': 10,
                'default_rotation_deg': 5,
                'speed_percent': 50,
                'coordinate_mode': 'user',
            },
            'motion': {
                'sync_mode': False,  # True = wait for motion to complete
                'tolerance_deg': 0.5,
                'tolerance_mm': 1.0,
                'timeout': 30.0,
            },
            'logging': {
                'level': 'INFO',
            },
        }

    def get(self, key: str, default: Any = None) -> Any:
        """
        Get configuration value using dot notation.

        Args:
            key: Configuration key (e.g., 'robot.ip')
            default: Default value if key not found

        Returns:
            Configuration value
        """
        keys = key.split('.')
        value = self._config

        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default

        return value

    # Convenience properties
    @property
    def robot_ip(self) -> str:
        return self.get('robot.ip', '192.168.1.6')

    @property
    def ros_namespace(self) -> str:
        return self.get('ros.namespace', '')

    @property
    def service_timeout(self) -> float:
        return self.get('ros.service_timeout', 5.0)

    @property
    def output_format(self) -> str:
        return self.get('display.format', 'table')

    @property
    def precision(self) -> int:
        return self.get('display.precision', 2)

    @property
    def use_color(self) -> bool:
        return self.get('display.color', True)

    @property
    def jog_default_distance(self) -> float:
        return self.get('jog.default_distance_mm', 10)

    @property
    def jog_default_rotation(self) -> float:
        return self.get('jog.default_rotation_deg', 5)

    @property
    def jog_speed(self) -> int:
        return self.get('jog.speed_percent', 50)

    @property
    def jog_coordinate_mode(self) -> str:
        return self.get('jog.coordinate_mode', 'user')

    @property
    def sync_mode(self) -> bool:
        return self.get('motion.sync_mode', False)

    @property
    def motion_tolerance_deg(self) -> float:
        return self.get('motion.tolerance_deg', 0.5)

    @property
    def motion_tolerance_mm(self) -> float:
        return self.get('motion.tolerance_mm', 1.0)

    @property
    def motion_timeout(self) -> float:
        return self.get('motion.timeout', 30.0)

    @property
    def config_file(self) -> Optional[Path]:
        return self._config_file
