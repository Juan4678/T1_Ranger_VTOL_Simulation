"""Configuration helpers for experiment scripts and notebooks."""

from pathlib import Path
import json
from typing import Any, Dict


def load_json_config(config_path: str) -> Dict[str, Any]:
    """Load a JSON configuration file.

    Args:
        config_path: Path to the JSON file.

    Returns:
        Parsed configuration dictionary.
    """
    with Path(config_path).open("r", encoding="utf-8") as handle:
        return json.load(handle)
