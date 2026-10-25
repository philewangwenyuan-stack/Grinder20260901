#!/usr/bin/env python3
"""Validate public and optional device-local scheduler YAML before launch."""

import argparse
import math
from pathlib import Path
import sys

import yaml


def load_config(path):
    """Return a mapping, reporting locations rather than configuration values."""
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise ValueError("scheduler configuration file is missing: {}".format(config_path))
    try:
        with config_path.open("r", encoding="utf-8-sig") as handle:
            config = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        mark = getattr(exc, "problem_mark", None)
        location = ""
        if mark is not None:
            location = " at line {}, column {}".format(mark.line + 1, mark.column + 1)
        raise ValueError("invalid scheduler YAML{}: {}".format(location, config_path)) from None
    except (OSError, UnicodeError):
        raise ValueError("cannot read scheduler configuration: {}".format(config_path)) from None
    if not isinstance(config, dict):
        raise ValueError("scheduler configuration must be a YAML mapping: {}".format(config_path))
    if any(not isinstance(key, str) for key in config):
        raise ValueError("scheduler parameter names must be strings: {}".format(config_path))
    return config


def validate_configs(default_path, local_path=""):
    config = load_config(default_path)
    if local_path:
        config.update(load_config(local_path))
    for key in ("file_upload_on_map_save", "mqtt_enabled", "stream_enabled"):
        if key in config and not isinstance(config[key], bool):
            raise ValueError("scheduler parameter {} must be a YAML boolean".format(key))
    if "active_segment_point_spacing" in config:
        try:
            value = config["active_segment_point_spacing"]
            spacing = float(value)
            if isinstance(value, bool) or not math.isfinite(spacing) or spacing < 0.01:
                raise ValueError()
        except (TypeError, ValueError):
            raise ValueError("active_segment_point_spacing must be finite and >= 0.01 m") from None
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--default-config", required=True)
    parser.add_argument("--local-config", default="")
    args = parser.parse_args()
    try:
        validate_configs(args.default_config, args.local_config)
    except ValueError as exc:
        print("[ERROR] {}".format(exc), file=sys.stderr)
        return 1
    print("[INFO] scheduler configuration validated (local override: {})".format(
        "enabled" if args.local_config else "none"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
