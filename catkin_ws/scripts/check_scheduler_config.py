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


def validate_channels(config):
    for key, default in (("sl_linka_port", 8002), ("sl_linka_control_port", 8003)):
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 65535:
            raise ValueError("{} must be an integer in 1..65535".format(key))
    if config.get("sl_linka_port", 8002) == config.get("sl_linka_control_port", 8003):
        raise ValueError("SL-LinkA data and control ports must differ")
    for key, default in (("sl_linka_session_timeout_sec", 3.0),
                         ("sl_linka_bulk_bytes_per_sec", 1048576),
                         ("sl_linka_bulk_manual_bytes_per_sec", 262144),
                         ("sl_linka_bulk_burst_bytes", 8192)):
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("{} must be finite and positive".format(key))
    return config


def validate_configs(default_path, local_path=""):
    config = load_config(default_path)
    if local_path:
        config.update(load_config(local_path))
    validate_channels(config)
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
    parser.add_argument("--print-ports", action="store_true", help="Print merged data/control ports for startup")
    args = parser.parse_args()
    try:
        config = validate_configs(args.default_config, args.local_config)
    except ValueError as exc:
        print("[ERROR] {}".format(exc), file=sys.stderr)
        return 1
    if args.print_ports:
        print("{} {}".format(config.get("sl_linka_port", 8002), config.get("sl_linka_control_port", 8003)))
        return 0
    print("[INFO] scheduler configuration validated (local override: {})".format(
        "enabled" if args.local_config else "none"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
