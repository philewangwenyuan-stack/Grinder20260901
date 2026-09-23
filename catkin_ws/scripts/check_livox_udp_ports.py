#!/usr/bin/env python3
"""Fail before starting Livox when its configured host UDP ports are occupied."""

import json
import socket
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_NAME = "livox_ros_driver2"


def resolve_config_path(package_root, launch_path):
    launch_root = ET.parse(launch_path).getroot()
    params = [
        element
        for element in launch_root.iter("param")
        if element.attrib.get("name") == "user_config_path"
    ]
    if len(params) != 1:
        raise RuntimeError(
            "expected one user_config_path in {} (found {})".format(
                launch_path, len(params)
            )
        )

    configured_path = params[0].attrib.get("value", "").strip()
    package_macro = "$(find {})".format(PACKAGE_NAME)
    if package_macro in configured_path:
        configured_path = configured_path.replace(package_macro, str(package_root))
    if "$(" in configured_path:
        raise RuntimeError(
            "unsupported ROS substitution in user_config_path: {}".format(
                configured_path
            )
        )

    config_path = Path(configured_path)
    if not config_path.is_absolute():
        config_path = Path(package_root) / config_path
    if not config_path.is_file():
        raise RuntimeError("Livox config file does not exist: {}".format(config_path))
    return config_path


def collect_host_ports(value):
    ports = []
    if isinstance(value, dict):
        host_info = value.get("host_net_info")
        if isinstance(host_info, dict):
            host_entries = [host_info]
        elif isinstance(host_info, list):
            host_entries = [entry for entry in host_info if isinstance(entry, dict)]
        else:
            host_entries = []

        for host_entry in host_entries:
            for key, raw_port in host_entry.items():
                if not key.endswith("_port"):
                    continue
                try:
                    port = int(raw_port)
                except (TypeError, ValueError):
                    raise RuntimeError("invalid Livox host port {}={!r}".format(key, raw_port))
                if port == 0:
                    continue
                if not 1 <= port <= 65535:
                    raise RuntimeError("invalid Livox host UDP port {}={}".format(key, port))
                host_key = key[:-5] + "_ip"
                configured_host = str(
                    host_entry.get(host_key) or host_entry.get("host_ip") or ""
                ).strip()
                if not configured_host:
                    # Livox configs use an empty host IP to disable optional
                    # endpoints such as log_data, even when a port is listed.
                    continue
                host = configured_host
                ports.append((host, port, key))
        for key, nested_value in value.items():
            if key != "host_net_info":
                ports.extend(collect_host_ports(nested_value))
    elif isinstance(value, list):
        for item in value:
            ports.extend(collect_host_ports(item))
    return ports


def assert_ports_free(ports):
    unique_ports = {}
    for host, port, key in ports:
        unique_ports.setdefault((host, port), key)
    if not unique_ports:
        raise RuntimeError("Livox config has no non-zero host UDP ports to preflight")

    for (host, port), key in sorted(unique_ports.items()):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((host, port))
        except OSError as exc:
            raise RuntimeError(
                "Livox UDP port is unavailable: {}:{} ({}, {})".format(
                    host, port, key, exc
                )
            )
        finally:
            sock.close()
        print("[INFO] Livox UDP port is free: {}:{} ({})".format(host, port, key))


def main(argv):
    if len(argv) != 3:
        raise RuntimeError(
            "usage: check_livox_udp_ports.py <livox-package-root> <launch-file>"
        )
    package_root = Path(argv[1]).resolve()
    launch_path = Path(argv[2]).resolve()
    config_path = resolve_config_path(package_root, launch_path)
    with config_path.open("r", encoding="utf-8") as config_file:
        config = json.load(config_file)
    assert_ports_free(collect_host_ports(config))


if __name__ == "__main__":
    try:
        main(sys.argv)
    except (OSError, RuntimeError, ET.ParseError, json.JSONDecodeError) as exc:
        print("[ERROR] Livox port preflight failed: {}".format(exc), file=sys.stderr)
        sys.exit(1)
