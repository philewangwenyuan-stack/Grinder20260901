"""Stable identity for immutable Super-LIO map bundles."""

import hashlib
import os


ASSET_FILES = ("loc_map.pcd", "plan_map.pcd", "map.yaml", "map.pgm")


def compute_map_asset_revision(bundle_dir):
    """Return a SHA-256 revision over the bundle's immutable runtime assets."""
    digest = hashlib.sha256()
    root = os.path.abspath(str(bundle_dir))
    for filename in ASSET_FILES:
        path = os.path.join(root, filename)
        if not os.path.isfile(path):
            raise RuntimeError("map asset missing: {}".format(path))
        digest.update(filename.encode("ascii"))
        digest.update(b"\0")
        with open(path, "rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
    return digest.hexdigest()


def get_or_compute_map_asset_revision(bundle_dir):
    """Read a persisted revision or compute one for a legacy immutable bundle."""
    manifest_path = os.path.join(os.path.abspath(str(bundle_dir)), "map_info.json")
    try:
        import json

        with open(manifest_path, "r", encoding="utf-8") as handle:
            manifest = json.load(handle)
        revision = str(manifest.get("assetRevision", "") or "").strip().lower()
        if len(revision) == 64 and all(ch in "0123456789abcdef" for ch in revision):
            return revision
    except Exception:
        pass
    return compute_map_asset_revision(bundle_dir)

