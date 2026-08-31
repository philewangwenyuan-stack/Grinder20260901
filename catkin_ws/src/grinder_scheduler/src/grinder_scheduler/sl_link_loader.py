import os
import sys


def ensure_sl_linka_sdk_on_path(sdk_dir):
    if not sdk_dir:
        return
    sdk_dir = os.path.abspath(os.path.expanduser(sdk_dir))
    if sdk_dir not in sys.path:
        sys.path.insert(0, sdk_dir)
