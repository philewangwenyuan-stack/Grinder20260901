# Third-Party Dependencies

This directory is the canonical location for external dependencies used by Grinder.

## Canonical Paths

- `SL-LinkA`: `third_party/sl_linka`
- `Path Planner`: `third_party/path_planner/mst25.py`

## Scheduler Defaults

The scheduler package defaults are already wired to this directory:

- `planner_script_path: third_party/path_planner/mst25.py`
- `sl_linka_python_sdk_dir: third_party/sl_linka/sdk/python`

## Migration Note

Legacy copies under `doc/` are temporarily kept for compatibility.
New integrations should always reference `third_party/`.
