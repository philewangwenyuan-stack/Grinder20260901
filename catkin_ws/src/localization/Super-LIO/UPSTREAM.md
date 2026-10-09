# Super-LIO source provenance

- Upstream: https://github.com/Liansheng-Wang/Super-LIO.git
- Upstream checkout: `239984eb8cff0ba5efdf999e9b90d02b2c7d5c5e`
- License: GPL-3.0, preserved in `LICENSE`; retain third-party notices in the source.
- Imported into the Grinder root repository on 2026-09-28 as ordinary files.

This snapshot includes the existing Grinder/MID-360S integration changes from
the local checkout, including relocation/map services, ROS diagnostics,
configuration, launch files and RViz settings. It is not an unmodified upstream
checkout. Existing tracked files and newly added source/documentation files were
preserved during import; build trees, caches and local analysis outputs are not
distributed as source.
The import also declares the existing `basic` dependency in the Super-LIO
package/export metadata so Catkin dependency selection includes that package.

There is no submodule initialization step. Use Git from the Grinder root for
future changes and compare upstream updates against the checkout above.
The former nested Git history remains in an ignored `.git.nested-backup-*`
directory on the migration workstation; it is not required to build a fresh clone.
