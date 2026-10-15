#!/usr/bin/env bash
set -eu
count=0
for block in /mnt/e/CODE/C++/Grinder/.planning/readme-standard/bash/*.sh; do
  bash -n "$block"
  count=$((count+1))
done
echo "Bash syntax checked: $count blocks; no deployment commands executed"
