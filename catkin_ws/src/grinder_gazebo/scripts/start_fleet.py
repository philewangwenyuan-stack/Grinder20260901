#!/usr/bin/env python3
from grinder_gazebo.fleet_runner import main

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
