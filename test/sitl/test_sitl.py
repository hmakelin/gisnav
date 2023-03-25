#!/usr/bin/env python3
"""Module for running SITL test scripts"""
import argparse
import asyncio
import sys

from sitl_px4 import PX4TestEnvironment
from sitl_utils import SITLEnvironment, SITLTestError, logger

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("autopilot", help='Choose "px4" or "ardupilot".')
    args = vars(parser.parse_args())

    autopilot = args.get("autopilot", "px4")
    if autopilot == "px4":
        test_env = PX4TestEnvironment
    else:
        raise NotImplementedError(f"SITL test for {autopilot} not implemented.")

    assert issubclass(test_env, SITLEnvironment)
    with test_env():
        loop = asyncio.get_event_loop()
        try:
            loop.run_until_complete(test_env.run())
        except SITLTestError as e:
            logger.error(f"SITL test error: {e}")
            sys.exit(1)
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
            sys.exit(2)
        else:
            sys.exit(0)
