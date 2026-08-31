import argparse
import asyncio
from .simulator import SlLinkSimulator, pb


def main():
    parser = argparse.ArgumentParser(description="SL-LinkA protocol simulator (single lower device)")
    parser.add_argument("--port", type=int, default=8002, help="Listening port")
    args = parser.parse_args()

    try:
        simulator = SlLinkSimulator(pb.DEVICE_LOWER)
        asyncio.run(simulator.start_server(port=args.port))
    except KeyboardInterrupt:
        print("\nShutdown simulator.")


if __name__ == "__main__":
    main()
