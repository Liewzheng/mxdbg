"""This is the command line interface
"""

import argparse


def entry() -> None:
    parser = argparse.ArgumentParser(description="mxESP32DriverPC CLI Tool")
    
    parser.add_argument("--version", action="store_true", help="Print the version of the tool")

    return None

if __name__ == "__main__":
    entry()
    exit(0)
