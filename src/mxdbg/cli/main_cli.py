
import argparse
import webbrowser

from mxdbg.__version__ import __version__
from mxdbg.mxdbg import MXDBG


def entry() -> int:
    parser = argparse.ArgumentParser(description="mxDBG CLI Tool", add_help=False)
    
    parser.add_argument("-h", "--help", action="store_true", help="Show this help message and exit")
    parser.add_argument("-m", "--manual", action="store_true", help="Show the manual and exit")
    parser.add_argument("-v", "--version", action="store_true", help="Show version number")
    parser.add_argument("-r", "--restart", action="store_true", help="Restart the debugger")
    
    args = parser.parse_args()
    
    if args.help:
        parser.print_help()
        return 0
        
    if args.version:
        print(f"mxDBG {__version__}")
            
    if args.manual:
        url = "https://baidu.com"
        print(f"Opening manual: {url}")
        webbrowser.open(url)
        return 0
        
    if args.restart:
        print("Restarting debugger...")
        
        try:
            dev = MXDBG()
        except Exception as e:
            print(f"Failed to restart debugger: {e}")
            return 1
        
        dev.restart()
        return 0
    
    return 0

if __name__ == "__main__":
    exit(entry())
