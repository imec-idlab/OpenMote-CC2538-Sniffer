#!/usr/bin/env python

import sys
import platform

platform = platform.system()

if __name__ == "__main__":
    if platform == 'Windows':
        import sniffer_windows as sniffer
    elif platform == 'Linux' or platform == 'Darwin':
        import sniffer_unix as sniffer
    else:
        raise RuntimeError('Unsupported OS')

    sniffer.main(sys.argv[1:])
