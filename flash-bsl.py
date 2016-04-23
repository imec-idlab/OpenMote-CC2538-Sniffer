#!/usr/bin/env python

import subprocess
import sys
import os

bslScript = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), 'OpenMoteFirmware/tools/openmote-bsl/openmote-bsl.py')
hexFile = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), 'OpenMoteSniffer.hex')

subprocess.call([sys.executable, bslScript, hexFile, '--board=openbase'])
