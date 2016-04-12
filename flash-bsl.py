import subprocess
import sys
import os

if len(sys.argv) != 2:
    print('Usage: python ' + sys.argv[0] + ' OpenMoteSniffer.hex')
    sys.exit()

subprocess.call([sys.executable, os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), 'OpenMoteFirmware/tools/openmote-bsl/openmote-bsl.py'), sys.argv[1], '--board=openbase'])
