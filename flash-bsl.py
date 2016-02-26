import subprocess
import sys

if len(sys.argv) != 2:
    print('Usage: python ' + sys.argv[0] + ' OpenMoteSniffer.hex')
    sys.exit()

subprocess.call([sys.executable, 'OpenMoteFirmware/tools/openmote-bsl/openmote-bsl.py', sys.argv[1], '--board=openbase'])
