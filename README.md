# IEEE 802.15.4 Sniffer for OpenMote-CC2538

This project provides a lossless sniffer for the IEEE 802.15.4 network which can be run on the [OpenMote-CC2538](http://www.openmote.com/hardware/openmote-cc2538-en.html) hardware.

## Setup
Download the source code and recursively update the submodules:
``` bash
git clone https://bitbucket.org/texus/openmote-cc2538-802.15.4-sniffer
git submodule update --init --recursive
```

Then patch the bsl script which was hardcoded to run only programs from inside the firmware folder:
``` bash
patch -p0 < bsl-patch.diff
```

Finally generate the libcc2538.a file:
``` bash
cd OpenMoteFirmware/platform/cc2538/libcc2538
python libcc2538.py
ls .. | grep libcc2538.a
```

The third command should have printed "libcc2538.a" which means that the file was successfully generated. If it doesn't print anything then something went wrong in the python script. Open libcc2538.py with a text editor and remove the "stderr=devnull" parameter that you find somewhere in it. Now run the python script again and fix the errors that you receive.


## Compiling and programming sniffer
Make sure the [OpenBase](http://www.openmote.com/hardware/openbase.html) is connected.

Run the following commands (from the sniffer directory) to compile the code and then program it to the connected OpenMote:
``` bash
make
sudo make bsl
```

## Running
``` bash
sudo python serial-connect.py
```

## Current modules
- sniffer  
    The actual sniffer program which is supposed to be lossless
- debug-receiver  
    Just show the received data in an unoptimized best-effort way
- custom-TX  
    Transmits packets with a sequence number one after the other
- max-radio-performance  
    Transmit packets as fast as possible by manually accessing radio and making various optimizations (max 1942 packets/sec)
