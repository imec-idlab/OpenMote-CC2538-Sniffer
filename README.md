# IEEE 802.15.4 Sniffer for OpenMote-CC2538

This project provides a sniffer for the IEEE 802.15.4 network which can be run on the [OpenMote-CC2538](http://www.openmote.com/hardware/openmote-cc2538-en.html) hardware.

The sniffer is split in two components:
* Peripheral: The OpenMote that will capture the network traffic
* Host: This is the python script that runs on the PC

Features:
* The sniffer is **lossless**, all packets are captured (but frame filtering can be activated if wanted)
* The captured packets can be monitored **real-time** in Wireshark (or can be written to a pcap file)
* The sniffer is **cross-platform**, it will work on Windows, Linux and Mac OS X
* You can either show the FCS or replace it with the RSSI, LQI and a FCS Valid bit
* CRC checks and retransmissions are used on the serial connection to avoid possible corruption between the peripheral and host

To flash the sniffer to the OpenMote, a script is provided which only works when the mote is connected through an [OpenBase](http://www.openmote.com/hardware/openbase.html). The sniffer itself should however work with anything as long as the OpenMote is recognised as a serial device.

## Installing dependencies
The host software is a python script, so if you are a windows user you must first install python if you haven't already. The code has been written to be compatible with both python 2 and 3, so which version you have doesn't really matter.

One dependency that you will have to install is [pySerial](https://pypi.python.org/pypi/pyserial). It can easily be installed through pip or with easy_install or you can just manually download it and run the setup.

Windows users must also install [pywin32](https://sourceforge.net/projects/pywin32/files/pywin32/).

## Program your peripheral
If the OpenMote is connected to the PC with a OpenBase then just run the flash-bsl script:
``` bash
python flash-bsl.py OpenMoteSniffer.hex
```

If you get the message "ERROR: Can't connect to target. Ensure boot loader is started." then you will have to enter the Bootloader Backdoor first. Try pressing the RESET button while the ON/SLEEP pin on the OpenBase is connected to GND.

## Running the sniffer
Now that the peripheral is running the required software, it is time to run a host program that communicates with it over the USB cable (which is acting as a serial port).

When not provided any parameters, the script will start interactively and will ask you to input the necessary information (e.g. the channel to capture on).
``` bash
python sniffer.py
```

By default the script will start wireshark to provide real-time output. If you instead want to capture to a file then you should at least provide the ouput parameter.
``` bash
python sniffer.py -o output.pcap
```

For more information about which other parameters you can pass to the script, just run it with -h or --help.
``` bash
python sniffer.py -h
```

To pause the sniffer to change the channel or quit, just press the return key.

## Potential issues

### Linux: Permission denied: '/dev/ttyUSBX'
In order to use the sniffer you need to be able to write data to the serial USB port.

One way to do this is to run the script with root access (using 'sudo') but a more permanent and better solution is to give yourself permission to access the port. Add your user to the group that grants you this permission ('dialout' on Ubuntu, 'uucp' on Arch Linux) and log out afterwards to allow the changes to take effect.
``` bash
sudo gpasswd -a $USER dialout
```

Note that this error can also appear if you try to run the script directly after the OpenMote was plugged in. Wait a second and try again.

### Windows: latency issue
When running the sniffer on Windows, a small change is required in the settings of the COM port to achieve maximum performance. Without the change the sniffer may not be able to capture all packets under high network load.
1. Control Panel -> Device Manager -> Ports (COM & LPT) -> {Select your USB Serial Port}
2. Right click and select "Properties"
3. Select the "Port Settings" tab
4. Click the "Advanced" button at the bottom
5. Change the Latency Timer (msec) to 1 (or the lowest possible setting)
6. Change the USB Transfer Sizes to 64 (or the lowest possible settings)
7. Save and close

## Leds and buttons on OpenMote
TODO: Short description about what the leds mean and what happens when the RESET or USER button is pressed
