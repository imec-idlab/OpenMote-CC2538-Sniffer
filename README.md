# IEEE 802.15.4 Sniffer for OpenMote-CC2538

This project provides a **lossless** sniffer for the IEEE 802.15.4 network which can be run on the [OpenMote-CC2538](http://www.openmote.com/hardware/openmote-cc2538-en.html) hardware. You also need the [OpenBase](http://www.openmote.com/hardware/openbase.html) to connect the OpenMote to your computer.

The captured packets can either be monitored **real-time** in Wireshark or be written to a pcap file.

The program that runs on the computer side is **cross-platform** and has been tested on Windows, Linux and Mac OS X.

## Installing dependencies
The software that runs on the computer uses python, so if you are a windows user you must first install this if you haven't already. The code has been written to be compatible with both python 2 and 3, so which version you have doesn't really matter.

One dependency that you will have to install is [pySerial](https://pypi.python.org/pypi/pyserial). It can easily be installed through pip or with easy_install or you can just manually download it and run the setup.

Windows users must also install [pywin32](https://sourceforge.net/projects/pywin32/files/pywin32/).

## Preparing the sniffer
To use it for the first time you must of course flash the program to the OpenMote. To do this, connect the OpenBase and run the following command:
``` bash
sudo python openmote-bsl.py OpenMoteSniffer.hex --board openbase
```

If you get the message "ERROR: Can't connect to target. Ensure boot loader is started." then you will have to enter the Bootloader Backdoor first. Try pressing the RESET button while the ON/SLEEP pin on the OpenBase is connected to GND.

## Running the sniffer
Now that the OpenMote should be running the sniffer, it is time to run a program on the computer that communicates with it over the USB cable (which is acting as a serial port).

When not provided any parameters, the program will start interactively and will ask you to input the necessary information (e.g. the channel to capture on).
``` bash
python sniffer.py
```

The optional parameters can be passed to make the sniffer start immediately. The -h or --help options will display which parameters are available.
``` bash
python sniffer.py -c 25 -p /dev/ttyUSB0 -w wireshark-gtk
```

When the sniffer starts, it will immediately start wireshark and the capturing will begin. To change the channel afterwards, just press the return key in the terminal where your python script is running. This will pause the sniffer and let you input a new channel number.

If wireshark is saying that the FCS field in the packets are incorrect then right click on the FCS and select Protocol Prefrences > TI CC24xx FCS format. The checksum should now be correctly parsed in all packets.

## Potential issues

### Linux: Permission denied: '/dev/ttyUSBX'
In order to use the sniffer you need to be able to write data to the serial USB port.

One way to do this is to run the script with root access (using 'sudo') but a more permanent and better solution is to give your user permission to access the port. Add your user to the group that grants you this permission ('dialout' on Ubuntu, 'uucp' on Arch Linux) and log out afterwards to allow the changes to take effect.
``` bash
sudo gpasswd -a $USER dialout
```

Note that this error can also appear if you try to run the script directly after the OpenMote was plugged in. Wait a few seconds and try again.

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
