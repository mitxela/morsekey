### micronucleus config

Micronucleus is a USB bootloader, see https://github.com/micronucleus/micronucleus

It is completely optional here, you don't need to worry about it to get the morse key to work, but installing it will allow you to update the firmware on the device without needing a programming cable.

Download micronucleus and copy the two files into `firmware/configuration/morsekey` then run `make CONFIG=morsekey`

Tested with micronucleus v2.04

Configuration is the same as `t85_default` but with the USB data lines on the correct pins, and the bootloader is only triggered if the key is held down while connecting the USB cable.
