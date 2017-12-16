# raspicomm-module
raspicomm kernel module with tty driver support for rs485 (raspicommrs485.ko)

This is a modifed version for kernel versions around 4.9, the SPI communication is completelyrewritten to use asynchronous communication only.

## Installation Instructions

 * Make and install the module (+++ document this)
 * Copy spi0devdis.dtbo to /boot/overlays/
 * Edit /boot/config.txt and add the line `dtoverlay=spi0devdis` to the end.
 * Edit `/etc/modules` and append `raspicommrs485` to the end of the file
 * Run: `dtoverlay spi0devdis`
 * Run: `depmod -a`
 * Run: `modprobe raspicommrs485`
 * Restart your Pi! (`shutdown -r now`)
 
Tip:
 * You'll likely need superuser priviliges to run some of the above commands. `sudo` is your friend.
 * Never blindly follow command line instructions from the internet. Always double check ;)
