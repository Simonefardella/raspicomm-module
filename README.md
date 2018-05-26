# raspicomm-module
[raspicomm](https://github.com/Martin-Furter/raspicomm-module) kernel module with tty driver support for rs485 (raspicommrs485.ko)

This is a modifed version for kernel versions around 4.9, the SPI communication is completely rewritten to use asynchronous SPI communication only.

**WARNING**: Unfortunately this version does not work reliable. Under load interrupts from the MAX3140 can get lost which leads to a temporary deadlock. Sending at least one byte seems to reset the communication. I think the problem comes from the fact, that the asynchronous SPI callbacks are scheduled to be run from a normal process. I'll try to rewrite this module do again take full control of the main SPI controller and do as much as needed in the interrupt routine.

## Installation Instructions

 * Make and install the module (+++ document this)
 * Edit /boot/config.txt and add the lines `dtparam=spi=on` and `dtoverlay=spi0-hw-cs` to the end.
 * Edit `/etc/modules` and append `raspicommrs485` to the end of the file
 * Run: `depmod -a`
 * Run: `modprobe raspicommrs485`
 * Restart your Pi! (`shutdown -r now`)
 
Tip:
 * You'll likely need superuser priviliges to run some of the above commands. `sudo` is your friend.
 * Never blindly follow command line instructions from the internet. Always double check ;)

