# raspicomm-module
raspicomm kernel module with tty driver support for rs485 (raspicommrs485.ko)

## Non-Raspbian kernels
Because I'm going through the motions of cross compiling the rs485 module for other Linux kernels, I thought I might as well host them on GitHub and (possibly) save time for other people doing the same thing.

Currently I've only compiled for the *3.18.0-20-rpi2* kernel, intended for the (non-snappy) Ubuntu Raspberry Pi port.

## Installation Instructions

 * Copy the desired `raspicommrs485.ko module` from `module/linux/<kernel_name_here>/` in this repository to `/lib/modules/<kernel_name_here>` on your Raspberry Pi
 * Edit `/etc/modules` and append `raspicommrs485` to the end of the file
 * Run: `depmod -a`
 * Run: `modprobe raspicommrs485`
 * Restart your Pi! (`shutdown -r now`)
 
Tip:
 * You'll likely need superuser priviliges to run some of the above commands. `sudo` is your friend.
 * Never blindly follow command line instructions from the internet. Always double check ;)
