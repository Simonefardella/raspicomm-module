# raspicomm-module
[raspicomm](https://github.com/Martin-Furter/raspicomm-module) kernel module with tty driver support for rs485 (raspicommrs485.ko)

This is a heavily modifed version of the original raspicomm module modified for Linux kernel versions around 4.9. It controls the BCM2835 SPI directly to communicate with the MAX3140 UART ont the RaspiComm PCB.

## Installation Instructions

 * Make and install the module: `make install`
 * Edit /boot/config.txt and add the lines `dtparam=spi=on` and `dtoverlay=spi0-hw-cs` to the end.
 * Edit `/etc/modules` and append `raspicommrs485` to the end of the file
 * Remove the original SPI modules: `mkdir /root/spi-modules-orig && mv /lib/modules/4.9.59-v7+/kernel/drivers/spi/* /root/spi-modules-orig` 
 * Run: `depmod -a`
 * Run: `modprobe raspicommrs485`
 * Restart your Pi! (`shutdown -r now`)
 
Tip:
 * You'll likely need superuser priviliges to run some of the above commands. `sudo` is your friend.
 * Never blindly follow command line instructions from the internet. Always double check ;)

## Binaries

 * [raspicommrs485.ko V1.0.1 for kernel 4.14.34-v7+ (md5sum 58d2016b744f12249f009dcf14d8ce64)](https://github.com/Martin-Furter/raspicomm-module/raw/master/binaries/4.14.34-v7%2B/raspicommrs485.ko)

## Known Bugs

From time to time there is an SPI error. The SPI transfer done interrupt is fired, but the FIFO is empty and no data can be read. Since this seems to only happen for "WrCfg" commands my solution is to restart the transfer if it failed. I don't know how to properly fix this since i do not know the real cause.

