// vim: noet:ts=4:sw=4:foldmethod=marker
/*

RS485 Driver for the RaspiComm Module.

Latest incarnation hijacking the SPI chip to get fast response times.
The Linux SPI driver responds too slow to handle the MAX3140 at 115200,
especially under load the responset time is way too long. It would be nice
if in the SPI code could be specified that the callbacks should come
directly from SPI interrupt.


RaspiComm to Raspberry Pi Connection:
https://amesberger.files.wordpress.com/2012/08/rasppicomm-to-rasberrypi-connector.jpg
https://pinout.xyz/pinout/spi#

MAX3140 INT    GPIO17   pin 11
MAX3140 SCK    GPIO11   pin 23  SPI0 SCK
MAX3140 MOSI   GPIO10   pin 19  SPI0 MOSI
MAX3140 MISO   GPIO9    pin 21  SPI0 MISO
MAX3140 CE     GPIO8    pin 24  SPI0 CE0


Overlays needed:
  spi0-hw-cs

*/
//============================================================================
// {{{ includes


// Needed by all modules
#include <linux/module.h>
// Needed for KERN_INFO
#include <linux/kernel.h>
// Needed for the macros
#include <linux/init.h>
// Needed for the file structure & register_chrdev()
#include <linux/fs.h>
//
#include <linux/tty.h>
// Needed for struct tty_driver
#include <linux/tty_driver.h>
// needed for gpio_X() calls
#include <linux/gpio.h>
// Needed for request_interrupt()
#include <linux/interrupt.h>
#include <linux/workqueue.h>
// Needed for udelay
#include <linux/delay.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
// needed for KERNEL_VERSION() macro
#include <linux/version.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/hrtimer.h>
// for hrtimer
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include "module.h"
// needed for queue_xxx functions
#include "queue.h"

// }}} includes
//============================================================================
// {{{ driver defines

#define DRV_NAME "raspi-comm"

// }}} driver defines
//============================================================================
// {{{ MAX3140 definitions

// MajorDriverNumber == 0 is using dynamically number
static const int RaspicommMajorDriverNumber = 0;


typedef enum {
	STOPBITS_ONE = 0,
	STOPBITS_TWO = 1
} Stopbits;

typedef enum {
	DATABITS_7 = 1,
	DATABITS_8 = 0
} Databits;

typedef enum {
	PARITY_OFF = 0,
	PARITY_ODD = 1,
	PARITY_EVEN = 2
} Parity;

typedef enum {
	// communication is blocked due to initialization or cleanup
	MAX3140_BLOCK_COMMUNICATION		= 1 << 16,

	// command to read data
	MAX3140_CMD_READ_DATA			= 0 << 14,
	// command to read configuration
	MAX3140_CMD_READ_CONFIG			= 1 << 14,
	// command to write data
	MAX3140_CMD_WRITE_DATA			= 2 << 14,
	// command to write configuration
	MAX3140_CMD_WRITE_CONFIG		= 3 << 14,

	// signals that a byte has been received
	// both data commands transmit the received byte
	MAX3140_RECEIVE_BUFFER_FULL		= 1 << 15,
	// signals that a byte can be added to the transmit buffer
	MAX3140_TRANSMIT_BUF_EMPTY		= 1 << 14,

	// mask including all the configuration flags
	MAX3140_CONFIG_MASK				= 0x3FFF,

	// disable the receive fifo
	MAX3140_CFG_DISABLE_FIFO		= 1 << 13,
	// shutdown the uart
	MAX3140_CFG_SHUTDOWN_UART		= 1 << 12,
	// enable the transmit buffer empty interrupt
	MAX3140_CFG_ENABLE_TX_INT		= 1 << 11,
	// enable the receive data interrupt
	MAX3140_CFG_ENABLE_RX_INT		= 1 << 10,
	// enable the parity bit receive interrupt
	MAX3140_CFG_ENABLE_PBRX_INT		= 1 << 9,
	// enable the receiver activity and frame error interrupt
	MAX3140_CFG_ENABLE_RA_FE_INT	= 1 << 8,
	// swith the IRDAmode on
	MAX3140_CFG_IRDA_MODE_ON		= 1 << 7,
	// transmit two stop bits instead of one
	MAX3140_CFG_TWO_STOP_BITS		= 1 << 6,
	// enable the parity
	MAX3140_CFG_ENABLE_PARITY		= 1 << 5,
	// use 7 bit words instead of 8 bit
	MAX3140_CFG_7_BIT_WORDS			= 1 << 4,
	// mask for the baudrate bits
	MAX3140_CFG_BAUDRATE_MASK		= 0x0F,

	// the defined baudrates
	MAX3140_BAUDRATE_600			= 0xF,
	MAX3140_BAUDRATE_1200			= 0xE,
	MAX3140_BAUDRATE_2400			= 0xD,
	MAX3140_BAUDRATE_4800			= 0xC,
	MAX3140_BAUDRATE_9600			= 0xB,
	MAX3140_BAUDRATE_19200			= 0xA,
	MAX3140_BAUDRATE_38400			= 0x9,
	MAX3140_BAUDRATE_57600			= 0x2,
	MAX3140_BAUDRATE_115200			= 0x1,
	MAX3140_BAUDRATE_230400			= 0x0,

	// index (bit nr) of the parity bit in the config commands
	MAX3140_PARITY_BIT_INDEX		= 8,

	// the write data command doe snot send a byte if this flag is set
	MAX3140_WRDAT_DO_NOT_TRANSMIT   = 1 << 10,
	// if set the transmitter is disabled and receiving data is possible
	MAX3140_WRDAT_TRANSMITTER_OFF   = 1 << 9,

	MAX3140_CMD_RECEIVE_MODE		= MAX3140_CMD_WRITE_DATA |
										MAX3140_WRDAT_DO_NOT_TRANSMIT |
										MAX3140_WRDAT_TRANSMITTER_OFF,

} MAX3140_Flags;


typedef struct {
	struct spi_message msg;
	struct spi_transfer xfer;
	char tx_buf[2];
	char rx_buf[2];
	const char* name;
} rpc_spi_msg_t;

typedef struct {
	struct spi_transfer xfer;
	char tx_buf[2];
	char rx_buf[2];
} rpc_spi_xfer_buf_t;

typedef struct {
	struct spi_message msg;
	rpc_spi_xfer_buf_t x1;
	rpc_spi_xfer_buf_t x2;
	const char* name;
} rpc_spi_msg2_t;

// }}} MAX3140 definitions
//============================================================================
// {{{ BCM2835 SPI definitions

/* SPI register offsets */
// SPI Master Control and Status
#define BCM2835_SPI_CS			0x00
// SPI Master TX and RX FIFOs
#define BCM2835_SPI_FIFO		0x04
// SPI Master Clock Divider
#define BCM2835_SPI_CLK			0x08
// SPI Master Data Length
#define BCM2835_SPI_DLEN		0x0c
// SPI LOSSI mode TOH
#define BCM2835_SPI_LTOH		0x10
// SPI DMA DREQ Controls
#define BCM2835_SPI_DC			0x14

/* Bitfields in CS */
// Enable Long data word in Lossi mode if DMA_LEN is set
#define BCM2835_SPI_CS_LEN_LONG		0x02000000
// Enable DMA mode in Lossi mode
#define BCM2835_SPI_CS_DMA_LEN		0x01000000
// Chip Select 2 Polarity
#define BCM2835_SPI_CS_CSPOL2		0x00800000
// Chip Select 1 Polarity
#define BCM2835_SPI_CS_CSPOL1		0x00400000
// Chip Select 0 Polarity
#define BCM2835_SPI_CS_CSPOL0		0x00200000
// RXF - RX FIFO Full
#define BCM2835_SPI_CS_RXF			0x00100000
// RXR RX FIFO needs Reading (full)
#define BCM2835_SPI_CS_RXR			0x00080000
// TXD TX FIFO can accept Data
#define BCM2835_SPI_CS_TXD			0x00040000
// RXD RX FIFO contains Data
#define BCM2835_SPI_CS_RXD			0x00020000
// Done transfer Done
#define BCM2835_SPI_CS_DONE			0x00010000
// LEN LoSSI enable
#define BCM2835_SPI_CS_LEN			0x00002000
// REN Read Enable (in bidirectional mode)
#define BCM2835_SPI_CS_REN			0x00001000
// ADCS Automatically Deassert Chip Select
#define BCM2835_SPI_CS_ADCS			0x00000800
// INTR Interrupt on RXR
#define BCM2835_SPI_CS_INTR			0x00000400
// INTD Interrupt on Done
#define BCM2835_SPI_CS_INTD			0x00000200
// DMAEN DMA Enable
#define BCM2835_SPI_CS_DMAEN		0x00000100
// Transfer Active
#define BCM2835_SPI_CS_TA			0x00000080
// Chip Select Polarity
#define BCM2835_SPI_CS_CSPOL		0x00000040
// Clear RX FIFO
#define BCM2835_SPI_CS_CLEAR_RX		0x00000020
// Clear TX FIFO
#define BCM2835_SPI_CS_CLEAR_TX		0x00000010
// Clock Polarity
#define BCM2835_SPI_CS_CPOL			0x00000008
// Clock Phase
#define BCM2835_SPI_CS_CPHA			0x00000004
// Chip Select
#define BCM2835_SPI_CS_CS_0			0
#define BCM2835_SPI_CS_CS_1			0x00000001
#define BCM2835_SPI_CS_CS_2			0x00000002

// #define BCM2835_SPI_POLLING_LIMIT_US	30
// #define BCM2835_SPI_POLLING_JIFFIES	2
// #define BCM2835_SPI_DMA_MIN_LENGTH	96
// #define BCM2835_SPI_MODE_BITS	(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH \ x
//				 | SPI_NO_CS | SPI_3WIRE)

#define SPI_CS_CONFIG	BCM2835_SPI_CS_CS_0
#define SPI_CS_RESET	(BCM2835_SPI_CS_DONE | BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX)
#define SPI_CS_START	(SPI_CS_CONFIG | BCM2835_SPI_CS_TA | BCM2835_SPI_CS_INTD)
// #define SPI_CS_DONE		(BCM2835_SPI_CS_DONE)
// #define SPI_CS_DONE		(BCM2835_SPI_CS_DONE | BCM2835_SPI_CS_CS_1)
#define SPI_CS_DONE		(BCM2835_SPI_CS_DONE | BCM2835_SPI_CS_CSPOL)

#define SPI_MAX_TRANSFER_COUNT	8

typedef void (*rpc_spi_callback_t)( uint16_t sent, uint16_t rcvd );

typedef struct {
	uint16_t send_data;
	uint16_t recv_data;
	rpc_spi_callback_t callback;
} rpc_spi_transfer_t;

#define MAX_TRANSFER_COUNT	8

static void rpc_spi_cancel_transfers_and_wait(void);
static bool rpc_spi_transfer_word( uint16_t send_data, rpc_spi_callback_t callback );

// }}} BCM2835 SPI definitions
//============================================================================
// {{{ RaspiComm driver definitions

typedef struct {
	int foo;

	// ------------------------------------------
	// SPI variables
	void __iomem *regs;
	struct clk *clk;
	int spi_irq;
	rpc_spi_transfer_t transfers[MAX_TRANSFER_COUNT];
	bool transfer_in_progress;
	int transfer_count;
	spinlock_t spi_lock;
	// ------------------------------------------
	// MAX3140 variables
	// transmit queue
	queue_t TxQueue;

	// ------------------------------------------
	// TTY related variables
	// the driver instance
	struct tty_driver* tty_drv;
	struct tty_port tty_port;
	// currently opened tty device
	struct tty_struct* tty_open;

	// the number of open() calls
	int tty_opened;

	// ParityIsOdd == true ? odd : even
	int ParityIsOdd;
	int ParityEnabled;


	spinlock_t dev_lock;

	// variable used in the delay to simulate the baudrate
	ktime_t OneCharDelay;
	struct hrtimer last_byte_sent_timer;
	int last_byte_sent_timer_initialized;

	// config setting of the UART
	int UartConfig;

	// struct spi_device *spi_slave;

	unsigned int irqGPIO;
	unsigned int irqNumber;
} RaspiCommData_t;

static RaspiCommData_t rcd = { 0, };

// }}} RaspiComm driver definitions
//============================================================================
// {{{ raspicomm private functions

static unsigned char rpc_max3140_get_baudrate_index( speed_t speed );
static void rpc_max3140_configure( speed_t speed,
				Databits databits, Stopbits stopbits, Parity parity );
static void raspicomm_rs485_received( struct tty_struct* tty, int c );
static irqreturn_t raspicomm_irq_handler( int irq, void* dev_id );
static int rpc_max3140_make_write_data_cmd( int n );

// }}} raspicomm private functions
//============================================================================
// {{{ raspicommDriver functions

static int rpc_tty_open( struct tty_struct *, struct file * );
static void rpc_tty_close( struct tty_struct *, struct file * );
static int rpc_tty_write( struct tty_struct *,
				const unsigned char *, int );
static int rpc_tty_write_room( struct tty_struct * );
static void rpc_tty_flush_buffer( struct tty_struct * );
static int rpc_tty_chars_in_buffer( struct tty_struct * );
static void rpc_tty_set_termios( struct tty_struct *,
				struct ktermios * );
static void rpc_tty_stop( struct tty_struct * );
static void rpc_tty_start( struct tty_struct * );
static void rpc_tty_hangup( struct tty_struct * );
static int rpc_tty_tiocmget( struct tty_struct *tty );
static int rpc_tty_tiocmset( struct tty_struct *tty,
				unsigned int set, unsigned int clear );
static int rpc_tty_ioctl( struct tty_struct* tty,
				unsigned int cmd, unsigned int long arg );
static void rpc_tty_throttle( struct tty_struct * tty );
static void rpc_tty_unthrottle( struct tty_struct * tty );

// }}} raspicommDriver functions
//============================================================================
// {{{ private fields

static const struct tty_operations raspicomm_ops = {
	.open				= rpc_tty_open,
	.close				= rpc_tty_close,
	.write				= rpc_tty_write,
	.write_room			= rpc_tty_write_room,
	.flush_buffer		= rpc_tty_flush_buffer,
	.chars_in_buffer	= rpc_tty_chars_in_buffer,
	.ioctl				= rpc_tty_ioctl,
	.set_termios		= rpc_tty_set_termios,
	.stop				= rpc_tty_stop,
	.start				= rpc_tty_start,
	.hangup				= rpc_tty_hangup,
	.tiocmget			= rpc_tty_tiocmget,
	.tiocmset			= rpc_tty_tiocmset,
	.throttle			= rpc_tty_throttle,
	.unthrottle			= rpc_tty_unthrottle
};

#define IRQ_DEV_NAME "raspicomm"
#define PORT_COUNT 1

// }}} private fields
//============================================================================
// {{{ spi functions

static void log_max3140_message( int tx, int rx, int err )
{
	char buffer[256];

	switch( tx>>14 )
	{
		case 0:
			sprintf( buffer,
				"RdDat %04X -- %04X "
				"R=%d T=%d RA/FE=%d CTS=%d P=%d D=%02X",
				tx,
				rx,
				(rx>>15)&1, (rx>>14)&1,
				(rx>>10)&1, (rx>>9)&1, (rx>>8)&1, rx&255 );
			break;
		case 1:
			sprintf( buffer,
				"RdCfg %04X TEST=%d -- %04X "
				"R=%d T=%d FEN=%d SHDN=%d TM=%d RM=%d PM=%d RAM=%d "
				"IR=%d ST=%d PE=%d L=%d B3=%X",
				tx,
				tx&1,
				rx,
				(rx>>15)&1, (rx>>14)&1, (rx>>13)&1, (rx>>12)&1,
				(rx>>11)&1, (rx>>10)&1, (rx>>9)&1, (rx>>8)&1,
				(rx>>7)&1, (rx>>6)&1, (rx>>5)&1, (rx>>4)&1,
				rx&15 );
			break;
		case 2:
			sprintf( buffer,
				"WrDat %04X TE=%d RTS=%d P=%d D=%02X "
				"-- %04X R=%d T=%d RA/FE=%d CTS=%d P=%d D=%02X",
				tx,
				(tx>>10)&1, (tx>>9)&1, (tx>>8)&1, tx&255,
				rx,
				(rx>>15)&1, (rx>>14)&1,
				(rx>>10)&1, (rx>>9)&1, (rx>>8)&1, rx&255 );
			break;
		case 3:
			sprintf( buffer,
				"WrCfg %04X FEN=%d SHDN=%d TM=%d RM=%d PM=%d RAM=%d "
				"IR=%d ST=%d PE=%d L=%d BR=%X -- %04X R=%d T=%d",
				tx,
				(tx>>13)&1, (tx>>12)&1, (tx>>11)&1, (tx>>10)&1,
				(tx>>9)&1, (tx>>8)&1, (tx>>7)&1, (tx>>6)&1,
				(tx>>5)&1, (tx>>4)&1, tx&15,
				rx,
				(rx>>15)&1, (rx>>14)&1 );
			break;
	}
	if( err )
	{
		LOG_ERR( "%s", buffer );
	}
	else
	{
		LOG( "%s", buffer );
	}
}

static void start_transmitting_done( uint16_t send_data, uint16_t recv_data )
{
	LOG( "start_transmitting_done" );
	// rpc_spi_msg2_rx( context, &rxdata, NULL );
	if( recv_data & MAX3140_RECEIVE_BUFFER_FULL )
	{
		// data is available in the receive register
		// handle the received data
		raspicomm_rs485_received( rcd.tty_open, recv_data );
		LOG( "start_transmitting_done recv: 0x%X", recv_data );
	}
}

#if 0
static void start_transmitting_done2( uint16_t send_data, uint16_t recv_data )
{
	LOG( "start_transmitting_done2" );
}
#else
#define start_transmitting_done2 0
#endif

#if 0
static void irq_msg_write_done( uint16_t send_data, uint16_t recv_data )
{
	LOG( "irq_msg_write_done" );
	// nothing to do here
}
#else
#define irq_msg_write_done 0
#endif

static void irq_msg_read_done( uint16_t send_data, uint16_t recv_data )
{
	unsigned long spinlock_flags;
	// int rxdata, txdata, rc, irqstate = 1;
	uint8_t byte;
	int rc;
	int irqstate = 1;

	LOG( "irq_msg_read_done" );
	// recv_data = rpc_spi_msg_rx( context );
	if( recv_data & MAX3140_RECEIVE_BUFFER_FULL )
	{
		// data is available in the receive register
		// handle the received data
		raspicomm_rs485_received( rcd.tty_open, recv_data );
		irqstate = gpio_get_value( rcd.irqGPIO );
		LOG( "irq_msg_read_done recv: 0x%X irq=%d", recv_data, irqstate );
	}
	if( recv_data & MAX3140_TRANSMIT_BUF_EMPTY )
	{
		// there is space in the transmit buffer
		spin_lock_irqsave( &rcd.dev_lock, spinlock_flags );
		if( rcd.UartConfig & MAX3140_CFG_ENABLE_TX_INT )
		{
			// transmit interrupt is on, this means we have to check the queue
			rc = queue_dequeue( &rcd.TxQueue, &byte );
			if( rc )
			{
				send_data = rpc_max3140_make_write_data_cmd( byte );
				irqstate = 1;
			}
			else
			{
				// no more data to send, disable transmit interrupt
				rcd.UartConfig &= ~MAX3140_CFG_ENABLE_TX_INT;
				send_data = rcd.UartConfig;
			}
			rpc_spi_transfer_word( send_data, irq_msg_write_done );
		}
		else
		{
			// transmit interrupt is off, nothing to do
			// prevent timer from starting
			rc = 1;
		}
		spin_unlock_irqrestore( &rcd.dev_lock, spinlock_flags );
		if( !rc )
		{
			// after the last byte has been sent the transmission is finished
			LOG( "start HR timer last_byte_sent_timer" );
			hrtimer_start( &rcd.last_byte_sent_timer, rcd.OneCharDelay,
						HRTIMER_MODE_REL );
		}
	}
	if( !irqstate )
	{
		// irq pin is still low, read again
		rpc_spi_transfer_word( MAX3140_CMD_READ_DATA, irq_msg_read_done );
	}
}

#if 0
static void stop_transmitting_done( uint16_t send_data, uint16_t recv_data )
{
	LOG( "stop_transmitting_done" );
	// nothing to do here
}
#else
#define stop_transmitting_done 0
#endif

static enum hrtimer_restart last_byte_sent( struct hrtimer *timer )
{
	LOG( "last_byte_sent" );
	rpc_spi_transfer_word( MAX3140_CMD_RECEIVE_MODE, stop_transmitting_done );
	return HRTIMER_NORESTART;
}

#if 0
static void configure_uart_done( uint16_t send_data, uint16_t recv_data )
{
	LOG( "configure_uart_done" );
	// nothing to do here
}
#else
#define configure_uart_done 0
#endif


// }}} spi functions
//============================================================================
// {{{ raspicomm private function

static void rpc_tty_cleanup( struct platform_device* pdev )
{
	unsigned long spinlock_flags;
	LOG_DBG( "cleanup all" );

	spin_lock_irqsave( &rcd.dev_lock, spinlock_flags );
	// set the shutdown flag
	rcd.UartConfig |= MAX3140_BLOCK_COMMUNICATION;
	// clear the queue
	rcd.TxQueue.read = rcd.TxQueue.write;
	spin_unlock_irqrestore( &rcd.dev_lock, spinlock_flags );

	// remove the interrupt
	if( rcd.irqNumber >= 0 )
	{
		LOG_DBG( "free_irq" );
		free_irq( rcd.irqNumber, NULL );
	}
	// cancel the timer
	if( rcd.last_byte_sent_timer_initialized )
	{
		LOG_DBG( "hrtimer_cancel" );
		hrtimer_cancel( &rcd.last_byte_sent_timer );
	}

	// wait for all SPI transfers to finish
	rpc_spi_cancel_transfers_and_wait();

	// do all the remaining cleanup...
	if( rcd.tty_drv )
	{
		LOG_DBG( "tty_unregister_driver" );
		tty_unregister_driver( rcd.tty_drv );
		LOG_DBG( "put_tty_driver" );
		put_tty_driver( rcd.tty_drv );
	}
	if( rcd.irqGPIO >= 0 )
	{
		LOG_DBG( "gpio_free" );
		gpio_free( rcd.irqGPIO );
	}
	LOG_DBG( "cleanup done" );
}

// initialization function that gets called when the module is loaded
static int rpc_tty_init( struct platform_device* pdev )
{
	unsigned long spinlock_flags;
	int pin = 17;
	int result;

	// log the start of the initialization
	LOG_INFO( "raspicommrs485 init: version " RASPICOMM_VERSION );

	LOG_DBG( "initializing globals" );
	// raspicommDriver = NULL;
	// memset( &Port, 0, sizeof(Port) );
	// OpenCount = 0;
	// ParityIsOdd = 0;
	// ParityEnabled = 0;
	// OpenTTY = NULL;
	// memset( &TxQueue, 0, sizeof(TxQueue) );
	rcd.UartConfig = MAX3140_BLOCK_COMMUNICATION;
	// spi_slave = NULL;
	rcd.irqGPIO = -EINVAL;
	rcd.irqNumber = -EINVAL;
	// last_byte_sent_timer_initialized = 0;

#if 0
	/*
	static int of_spi_register_master(struct spi_master *master)
	int nb, i, *cs;
	struct device_node *np = master->dev.of_node;
	*/

	if( of_gpio_named_count( np, "cs-gpios" ) < 1 )
	{
		LOG_ERR( "of_gpio_named_count(cs-gpios) failed" );
		return -ENOENT;
	}
	cs[i] = of_get_named_gpio( np, "cs-gpios", 1 );
#endif
	// Request a GPIO pin from the driver
	LOG_DBG( "gpio_request" );
	result = gpio_request( pin, "rpc0irq" );
	if( result < 0 )
	{
		LOG_ERR( "gpio_request failed with code %d", result );
		goto cleanup;
	}
	rcd.irqGPIO = pin;
	// 'irqGPIO' is expected to be an unsigned int, i.e. the GPIO number
	// Set GPIO as input
	LOG_DBG( "gpio_direction_input" );
	result = gpio_direction_input( rcd.irqGPIO );
	if( result < 0 )
	{
		LOG_ERR( "gpio_direction_input failed with code %d", result );
		goto cleanup;
	}

	// map your GPIO to an IRQ
	LOG_DBG( "gpio_to_irq( %d )", rcd.irqGPIO );
	rcd.irqNumber = gpio_to_irq( rcd.irqGPIO );
	if( rcd.irqNumber < 0 )
	{
		LOG_ERR( "gpio_to_irq failed with code %d", result );
		goto cleanup;
	}
	// requested interrupt
	LOG_DBG( "request_irq = %d", rcd.irqNumber );
	result = request_irq( rcd.irqNumber, raspicomm_irq_handler,
						// interrupt mode flag
						IRQF_TRIGGER_FALLING,
						// used in /proc/interrupts
						IRQ_DEV_NAME,
						// the *dev_id shared interrupt lines, NULL is okay
						NULL );
	if( result < 0 )
	{
		LOG_ERR( "request_irq failed with code %d", result );
		goto cleanup;
	}

	// initialize the port
	LOG_DBG( "tty_port_init" );
	tty_port_init( &rcd.tty_port );
	rcd.tty_port.low_latency = 1;

	// allocate the driver
	LOG_DBG( "tty_alloc_driver" );
	rcd.tty_drv = tty_alloc_driver( PORT_COUNT, TTY_DRIVER_REAL_RAW );

	// return if allocation fails
	if( IS_ERR( rcd.tty_drv ) )
	{
		LOG_ERR( "tty_alloc_driver failed" );
		goto cleanup;
	}

	// init the driver
	rcd.tty_drv->owner					= THIS_MODULE;
	rcd.tty_drv->driver_name			= "raspicomm rs485";
	rcd.tty_drv->name					= "ttyRPC";
	rcd.tty_drv->major					= RaspicommMajorDriverNumber;
	rcd.tty_drv->minor_start			= 0;
	//rcd.tty_drv->flags				= TTY_DRIVER_REAL_RAW;
	rcd.tty_drv->type					= TTY_DRIVER_TYPE_SERIAL;
	rcd.tty_drv->subtype				= SERIAL_TYPE_NORMAL;
	rcd.tty_drv->init_termios			= tty_std_termios;
	rcd.tty_drv->init_termios.c_ispeed  = 9600;
	rcd.tty_drv->init_termios.c_ospeed  = 9600;
	rcd.tty_drv->init_termios.c_iflag   = 0;
	rcd.tty_drv->init_termios.c_oflag   = 0;
	rcd.tty_drv->init_termios.c_cflag   = B9600 | CREAD | CS8 | CLOCAL;
	rcd.tty_drv->init_termios.c_lflag   = 0;

	// ++++ is that OK?
	// rcd.tty_drv->driver_state = rcd;
	// +++->tty_struct->driver_data = rcd;
	// +++->tty_struct->disc_data = rcd;
	//rcd.tty_open->driver_data = rcd;

	// initialize function callbacks of tty_driver,
	// necessary before tty_register_driver()
	LOG_DBG( "tty_set_operations" );
	tty_set_operations( rcd.tty_drv, &raspicomm_ops );

	// link the port with the driver
	LOG_DBG( "tty_port_link_device" );
	tty_port_link_device( &rcd.tty_port, rcd.tty_drv, 0 );

	// try to register the tty driver
	LOG_DBG( "tty_register_driver" );
	if( tty_register_driver( rcd.tty_drv ) )
	{
		LOG_ERR( "tty_register_driver failed" );
		goto cleanup;
	}

	LOG_DBG( "initializing hrtimer" );
	hrtimer_init( &rcd.last_byte_sent_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	rcd.last_byte_sent_timer.function = &last_byte_sent;
	rcd.last_byte_sent_timer_initialized = 1;

	// now configure the UART
	rpc_spi_transfer_word( MAX3140_CMD_RECEIVE_MODE, stop_transmitting_done );
	rpc_max3140_configure( 9600, DATABITS_8, STOPBITS_ONE, PARITY_OFF );

	// successfully initialized the module
	spin_lock_irqsave( &rcd.dev_lock, spinlock_flags );
	rcd.UartConfig &= ~MAX3140_BLOCK_COMMUNICATION;
	spin_unlock_irqrestore( &rcd.dev_lock, spinlock_flags );
	LOG_INFO( "raspicomm_init() completed" );
	return 0;

cleanup:
	rpc_tty_cleanup( pdev );
	LOG_ERR( "raspicomm_init() failed" );
	return -ENODEV;
}

// cleanup function that gets called when the module is unloaded
static void rpc_tty_exit( struct platform_device* pdev )
{
	LOG_INFO( "raspicomm_exit() called" );
	rpc_tty_cleanup( pdev );
	LOG_INFO( "kernel module exit" );
}


// helper function
static unsigned char rpc_max3140_get_baudrate_index( speed_t speed )
{
	switch( speed )
	{
		case 600:	return MAX3140_BAUDRATE_600;
		case 1200:   return MAX3140_BAUDRATE_1200;
		case 2400:   return MAX3140_BAUDRATE_2400;
		case 4800:   return MAX3140_BAUDRATE_4800;
		default:
		case 9600:   return MAX3140_BAUDRATE_9600;
		case 19200:  return MAX3140_BAUDRATE_19200;
		case 38400:  return MAX3140_BAUDRATE_38400;
		case 57600:  return MAX3140_BAUDRATE_57600;
		case 115200: return MAX3140_BAUDRATE_115200;
		case 230400: return MAX3140_BAUDRATE_230400;
	}
}

static void rpc_max3140_configure( speed_t speed,
				Databits databits, Stopbits stopbits, Parity parity )
{
	unsigned long spinlock_flags;
	ktime_t delay;
	int config = MAX3140_CMD_WRITE_CONFIG | MAX3140_CFG_ENABLE_RX_INT;
	// default is 8 data bits plus startbit plus stop bit
	int bit_count = 8+2;

	config |= rpc_max3140_get_baudrate_index( speed );
	if( databits == DATABITS_7 )
	{
		config |= MAX3140_CFG_7_BIT_WORDS;
		databits--;
	}
	if( stopbits == STOPBITS_TWO )
	{
		config |= MAX3140_CFG_TWO_STOP_BITS;
		databits++;
	}
	if( parity != PARITY_OFF )
	{
		config |= MAX3140_CFG_ENABLE_PARITY;
		databits++;
	}
	delay = ktime_set( 0, (NSEC_PER_SEC / speed) * bit_count );

	LOG( "rpc_max3140_configure() called "
		"speed=%i, databits=%i, stopbits=%i, parity=%i "
		"=> config: %X, delay: %d",
		speed, databits, stopbits, parity,
		config, (int)ktime_to_us(delay) );

	spin_lock_irqsave( &rcd.dev_lock, spinlock_flags );
	config |= rcd.UartConfig &
				(MAX3140_BLOCK_COMMUNICATION | MAX3140_CFG_ENABLE_TX_INT);
	if( rcd.UartConfig != config )
	{
		// update the uart only if the config changed
		rcd.UartConfig = config;
		rcd.OneCharDelay = delay;
		rcd.ParityEnabled = parity != PARITY_OFF;
		rcd.ParityIsOdd = parity == PARITY_ODD;
		rpc_spi_transfer_word( rcd.UartConfig, configure_uart_done );
	}
	spin_unlock_irqrestore( &rcd.dev_lock, spinlock_flags );
}

static int rpc_max3140_make_write_data_cmd( int data )
{
	data |= MAX3140_CMD_WRITE_DATA;
	if( rcd.ParityEnabled )
	{
		int n = data;
		// put the xor of the lowest 8 bits into bit 0 of n
		n ^= (n >> 4);
		n ^= (n >> 2);
		n ^= (n >> 1);
		n &= 1;
		// now 0 = even number of one bits, 1 = odd, this is even parity
		// add parity config, for odd parity the bit must be inverted
		n ^= rcd.ParityIsOdd;
		// add the parity bit to the command
		data |= n << MAX3140_PARITY_BIT_INDEX;
	}
	return data;
}

static irqreturn_t raspicomm_irq_handler( int irq, void* dev_id )
{
	LOG( "raspicomm_irq_handler" );
	rpc_spi_transfer_word( MAX3140_CMD_READ_DATA, irq_msg_read_done );
	return IRQ_HANDLED;
}

// this function pushes a received character to the opened tty device,
// called by the interrupt function
static void raspicomm_rs485_received( struct tty_struct* tty, int c )
{
	LOG( "raspicomm_rs485_received(c=%03X)", c & 0x1FF );

	// ++++ (mf) should check parity here!?!

	if( tty != NULL && tty->port != NULL )
	{
		// send the character to the tty
		tty_insert_flip_char( tty->port, c, TTY_NORMAL );

		// tell it to flip the buffer
		tty_flip_buffer_push( tty->port );
	}
}

// }}} raspicomm private function
//============================================================================
// {{{ TTY Interface Functions

// called by the kernel when open() is called for the device
static void rpc_tty_open( struct tty_struct* tty, struct file* file )
{
	LOG_DBG( "rpc_tty_open() called" );

	if( rcd.tty_opened )
	{
		LOG_ERR( "rpc_tty_open() was not successful as rcd.tty_opened != 0 - Trying to close port and reopen" );
		rpc_tty_close(tty, file );
		rpc_tty_open(tty, file );
	}
	else
	{
		LOG_INFO( "rpc_tty_open() was successful" );
		rcd.tty_open = tty;
		rcd.tty_opened = 1;
	}
}

// called by the kernel when close() is called for the device
static void rpc_tty_close( struct tty_struct* tty, struct file* file )
{
	LOG_DBG( "rpc_tty_close called" );
	if( !rcd.tty_opened )
	{
		LOG_ERR( "rpc_tty_close: can't close device since it is already closed" );
	}
	else
	{
		// rcd.tty_open->driver_data = NULL;
		rcd.tty_open = NULL;
		rcd.tty_opened = 0;
		LOG_INFO( "rpc_tty_close: device was closed" );
	}
}

static int rpc_tty_write( struct tty_struct* tty,
	const unsigned char* buf, int count )
{
	unsigned long spinlock_flags;
	int rc;
	int data;

	LOG( "rpc_tty_write(count=%i)", count );
	if( count <= 0 )
	{
		return 0;
	}
	spin_lock_irqsave( &rcd.dev_lock, spinlock_flags );
	if( rcd.UartConfig & MAX3140_BLOCK_COMMUNICATION )
	{
		// this device is gone
		rc = -ENODEV;
	}
	else
	{
		rc = 0;
		if( !(rcd.UartConfig & MAX3140_CFG_ENABLE_TX_INT) )
		{
			// no transfer in progress or it is sending the last byte
			LOG( "starting transfer" );
			// send the first byte
			data = rpc_max3140_make_write_data_cmd( buf[0] );
			rcd.UartConfig |= MAX3140_CFG_ENABLE_TX_INT;
			rpc_spi_transfer_word( rcd.UartConfig, start_transmitting_done2 );
			rpc_spi_transfer_word( data, start_transmitting_done );
			rc++;
			// cancel a pending EOT
			hrtimer_cancel( &rcd.last_byte_sent_timer );
		}
		// add the remaining bytes to the queue, stop if it is full
		while( rc < count )
		{
			if( !queue_enqueue( &rcd.TxQueue, buf[rc] ) )
			{
				// queue full
				break;
			}
			rc++;
		}
	}
	spin_unlock_irqrestore( &rcd.dev_lock, spinlock_flags );
	LOG( "rpc_tty_write: %d", rc );
	return rc;
}

// called by kernel to evaluate how many bytes can be written
static int rpc_tty_write_room( struct tty_struct *tty )
{
	unsigned long spinlock_flags;
	int rc;

	spin_lock_irqsave( &rcd.dev_lock, spinlock_flags );
	if( rcd.UartConfig & MAX3140_BLOCK_COMMUNICATION )
	{
		rc = 0;
	}
	else
	{
		rc = queue_get_room( &rcd.TxQueue );
	}
	spin_unlock_irqrestore( &rcd.dev_lock, spinlock_flags );
	return rc;
}

static void rpc_tty_flush_buffer( struct tty_struct * tty )
{
	LOG( "rpc_tty_flush_buffer called" );
}

static int rpc_tty_chars_in_buffer( struct tty_struct * tty )
{
	unsigned long spinlock_flags;
	int rc;

	spin_lock_irqsave( &rcd.dev_lock, spinlock_flags );
	rc = QUEUE_SIZE - 1 - queue_get_room( &rcd.TxQueue );
	spin_unlock_irqrestore( &rcd.dev_lock, spinlock_flags );
	return rc;
}

// called by the kernel when cfsetattr() is called from userspace
static void rpc_tty_set_termios( struct tty_struct* tty,
				struct ktermios* kt )
{
	unsigned long spinlock_flags;
	int rc;
	int cflag;
	speed_t baudrate;
	Databits databits;
	Parity parity;
	Stopbits stopbits;

	LOG( "rpc_tty_set_termios() called" );
	spin_lock_irqsave( &rcd.dev_lock, spinlock_flags );
	rc = (rcd.UartConfig & MAX3140_BLOCK_COMMUNICATION);
	spin_unlock_irqrestore( &rcd.dev_lock, spinlock_flags );
	if( rc )
	{
		LOG( "rpc_tty_set_termios() call not allowed" );
		return;
	}

	// get the baudrate
	baudrate = tty_get_baud_rate( tty );

	// get the cflag
	cflag = tty->termios.c_cflag;

	// get the databits
	switch( cflag & CSIZE )
	{
		case CS7:
			databits = DATABITS_7;
			break;

		default:
		case CS8:
			databits = DATABITS_8;
			break;
	}

	// get the stopbits
	stopbits = ( cflag & CSTOPB ) ? STOPBITS_TWO : STOPBITS_ONE;

	// get the parity
	// is parity used
	if( cflag & PARENB )
	{
		// is it even or odd? store it for sending
		parity = ( cflag & PARODD ) ? PARITY_ODD : PARITY_EVEN;
	}
	else
	{
		parity = PARITY_OFF;
	}

	// update the configuration
	rpc_max3140_configure( baudrate, databits, stopbits, parity );
}

static void rpc_tty_stop( struct tty_struct * tty )
{
	LOG( "rpc_tty_stop called" );
}

static void rpc_tty_start( struct tty_struct * tty )
{
	LOG( "rpc_tty_start called" );
}

static void rpc_tty_hangup( struct tty_struct * tty )
{
	LOG( "rpc_tty_hangup called" );
}

static int rpc_tty_tiocmget( struct tty_struct *tty )
{
	LOG( "rpc_tty_tiocmget called" );
	return 0;
}

static int rpc_tty_tiocmset( struct tty_struct *tty,
								unsigned int set, unsigned int clear )
{
	LOG( "rpc_tty_tiocmset called" );
	return 0;
}

// called by the kernel to get/set data
static int rpc_tty_ioctl( struct tty_struct* tty,
								unsigned int cmd, unsigned int long arg )
{
	int ret;

	LOG( "rpc_tty_ioctl() called with cmd=%X, arg=%lX", cmd, arg );
	switch( cmd )
	{
		case TIOCMSET:
		case TIOCMGET:
			// ioctl to get and set DTR, DSR, RTS, CTS, etc...
			ret = 0;
			break;

		default:
			ret = -ENOIOCTLCMD;
			break;
	}
	return ret;
}

static void rpc_tty_throttle( struct tty_struct * tty )
{
	LOG( "throttle" );
}

static void rpc_tty_unthrottle( struct tty_struct * tty )
{
	LOG( "unthrottle" );
}

// }}} TTY Interface Functions
//============================================================================
// {{{ SPI BCM2835 functions

static inline uint32_t rpc_spi_read_reg( unsigned reg )
{
	uint32_t val;
	val = readl( rcd.regs + reg );
	// LOG( "rpc_spi_read_reg %d = %08X", reg, val );
	return val;
}

static inline void rpc_spi_write_reg( unsigned reg, uint32_t val )
{
	// LOG( "rpc_spi_write_reg %d = %08X", reg, val );
	writel( val, rcd.regs + reg );
}

static inline bool rpc_spi_read_fifo( uint8_t* data )
{
	uint32_t cs;

	// if( rpc_spi_read_reg( BCM2835_SPI_CS ) & BCM2835_SPI_CS_RXD )
	cs = rpc_spi_read_reg( BCM2835_SPI_CS );
	if( cs & BCM2835_SPI_CS_RXD )
	{
		*data = rpc_spi_read_reg( BCM2835_SPI_FIFO );
		return true;
	}
	return false;
}

static inline bool rpc_spi_write_fifo( uint8_t data )
{
	if( rpc_spi_read_reg( BCM2835_SPI_CS ) & BCM2835_SPI_CS_TXD )
	{
		rpc_spi_write_reg( BCM2835_SPI_FIFO, data );
		return true;
	}
	return false;
}

static void rpc_spi_reset(void)
{
	rpc_spi_write_reg( BCM2835_SPI_CS, SPI_CS_RESET );
	rpc_spi_write_reg( BCM2835_SPI_CS, SPI_CS_DONE );
}

static void rpc_spi_cancel_transfers_and_wait(void)
{
	unsigned long spinlock_flags;
	int n = 100;
	int tcnt = 1;

	while( n > 0 )
	{
		spin_lock_irqsave( &rcd.spi_lock, spinlock_flags );
		if( rcd.transfer_count > 1 )
		{
			rcd.transfer_count = 1;
		}
		tcnt = rcd.transfer_count;
		spin_unlock_irqrestore( &rcd.spi_lock, spinlock_flags );
		if( tcnt > 0 )
		{
			msleep( 1 );
			n--;
		}
		else
		{
			n = 0;
		}
	}
	if( tcnt != 0 )
	{
		LOG_DBG( "rpc_spi_cancel_transfers_and_wait: timeout waiting for eond of transfers" );
		rpc_spi_reset();
	}
}

/* Start a transfer if there is one and none is in progress.
 * To be called only by rpc_spi_interrupt() and rpc_spi_transfer_word().
 */
static bool rpc_spi_start_transfer(void)
{
	unsigned long spinlock_flags;
	bool rc = true;

	spin_lock_irqsave( &rcd.spi_lock, spinlock_flags );
	if( rcd.transfer_count == 0 )
	{
		// no transfers to start
	}
	else if( rpc_spi_read_reg( BCM2835_SPI_CS ) & BCM2835_SPI_CS_INTD )
	{
		// a transfer is already in progress
	}
	else
	{
		uint16_t data = rcd.transfers[0].send_data;
		// set Transfer Active flag
		rpc_spi_write_reg( BCM2835_SPI_CS, SPI_CS_START );
		if( !rpc_spi_write_fifo( data>>8 ) )
		{
			// writing to FIFO failed
			rc = false;
			rpc_spi_reset();
			LOG_ERR( "rpc_spi_start_transfer: writing byte 1 FIFO failed" );
		}
		else if( !rpc_spi_write_fifo( data ) )
		{
			// writing to FIFO failed
			rc = false;
			rpc_spi_reset();
			LOG_ERR( "rpc_spi_start_transfer: writing byte 2 FIFO failed" );
		}
		else
		{
			LOG( "rpc_spi_start_transfer: wrote %04X", data );
			rcd.transfer_in_progress = true;
		}
	}
	spin_unlock_irqrestore( &rcd.spi_lock, spinlock_flags );
	return rc;
}

/* SPI interrupt function called when a transfer is complete.
 */
static irqreturn_t rpc_spi_interrupt( int irq, void *dev_id )
{
	unsigned long spinlock_flags;
	uint8_t h, l;
	rpc_spi_transfer_t t;
	bool more;
	uint8_t read_err;

	spin_lock_irqsave( &rcd.spi_lock, spinlock_flags );
	t = rcd.transfers[0];

	read_err = 0;
	h = l = 0;
	if( !rpc_spi_read_fifo( &h ) )
	{
		read_err |= 0x10;
	}
	if( !rpc_spi_read_fifo( &l ) )
	{
		read_err |= 0x01;
	}
	t.recv_data = h<<8 | l;
	rpc_spi_write_reg( BCM2835_SPI_CS, SPI_CS_DONE );
	rcd.transfer_in_progress = false;
	if( read_err == 0 )
	{
		// SPI transfer finished
		rcd.transfer_count--;
		more = rcd.transfer_count;
		if( more )
		{
			// remove the transfer from the list by moving all others one down
			memmove( rcd.transfers, rcd.transfers+1,
					rcd.transfer_count*sizeof(*rcd.transfers) );
		}
		spin_unlock_irqrestore( &rcd.spi_lock, spinlock_flags );
#ifdef DEBUG
		log_max3140_message( t.send_data, t.recv_data, 0 );
#endif
		if( t.callback )
		{
			t.callback( t.send_data, t.recv_data );
		}
		// LOG( "MAX3140 IRQ %d", gpio_get_value( rcd.irqGPIO ) );
	}
	else
	{
		// SPI transfer failed, try it again
		spin_unlock_irqrestore( &rcd.spi_lock, spinlock_flags );
		more = true;
		LOG_ERR( "rpc_spi_interrupt: error reading FIFO (%02X)", read_err );
		log_max3140_message( t.send_data, -1, 1 );
		rpc_spi_write_reg( BCM2835_SPI_CS, SPI_CS_RESET );
	}
	if( more )
	{
		// udelay( 1 );
		rpc_spi_start_transfer();
	}
	return IRQ_HANDLED;
}

bool rpc_spi_transfer_word( uint16_t send_data, rpc_spi_callback_t callback )
{
	unsigned long spinlock_flags;

	if( rcd.transfer_count >= (SPI_MAX_TRANSFER_COUNT-1) )
	{
		return false;
	}
	spin_lock_irqsave( &rcd.spi_lock, spinlock_flags );
	rcd.transfers[rcd.transfer_count].send_data = send_data;
	rcd.transfers[rcd.transfer_count].recv_data = 0;
	rcd.transfers[rcd.transfer_count].callback = callback;
	rcd.transfer_count++;
	spin_unlock_irqrestore( &rcd.spi_lock, spinlock_flags );
	rpc_spi_start_transfer();
	return true;
}

int rpc_spi_bcm2835_init( struct platform_device* pdev )
{
	struct resource *res;
	int err;
	int clk_value;

	res = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
	rcd.regs = devm_ioremap_resource( &pdev->dev, res );
	if( IS_ERR(rcd.regs) )
	{
		err = PTR_ERR( rcd.regs );
		goto out_master_put;
	}

	rcd.clk = devm_clk_get( &pdev->dev, NULL );
	if( IS_ERR(rcd.clk) )
	{
		err = PTR_ERR( rcd.clk );
		dev_err( &pdev->dev, "could not get clk: %d\n", err );
		goto out_master_put;
	}

	rcd.spi_irq = platform_get_irq( pdev, 0 );
	if( rcd.spi_irq <= 0 )
	{
		dev_err( &pdev->dev, "could not get IRQ: %d\n", rcd.spi_irq );
		err = rcd.spi_irq ? rcd.spi_irq : -ENODEV;
		goto out_master_put;
	}
	LOG_DBG( "spi_irq = %d", rcd.spi_irq );

	spin_lock_init( &rcd.spi_lock );

	clk_prepare_enable( rcd.clk );

	err = devm_request_irq( &pdev->dev, rcd.spi_irq, rpc_spi_interrupt,
					0, dev_name(&pdev->dev), &pdev->dev );
	if( err )
	{
		dev_err( &pdev->dev, "could not request IRQ: %d\n", err );
		goto out_clk_disable;
	}

	// 1MHz SPI clock, should be calculated ++++++
	clk_value = 250;
	// clk_value = 250 * 250;
	rpc_spi_reset();
	rpc_spi_write_reg( BCM2835_SPI_CLK, clk_value );

	return 0;

out_master_put:
out_clk_disable:
	clk_disable_unprepare( rcd.clk );
// out_none:
	return err;
}

void rpc_spi_bcm2835_exit( struct platform_device* pdev )
{
	/* Clear FIFOs, and disable the HW block */
	rpc_spi_write_reg( BCM2835_SPI_CS,
			BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX );

	// free_irq( rcd.spi_irq, NULL );
	clk_disable_unprepare( rcd.clk );
}

// }}} SPI BCM2835 functions
//============================================================================
// {{{ Platform Driver Code

static int raspicomm_probe( struct platform_device *pdev )
{
	int err;

	memset( &rcd, 0, sizeof(rcd) );

	spin_lock_init( &rcd.dev_lock );
	err = rpc_spi_bcm2835_init( pdev );
	if( err )
	{
		dev_err( &pdev->dev, "could not request IRQ: %d\n", err );
		goto out_undo_none;
	}

	// init tty
	err = rpc_tty_init( pdev );
	if( err )
	{
		goto out_undo_spi_init;
	}

	return 0;

out_undo_spi_init:
	rpc_spi_bcm2835_exit( pdev );
out_undo_none:
	return err;
}

static int raspicomm_remove( struct platform_device *pdev )
{
	rpc_tty_exit( pdev );
	rpc_spi_bcm2835_exit( pdev );
	return 0;
}

static const struct of_device_id raspicomm_match[] = {
	{ .compatible = "brcm,bcm2835-spi", },
	{}
};
MODULE_DEVICE_TABLE( of, raspicomm_match );

static struct platform_driver raspicomm_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = raspicomm_match,
	},
	.probe = raspicomm_probe,
	.remove = raspicomm_remove,
};
module_platform_driver( raspicomm_driver );

MODULE_DESCRIPTION( "Raspicomm kernel module for RS485 tty driver."
"\n                This module includes driver code for the Broadcom BCM2835 SPI controller."
"\n                https://github.com/Martin-Furter/raspicomm-module/" );
MODULE_AUTHOR( "Martin Furter (mf), mdk" );
MODULE_LICENSE( "GPL v2" );
MODULE_VERSION( RASPICOMM_VERSION );
MODULE_SUPPORTED_DEVICE( "ttyRPC" );

// }}} Platform Driver Code
//============================================================================

