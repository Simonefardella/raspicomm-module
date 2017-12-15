// vim: et:ts=4:sw=4:smarttab

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

#include "module.h"
// needed for queue_xxx functions
#include "queue.h"         

// MajorDriverNumber == 0 is using dynamically number
static const int RaspicommMajorDriverNumber = 0;

static DEFINE_SPINLOCK( dev_lock );

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
    PARITY_ON = 1
} Parity;

typedef enum {
    MAX3140_WRITE_DATA_R = 1 << 15,
    MAX3140_WRITE_DATA_TE = 1 << 10,
    MAX3140_WRITE_DATA_RTS = 1 << 9
} MAX3140_WRITE_DATA_t;

typedef enum {
    MAX3140_STOP_COMMUNICATION  = 1 << 16,

    MAX3140_UART_R      = 1 << 15, 
    MAX3140_UART_T      = 1 << 14,
    MAX3140_UART_FEN    = 1 << 13,
    MAX3140_UART_SHDNo  = 1 << 12,
    MAX3140_UART_TM     = 1 << 11,
    MAX3140_UART_RM     = 1 << 10,
    MAX3140_UART_PM     = 1 << 9,
    MAX3140_UART_RAM    = 1 << 8,
    MAX3140_UART_IR     = 1 << 7,
    MAX3140_UART_ST     = 1 << 6,
    // Parity Enable
    MAX3140_UART_PE     = 1 << 5,    
    MAX3140_UART_L      = 1 << 4,
    MAX3140_UART_B3     = 1 << 3,
    MAX3140_UART_B2     = 1 << 2,
    MAX3140_UART_B1     = 1 << 1,
    MAX3140_UART_B0     = 1 << 0,

    MAX3140_PARITY_BIT_INDEX    = 8,
    MAX3140_wd_Pt = 1 << 8    

} MAX3140_UartFlags;

#define MAX3140_WRITE_CONFIG ( MAX3140_UART_R | MAX3140_UART_T )
#define MAX3140_READ_CONFIG ( MAX3140_UART_T )
#define MAX3140_READ_DATA ( 0 )
#define MAX3140_WRITE_DATA ( MAX3140_UART_R )


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

// funny that this function is not standard
static inline int spi_transceive( struct spi_device *spi,
                void *tx_buf, void *rx_buf, size_t len )
{
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = len,
    };

    return spi_sync_transfer( spi, &t, 1 );
}

// ****************************************************************************
// **** START raspicomm private functions ****
// ****************************************************************************
// forward declarations of private functions
static int __init raspicomm_init( void );
static void __exit raspicomm_exit( void );

static ktime_t raspicomm_max3140_get_swbacksleep( speed_t speed );
static unsigned char raspicomm_max3140_get_baudrate_index( speed_t speed );
static unsigned int raspicomm_max3140_get_uart_config( speed_t speed, 
                Databits databits, Stopbits stopbits, Parity parity );
static void raspicomm_max3140_configure( speed_t speed, 
                Databits databits, Stopbits stopbits, Parity parity );
static bool raspicomm_max3140_apply_config( void );
static int raspicomm_spi0_send( unsigned int mosi );
static void raspicomm_rs485_received( struct tty_struct* tty, char c );
static irqreturn_t raspicomm_irq_handler( int irq, void* dev_id );
static int max3140_make_write_data_cmd( int n );

// ****************************************************************************
// *** END raspicomm private functions ****
// ****************************************************************************

// ****************************************************************************
// **** START raspicommDriver functions ****
// ****************************************************************************
static int raspicommDriver_open( struct tty_struct *, struct file * );
static void raspicommDriver_close( struct tty_struct *, struct file * );
static int raspicommDriver_write( struct tty_struct *,
                const unsigned char *, int );
static int raspicommDriver_write_room( struct tty_struct * );
static void raspicommDriver_flush_buffer( struct tty_struct * );
static int raspicommDriver_chars_in_buffer( struct tty_struct * );
static void raspicommDriver_set_termios( struct tty_struct *,
                struct ktermios * );
static void raspicommDriver_stop( struct tty_struct * );
static void raspicommDriver_start( struct tty_struct * );
static void raspicommDriver_hangup( struct tty_struct * );
static int raspicommDriver_tiocmget( struct tty_struct *tty );
static int raspicommDriver_tiocmset( struct tty_struct *tty,
                unsigned int set, unsigned int clear );
static int raspicommDriver_ioctl( struct tty_struct* tty,
                unsigned int cmd, unsigned int long arg );
static void raspicommDriver_throttle( struct tty_struct * tty );
static void raspicommDriver_unthrottle( struct tty_struct * tty );

// ****************************************************************************
// **** END raspicommDriver functions ****
// ****************************************************************************

// ****************************************************************************
// **** START raspicomm private fields ****
// ****************************************************************************
static const struct tty_operations raspicomm_ops = {
    .open               = raspicommDriver_open,
    .close              = raspicommDriver_close,
    .write              = raspicommDriver_write,
    .write_room         = raspicommDriver_write_room,
    .flush_buffer       = raspicommDriver_flush_buffer,
    .chars_in_buffer    = raspicommDriver_chars_in_buffer,
    .ioctl              = raspicommDriver_ioctl,
    .set_termios        = raspicommDriver_set_termios,
    .stop               = raspicommDriver_stop,
    .start              = raspicommDriver_start,
    .hangup             = raspicommDriver_hangup,
    .tiocmget           = raspicommDriver_tiocmget,
    .tiocmset           = raspicommDriver_tiocmset,
    .throttle           = raspicommDriver_throttle,
    .unthrottle         = raspicommDriver_unthrottle
};

#define IRQ_DEV_NAME "raspicomm"
#define PORT_COUNT 1

// the driver instance
static struct tty_driver* raspicommDriver;

static struct tty_port Port;

// the number of open() calls
static int OpenCount;

// ParityIsOdd == true ? odd : even
static int ParityIsOdd;
static int ParityEnabled;

// currently opened tty device
static struct tty_struct* OpenTTY = NULL;

// transmit queue
static queue_t TxQueue;

// variable used in the delay to simulate the baudrate
// static int SwBacksleep;
static ktime_t SwBacksleep;

// config setting of the spi0
static int SpiConfig;

static struct spi_device *spi_slave;

static unsigned int irqGPIO;
static unsigned int irqNumber;

static rpc_spi_msg2_t start_transmitting;
static rpc_spi_msg_t irq_msg_read;
static rpc_spi_msg_t irq_msg_write;
static rpc_spi_msg_t stop_transmitting;
static struct hrtimer last_byte_sent_timer;
static int last_byte_sent_timer_initialized;

// ****************************************************************************
// **** END raspicomm private fields
// ****************************************************************************

static void rpc_spi_msg_init( rpc_spi_msg_t* msg,
            void (*complete)(void*), const char* name )
{
    memset( msg, 0, sizeof(*msg) );
    spi_message_init_no_memset( &msg->msg );
    spi_message_add_tail( &msg->xfer, &msg->msg );
    msg->msg.spi = spi_slave;
    msg->msg.complete = complete;
    msg->msg.context = msg;
    msg->xfer.tx_buf = msg->tx_buf;
    msg->xfer.rx_buf = msg->rx_buf;
    msg->xfer.len = 2;
    msg->name = name;
}

static void rpc_spi_msg2_init( rpc_spi_msg2_t* msg,
            void (*complete)(void*), const char* name )
{
    memset( msg, 0, sizeof(*msg) );
    spi_message_init_no_memset( &msg->msg );
    spi_message_add_tail( &msg->x1.xfer, &msg->msg );
    spi_message_add_tail( &msg->x2.xfer, &msg->msg );
    msg->msg.spi = spi_slave;
    msg->msg.complete = complete;
    msg->msg.context = msg;
    msg->x1.xfer.tx_buf = msg->x1.tx_buf;
    msg->x1.xfer.rx_buf = msg->x1.rx_buf;
    msg->x1.xfer.len = 2;
    msg->x1.xfer.cs_change = 1;
    msg->x2.xfer.tx_buf = msg->x2.tx_buf;
    msg->x2.xfer.rx_buf = msg->x2.rx_buf;
    msg->x2.xfer.len = 2;
    msg->name = name;
}

static int rpc_spi_msg_async( rpc_spi_msg_t* msg, unsigned int tx )
{
    int rc;

    msg->tx_buf[0] = tx >> 8;
    msg->tx_buf[1] = tx;
    rc = spi_async( msg->msg.spi, &msg->msg );
    LOG( "rpc_spi_msg_async(%s,%04X) = %d", msg->name, tx, rc );
    return rc;
}

static int rpc_spi_msg2_async( rpc_spi_msg2_t* msg,
            unsigned int tx1, unsigned int tx2 )
{
    int rc;

    msg->x1.tx_buf[0] = tx1 >> 8;
    msg->x1.tx_buf[1] = tx1;
    msg->x2.tx_buf[0] = tx2 >> 8;
    msg->x2.tx_buf[1] = tx2;
    rc = spi_async( msg->msg.spi, &msg->msg );
    LOG( "rpc_spi_msg2_async(%s,%04X,%04X) = %d", msg->name, tx1, tx2, rc );
    return rc;
}

static unsigned int rpc_spi_msg_rx( void* context )
{
    rpc_spi_msg_t* msg = context;
    unsigned int rc = (msg->rx_buf[0] << 8) | msg->rx_buf[1];
    LOG( "spi_msg_rx(%s) = %04X", msg->name, rc );
    return rc;
}

static void rpc_spi_msg2_rx( void* context, unsigned int* prx1, unsigned int* prx2 )
{
    rpc_spi_msg2_t* msg = context;
    unsigned int rx1 = (msg->x1.rx_buf[0] << 8) | msg->x1.rx_buf[1];
    unsigned int rx2 = (msg->x2.rx_buf[0] << 8) | msg->x2.rx_buf[1];
    LOG( "spi_msg_rx(%s) = %04X,%04X", msg->name, rx1, rx2 );
    if( prx1 ) *prx1 = rx1;
    if( prx2 ) *prx2 = rx2;
}


static void start_transmitting_done( void* context )
{
    int rxdata;

    LOG( "start_transmitting_done" );
    rpc_spi_msg2_rx( context, &rxdata, NULL );
    if( rxdata & MAX3140_UART_R )
    {
        // data is available in the receive register
        // handle the received data
        raspicomm_rs485_received( OpenTTY, rxdata & 0x00FF );
        LOG( "irq_msg_read_done recv: 0x%X", rxdata );
    }
}

static void irq_msg_read_done( void* context )
{
    int rxdata, txdata, rc;

    LOG( "irq_msg_read_done" );
    rxdata = rpc_spi_msg_rx( context );
    if( rxdata & MAX3140_UART_R )
    {
        // data is available in the receive register
        // handle the received data
        raspicomm_rs485_received( OpenTTY, rxdata & 0x00FF );
        LOG( "irq_msg_read_done recv: 0x%X", rxdata );
    }
    if( (rxdata & MAX3140_UART_T) == 0 )
    {
        // nothing to send
        return;
    }
    spin_lock_bh( &dev_lock );
    rc = queue_dequeue( &TxQueue, &txdata );
    if( rc )
    {
        txdata = max3140_make_write_data_cmd( txdata );
    }
    else
    {
        SpiConfig &= ~MAX3140_UART_TM;
        txdata = SpiConfig;
    }
    spin_unlock_bh( &dev_lock ); 
    rpc_spi_msg_async( &irq_msg_write, txdata );
    if( !rc )
    {
        // after the last byte has been sent the transmission is finished
        LOG( "start HR timer last_byte_sent_timer" );
        hrtimer_start( &last_byte_sent_timer, SwBacksleep, HRTIMER_MODE_REL );
    }
}

static void irq_msg_write_done( void* context )
{
    LOG( "irq_msg_write_done" );
    // nothing to do here
}

static enum hrtimer_restart last_byte_sent( struct hrtimer *timer )
{
    int txdata = MAX3140_WRITE_DATA_R | MAX3140_WRITE_DATA_RTS | MAX3140_WRITE_DATA_TE;
    LOG( "last_byte_sent" );
    rpc_spi_msg_async( &stop_transmitting, txdata );
    return HRTIMER_NORESTART;
}

static void stop_transmitting_done( void* context )
{
    LOG( "stop_transmitting_done" );
    // nothing to do here
}
/*
static void _done( void* context )
{
    LOG( "" );
}
*/

// ****************************************************************************
// **** START module specific functions ****
// ****************************************************************************
// module entry point function - gets called from insmod
module_init( raspicomm_init );

// module exit point function - gets called from rmmod
module_exit( raspicomm_exit );
// ****************************************************************************
// **** END module specific functions ****
// ****************************************************************************

// ****************************************************************************
// **** START raspicomm private function implementations
// ****************************************************************************

static void raspicomm_cleanup( void )
{
    LOG( "cleanup all" );

    spin_lock_bh( &dev_lock );
    // set the shutdown flag
    SpiConfig |= MAX3140_STOP_COMMUNICATION;
    // clear the queue
    TxQueue.read = TxQueue.write;
    spin_unlock_bh( &dev_lock ); 

    // remove the interrupt
    if( irqNumber >= 0 )
    {
        LOG( "free_irq" );
        free_irq( irqNumber, NULL );
    }

    // how do i wait for all SPI transfers to finish?
    // maybe try with a delay (mf) ++++
    // udelay( 10000 );
    msleep( 10 );

    // do all the remaining cleanup...
    if( last_byte_sent_timer_initialized )
    {
        hrtimer_cancel( &last_byte_sent_timer );
    }
    if( raspicommDriver )
    {
        LOG( "tty_unregister_driver" );
        tty_unregister_driver( raspicommDriver );
        LOG( "put_tty_driver" );
        put_tty_driver( raspicommDriver );
    }
    if( irqGPIO >= 0 )
    {
        LOG( "gpio_free" );
        gpio_free( irqGPIO );
    }
    if( spi_slave )
    {
        LOG( "spi_unregister_device" );
        spi_unregister_device( spi_slave );
    }
    LOG( "cleanup done" );
}

// initialization function that gets called when the module is loaded
static int __init raspicomm_init( void )
{
    struct spi_master *master;
    struct spi_board_info spi_device_info = {
                .modalias = "raspicommrs485",
                // speed of your device splace can handle
                .max_speed_hz = 1000000, 
                // BUS number
                .bus_num = 0, 
                .chip_select = 0,
                .mode = SPI_MODE_0,
    };
    int pin = 17;
    int result;

    // log the start of the initialization
    LOG( "raspicommrs485 init, compiled " COMPILETIME );

    LOG( "initializing globals" );
    raspicommDriver = NULL;
    memset( &Port, 0, sizeof(Port) );
    OpenCount = 0;
    ParityIsOdd = 0;
    ParityEnabled = 0;
    OpenTTY = NULL;
    memset( &TxQueue, 0, sizeof(TxQueue) );
    SwBacksleep = raspicomm_max3140_get_swbacksleep( 9600 );
    SpiConfig = 0;
    spi_slave = NULL;
    irqGPIO = -EINVAL;
    irqNumber = -EINVAL;
    last_byte_sent_timer_initialized = 0;

    LOG( "spi_busnum_to_master" );
    master = spi_busnum_to_master( 0 );
    if( !master )
    {
        printk( KERN_ERR "spi_busnum_to_master failed" );
        goto cleanup;
    }
    LOG( "spi_new_device" );
    spi_slave = spi_new_device( master, &spi_device_info );
    if( !spi_slave )
    {
        printk( KERN_ERR "spi_new_device failed" );
        goto cleanup;
    }

    // Request a GPIO pin from the driver
    LOG( "gpio_request" );
    result = gpio_request( pin, "rpc0irq" );
    if( result < 0 )
    {
        printk( KERN_ERR "gpio_request failed with code %d", result );
        goto cleanup;
    }
    irqGPIO = pin;
    // 'irqGPIO' is expected to be an unsigned int, i.e. the GPIO number
    // Set GPIO as input
    LOG( "gpio_direction_input" );
    result = gpio_direction_input( irqGPIO );
    if( result < 0 )
    {
        printk( KERN_ERR "gpio_direction_input failed with code %d", result );
        goto cleanup;
    }
    
    // map your GPIO to an IRQ
    LOG( "gpio_to_irq" );
    irqNumber = gpio_to_irq( irqGPIO );
    if( irqNumber < 0 )
    {
        printk( KERN_ERR "gpio_to_irq failed with code %d", result );
        goto cleanup;
    }
    // requested interrupt
    LOG( "request_irq" );
    result = request_irq( irqNumber, raspicomm_irq_handler,
                        // interrupt mode flag
                        IRQF_TRIGGER_FALLING, 
                        // used in /proc/interrupts
                        IRQ_DEV_NAME,                
                        // the *dev_id shared interrupt lines, NULL is okay
                        NULL );                             
    if( result < 0 )
    {
        printk( KERN_ERR "request_irq failed with code %d", result );
        goto cleanup;
    }

    // initialize the port
    LOG( "tty_port_init" );
    tty_port_init( &Port );
    Port.low_latency = 1;

    // allocate the driver
    LOG( "tty_alloc_driver" );
    raspicommDriver = tty_alloc_driver( PORT_COUNT, TTY_DRIVER_REAL_RAW );

    // return if allocation fails
    if( IS_ERR( raspicommDriver ) )
    {
        printk( KERN_ERR "tty_alloc_driver failed" );
        goto cleanup;
    }

    // init the driver
    raspicommDriver->owner                  = THIS_MODULE;
    raspicommDriver->driver_name            = "raspicomm rs485";
    raspicommDriver->name                   = "ttyRPC";
    raspicommDriver->major                  = RaspicommMajorDriverNumber;
    raspicommDriver->minor_start            = 0;
    //raspicommDriver->flags                = TTY_DRIVER_REAL_RAW;
    raspicommDriver->type                   = TTY_DRIVER_TYPE_SERIAL;
    raspicommDriver->subtype                = SERIAL_TYPE_NORMAL;
    raspicommDriver->init_termios           = tty_std_termios;
    raspicommDriver->init_termios.c_ispeed  = 9600;
    raspicommDriver->init_termios.c_ospeed  = 9600;
    raspicommDriver->init_termios.c_cflag   = B9600 | CREAD | CS8 | CLOCAL;

    // initialize function callbacks of tty_driver,
    // necessary before tty_register_driver()
    LOG( "tty_set_operations" );
    tty_set_operations( raspicommDriver, &raspicomm_ops );

    // link the port with the driver
    LOG( "tty_port_link_device" );
    tty_port_link_device( &Port, raspicommDriver, 0 );
    
    // try to register the tty driver
    LOG( "tty_register_driver" );
    if( tty_register_driver( raspicommDriver ) )
    {
        printk( KERN_ERR "tty_register_driver failed" );
        goto cleanup;
    }

    LOG( "initializing messages" );
    rpc_spi_msg2_init( &start_transmitting, start_transmitting_done,
            "start_transmitting" );
    rpc_spi_msg_init( &irq_msg_read, irq_msg_read_done, "irq_msg_read" );
    rpc_spi_msg_init( &irq_msg_write, irq_msg_write_done, "irq_msg_write" );
    rpc_spi_msg_init( &stop_transmitting, stop_transmitting_done,
            "stop_transmitting" );
    LOG( "initializing hrtimer" );
    hrtimer_init( &last_byte_sent_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    last_byte_sent_timer.function = &last_byte_sent;
    last_byte_sent_timer_initialized = 1;

    // successfully initialized the module
    LOG( "raspicomm_init() completed" );
    return 0; 

cleanup:
    raspicomm_cleanup();
    return -ENODEV;
}

// cleanup function that gets called when the module is unloaded
static void __exit raspicomm_exit( void )
{
    LOG( "raspicomm_exit() called" );
    raspicomm_cleanup();
    LOG( "kernel module exit" );
}


// helper function
static unsigned char raspicomm_max3140_get_baudrate_index( speed_t speed )
{
    switch( speed )
    {
        case 600: return 0xF;
        case 1200: return 0xE;
        case 2400: return 0xD;
        case 4800: return 0xC;
        case 9600: return 0xB;
        case 19200: return 0xA;
        case 38400: return 0x9;
        case 57600: return 0x2;
        case 115200: return 0x1;
        case 230400: return 0x0;
        default: return raspicomm_max3140_get_baudrate_index( 9600 );
    }
}

// helper function that creates the config for spi
static unsigned int raspicomm_max3140_get_uart_config( speed_t speed,
                Databits databits, Stopbits stopbits, Parity parity )
{
    unsigned int value = 0;

    value |= MAX3140_WRITE_CONFIG;
    value |= MAX3140_UART_RM;
    value |= raspicomm_max3140_get_baudrate_index( speed );
    value |= stopbits << 6;
    value |= parity << 5;
    value |= databits << 4;

    return value;
}

static ktime_t raspicomm_max3140_get_swbacksleep( speed_t speed )
{
    // +++++ parity and number of bits have to be added to this calculation too
    // return 10000000 / speed;
    int bit_count = 11;
    return ktime_set( 0, (NSEC_PER_SEC / speed) * bit_count );
}

static void raspicomm_max3140_configure( speed_t speed,
                Databits databits, Stopbits stopbits, Parity parity )
{
    ktime_t swBacksleep = raspicomm_max3140_get_swbacksleep( speed );
    int config = raspicomm_max3140_get_uart_config( speed, databits,
                    stopbits, parity );

    LOG( "raspicomm_max3140_configure() called "
        "speed=%i, databits=%i, stopbits=%i, parity=%i "
        "=> config: %X, swBacksleep: %d",
        speed, databits, stopbits, parity,
        config, (int)ktime_to_us(swBacksleep) );

    SpiConfig = config;
    SwBacksleep = swBacksleep;
}

// initializes the spi0 for supplied configuration
static bool raspicomm_max3140_apply_config()
{ 
    int rxconfig;

    // configure the SPI
    raspicomm_spi0_send( SpiConfig );

    // read back the config
    rxconfig = raspicomm_spi0_send( MAX3140_READ_CONFIG );

    if( (rxconfig & 0xFFF) != (SpiConfig & 0xFFF))
    {
        return 0;
    }

    // write data (R set, T not set) and enable receive by disabling RTS
    // (TE set so that no data is sent)
    raspicomm_spi0_send( MAX3140_WRITE_DATA_R | MAX3140_WRITE_DATA_TE | MAX3140_WRITE_DATA_RTS );

    return 1;
}

// Uncommented by javicient

static int max3140_make_write_data_cmd( int data )
{
    data |= MAX3140_WRITE_DATA;
    if( ParityEnabled )
    {
        int n = data;
        // put the xor of the lowest 8 bits into bit 0 of n
        n ^= (n >> 4);
        n ^= (n >> 2);
        n ^= (n >> 1);
        n &= 1;
        // now 0 = even number of one bits, 1 = odd
        // add parity config
        n ^= ParityIsOdd;
        data |= n << MAX3140_PARITY_BIT_INDEX;
    }
    return data;
}

static int raspicomm_spi0_send( unsigned int tx )
{
    unsigned char txbuf[2], rxbuf[2];
    unsigned int rx;
    int rc;

    txbuf[0] = tx >> 8;
    txbuf[1] = tx;
    rc = spi_transceive( spi_slave, txbuf, rxbuf, 2 );
    if( rc < 0 )
    {
        printk( KERN_ERR "spi_transceive( %02X %02X failed with code %d",
                txbuf[0], txbuf[1], rc );
        return 0;
    }
    rx = (rxbuf[0] << 8) | rxbuf[1];
#if 1
    LOG( "raspicomm_spi0_send => %02X %02X <= %02X %02X",
                txbuf[0], txbuf[1], rxbuf[0], rxbuf[1] );
#else
    {
        char buffer[256];
        switch( tx>>14 )
        {
            case 0:
                sprintf( buffer, "RdDat %02X %02X -- %02X %02X R=%d T=%d RA/FE=%d CTS=%d P=%d D=%02X",
                    txbuf[0], txbuf[1],
                    rxbuf[0], rxbuf[1],
                    (rx>>15)&1, (rx>>14)&1,
                    (rx>>10)&1, (rx>>9)&1, (rx>>8)&1, rx&255 );
                break;
            case 1:
                sprintf( buffer, "RdCfg %02X %02X TEST=%d -- %02X %02X R=%d T=%d FEN=%d SHDN=%d TM=%d RM=%d PM=%d RAM=%d IR=%d ST=%d PE=%d L=%d B3=%X",
                    txbuf[0], txbuf[1],
                    tx&1,
                    rxbuf[0], rxbuf[1],
                    (rx>>15)&1, (rx>>14)&1, (rx>>13)&1, (rx>>12)&1,
                    (rx>>11)&1, (rx>>10)&1, (rx>>9)&1, (rx>>8)&1,
                    (rx>>7)&1, (rx>>6)&1, (rx>>5)&1, (rx>>4)&1,
                    rx&15 );
                break;
            case 2:
                sprintf( buffer, "WrDat %02X %02X TE=%d RTS=%d P=%d D=%02X -- %02X %02X R=%d T=%d RA/FE=%d CTS=%d P=%d D=%02X",
                    txbuf[0], txbuf[1],
                    (tx>>10)&1, (tx>>9)&1, (tx>>8)&1, tx&255,
                    rxbuf[0], rxbuf[1],
                    (rx>>15)&1, (rx>>14)&1,
                    (rx>>10)&1, (rx>>9)&1, (rx>>8)&1, rx&255 );
                break;
            case 3:
                sprintf( buffer, "WrCfg %02X %02X FEN=%d SHDN=%d TM=%d RM=%d PM=%d RAM=%d IR=%d ST=%d PE=%d L=%d BR=%X -- %02X %02X R=%d T=%d",
                    txbuf[0], txbuf[1],
                    (tx>>13)&1, (tx>>12)&1, (tx>>11)&1, (tx>>10)&1,
                    (tx>>9)&1, (tx>>8)&1, (tx>>7)&1, (tx>>6)&1,
                    (tx>>5)&1, (tx>>4)&1, tx&15,
                    rxbuf[0], rxbuf[1],
                    (rx>>15)&1, (rx>>14)&1 );
                break;
        }
        LOG( "%s", buffer );
    }
#endif
    return rx;
}

static irqreturn_t raspicomm_irq_handler( int irq, void* dev_id )
{
    LOG( "raspicomm_irq_handler" );
    rpc_spi_msg_async( &irq_msg_read, MAX3140_READ_DATA );
    return IRQ_HANDLED;
}

// this function pushes a received character to the opened tty device,
// called by the interrupt function
static void raspicomm_rs485_received( struct tty_struct* tty, char c )
{
    LOG( "raspicomm_rs485_received(c=%02X)", c );

    if( tty != NULL && tty->port != NULL )
    {
        // send the character to the tty
        tty_insert_flip_char( tty->port, c, TTY_NORMAL );

        // tell it to flip the buffer
        tty_flip_buffer_push( tty->port );
    }
}

// ****************************************************************************
// **** END raspicomm private function implementations
// ****************************************************************************


// ****************************************************************************
// **** START tty driver interface function implementations
// ****************************************************************************
// called by the kernel when open() is called for the device
static int raspicommDriver_open( struct tty_struct* tty, struct file* file )
{
    LOG( "raspicommDriver_open() called" );

    if( OpenCount++ )
    {
        LOG( "raspicommDriver_open() was not successful as OpenCount = %i",
                    OpenCount );

        return -ENODEV;
    }
    else
    {
        LOG( "raspicommDriver_open() was successful" );

        OpenTTY = tty;

        // TODO Do we need to reset the connection?
        // reset the connection
        // raspicomm_max3140_apply_config();

        return SUCCESS;
    }

}

// called by the kernel when close() is called for the device
static void raspicommDriver_close( struct tty_struct* tty, struct file* file )
{
    LOG( "raspicommDriver_close called" );

    if( --OpenCount )
    {
        LOG( "device was not closed, as an open count is %i", OpenCount );
    }
    else
    {
        OpenTTY = NULL;
        LOG( "device was closed" );
    }
}

static int raspicommDriver_write( struct tty_struct* tty,
    const unsigned char* buf, int count )
{
    int rc;
    int data;

    LOG( "raspicommDriver_write(count=%i)", count );
    if( count <= 0 )
    {
        return 0;
    }
    spin_lock_bh( &dev_lock );
    if( SpiConfig & MAX3140_STOP_COMMUNICATION )
    {
        // this device is gone
        rc = -ENODEV;
    }
    else
    {
        rc = 0;
        if( !(SpiConfig & MAX3140_UART_TM) )
        {
            // no transfer in progress or it is sending the last byte
            LOG( "starting transfer" );
            // send the first byte
            data = max3140_make_write_data_cmd( buf[0] );
            SpiConfig |= MAX3140_WRITE_CONFIG | MAX3140_UART_TM;
            rpc_spi_msg2_async( &start_transmitting, data, SpiConfig );
            rc++;
            // cancel a pending EOT
            hrtimer_cancel( &last_byte_sent_timer );
        }
        // add the remaining bytes to the queue, stop if it is full
        while( rc < count )
        {
            if( !queue_enqueue( &TxQueue, buf[rc] ) )
            {
                // queue full
                break;
            }
            rc++;
        }
    }
    spin_unlock_bh( &dev_lock ); 
    LOG( "raspicommDriver_write: %d", rc );
    return rc;
}

// called by kernel to evaluate how many bytes can be written
static int raspicommDriver_write_room( struct tty_struct *tty )
{
    int rc;

    spin_lock_bh( &dev_lock );
    if( SpiConfig & MAX3140_STOP_COMMUNICATION )
    {
        rc = 0;
    }
    else
    {
        rc = queue_get_room( &TxQueue );
    }
    spin_unlock_bh( &dev_lock ); 
    return rc;
}

static void raspicommDriver_flush_buffer( struct tty_struct * tty )
{
    LOG( "raspicommDriver_flush_buffer called" );
}

static int raspicommDriver_chars_in_buffer( struct tty_struct * tty )
{
    int rc;

    spin_lock_bh( &dev_lock );
    rc = QUEUE_SIZE - 1 - queue_get_room( &TxQueue );
    spin_unlock_bh( &dev_lock ); 
    return rc;
}

// called by the kernel when cfsetattr() is called from userspace
static void raspicommDriver_set_termios( struct tty_struct* tty,
                struct ktermios* kt )
{
    int cflag;
    speed_t baudrate;
    Databits databits;
    Parity parity;
    Stopbits stopbits;

    LOG( "raspicommDriver_set_termios() called" );

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
        ParityIsOdd = !!( cflag & PARODD ); 
        parity = PARITY_ON;
        ParityEnabled = 1;
    }
    else
    {
        parity = PARITY_OFF;
        ParityEnabled = 0;
    }

    // #if DEBUG
    //     printk ( KERN_INFO "raspicomm: Parity=%i, ParityIsEven = %i", parity, ParityIsEven);
    // #endif
    
    // update the configuration
    raspicomm_max3140_configure( baudrate, databits, stopbits, parity );

    raspicomm_max3140_apply_config();
}

static void raspicommDriver_stop( struct tty_struct * tty )
{
    LOG( "raspicommDriver_stop called" );
}

static void raspicommDriver_start( struct tty_struct * tty )
{
    LOG( "raspicommDriver_start called" );
}

static void raspicommDriver_hangup( struct tty_struct * tty )
{
    LOG( "raspicommDriver_hangup called" );
}

static int raspicommDriver_tiocmget( struct tty_struct *tty )
{
    LOG( "raspicommDriver_tiocmget called" );
    return 0;
}

static int raspicommDriver_tiocmset( struct tty_struct *tty,
                                unsigned int set, unsigned int clear )
{
    LOG( "raspicommDriver_tiocmset called" );
    return 0;
}

// called by the kernel to get/set data
static int raspicommDriver_ioctl( struct tty_struct* tty,
                                unsigned int cmd, unsigned int long arg )
{
    int ret;

    LOG( "raspicommDriver_ioctl() called with cmd=%i, arg=%li", cmd, arg );
    switch( cmd )
    {
        case TIOCMSET:
            ret = 0;
            break;

        case TIOCMGET:
            ret = 0;
            break;

        default:
            ret = -ENOIOCTLCMD;
            break;
    }
    return ret;
}

static void raspicommDriver_throttle( struct tty_struct * tty )
{
    LOG_INFO( "throttle" );
}

static void raspicommDriver_unthrottle( struct tty_struct * tty )
{
    LOG_INFO( "unthrottle" );
}


// ****************************************************************************
// **** END raspicommDriver interface functions ****
// ****************************************************************************

