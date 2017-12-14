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
#define DEBUG
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

    MAX3140_wd_Pt = 1 << 8    

} MAX3140_UartFlags;

#define MAX3140_WRITE_CONFIG ( MAX3140_UART_R | MAX3140_UART_T )
#define MAX3140_READ_CONFIG ( MAX3140_UART_T )
#define MAX3140_READ_DATA ( 0 )
#define MAX3140_WRITE_DATA ( MAX3140_UART_R )

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

static int raspicomm_max3140_get_swbacksleep(speed_t speed );
static unsigned char raspicomm_max3140_get_baudrate_index( speed_t speed );
static unsigned int raspicomm_max3140_get_uart_config( speed_t speed, 
                Databits databits, Stopbits stopbits, Parity parity );
static void raspicomm_max3140_configure( speed_t speed, 
                Databits databits, Stopbits stopbits, Parity parity );
static bool raspicomm_max3140_apply_config( void );
static int raspicomm_spi0_send( unsigned int mosi );
static void raspicomm_rs485_received( struct tty_struct* tty, char c );
static irqreturn_t raspicomm_irq_handler( int irq, void* dev_id );

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
static void raspicomm_start_transfer( void );

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

// ParityIsEven == true ? even : odd
static int ParityIsEven;
static int ParityEnabled;

// currently opened tty device
static struct tty_struct* OpenTTY = NULL;

// transmit queue
static queue_t TxQueue;

// variable used in the delay to simulate the baudrate
static int SwBacksleep;

// config setting of the spi0
static int SpiConfig;

static struct spi_device *spi_slave;

static unsigned int irqGPIO;
static unsigned int irqNumber;

// ****************************************************************************
// **** END raspicomm private fields
// ****************************************************************************


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
    if( raspicommDriver )
    {
        LOG( "tty_unregister_driver" );
        tty_unregister_driver( raspicommDriver );
        LOG( "put_tty_driver" );
        put_tty_driver( raspicommDriver );
    }
    if( irqNumber >= 0 )
    {
        LOG( "free_irq" );
        free_irq( irqNumber, NULL );
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
    LOG( "kernel module initialization" );


    LOG( "initializing globals" );
    raspicommDriver = NULL;
    memset( &Port, 0, sizeof(Port) );
    OpenCount = 0;
    ParityIsEven = 1;
    ParityEnabled = 0;
    OpenTTY = NULL;
    memset( &TxQueue, 0, sizeof(TxQueue) );
    SwBacksleep = 0;
    SpiConfig = 0;
    spi_slave = NULL;
    irqGPIO = -EINVAL;
    irqNumber = -EINVAL;

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

static int raspicomm_max3140_get_swbacksleep( speed_t speed )
{
    return 10000000 / speed;
}

static void raspicomm_max3140_configure( speed_t speed,
                Databits databits, Stopbits stopbits, Parity parity )
{
    int swBacksleep = raspicomm_max3140_get_swbacksleep( speed );
    int config = raspicomm_max3140_get_uart_config( speed, databits,
                    stopbits, parity );

    LOG( "raspicomm_max3140_configure() called "
        "speed=%i, databits=%i, stopbits=%i, parity=%i "
        "=> config: %X, swBacksleep: %i",
        speed, databits, stopbits, parity,
        config, swBacksleep );

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

static int raspicomm_max3140_get_parity_flag( char c )
{
    // even parity: is 1 if number of ones is odd
    // -> making number of bits of value and parity = even
    // odd parity: is 1 if the number of ones is even
    // -> making number of bits of value and parity = odd

    int parityEven = ParityIsEven;
    int parityEnabled = ParityEnabled;
    int count = 0, i;
    int ret;

    if( parityEnabled == 0 )
        return 0;

    // count the number of ones    
     for( i = 0; i < 8; i++ )
         if( c & (1 << i) )
             count++;

     if( parityEven )
         ret = (count % 2) ? MAX3140_wd_Pt : 0;
     else
         ret = (count % 2) ? 0 : MAX3140_wd_Pt;

     LOG( "raspicomm_max3140_get_parity_flag(c=%c) parityEven=%i, count=%i, ret=%i", c, parityEven, count, ret );

     return ret;
 }

static void raspicomm_start_transfer()
{
    int rxdata, txdata;
    // a place to store the previous spin lock state
    unsigned long flags; 
    // AHB vor Eintritt
    spin_lock_irqsave( &dev_lock, flags ); 

    // TBE-interrupt enable, falls das noch nicht erledigt ist
    if( SpiConfig & MAX3140_UART_TM )
    {
        // bereits eingeschaltet --> nichts mehr tun,
        // der TBE-IR sorgt schon irgendwann für ein Leeren der Queue
        rxdata = 0;
    }
    else
    {
        // noch nicht eingeschaltet --> jetzt einschalten
        SpiConfig = SpiConfig | MAX3140_WRITE_CONFIG | MAX3140_UART_TM;
        rxdata = raspicomm_spi0_send( SpiConfig );
    }

    if( rxdata & MAX3140_UART_T )
    {
        // TBE --> senden möglich
        if( queue_dequeue( &TxQueue, &txdata ) )
        {
            // Byte zum Senden da --> gleich erledigen
            // send the data (RTS enabled) AHB rxdata für Log
            rxdata = raspicomm_spi0_send( MAX3140_WRITE_DATA | txdata | raspicomm_max3140_get_parity_flag( (char)txdata) );
            // AHB
            LOG( "raspicomm_start_transfer: 0x%X --> 0x%X", txdata, rxdata ); 
        } 
    }

    // AHB Freigabe des Locks
    spin_unlock_irqrestore( &dev_lock, flags ); 
}

static int raspicomm_spi0_send( unsigned int mosi )
{
    unsigned char tx[2], rx[2];
    int rc;

    tx[0] = mosi >> 8;
    tx[1] = mosi;
    rc = spi_transceive( spi_slave, tx, rx, 2 );
    if( rc < 0 )
    {
        printk( KERN_ERR "spi_transceive( %02X %02X failed with code %d",
                tx[0], tx[1], rc );
        return 0;
    }
    LOG( "raspicomm_spi0_send => %02X %02X <= %02X %02X", tx[0], tx[1], rx[0], rx[1] );
    return (tx[0] << 8) | tx[1];
}

// irq handler, that gets fired when the gpio 17 falling edge occurs
static irqreturn_t raspicomm_irq_handler( int irq, void* dev_id )
{
    int rxdata, txdata;
    // a place to store the previous spin lock state
    unsigned long flags;

    // AHB Der Zugriff auf den UART wird durch einen Spinlock abgesichert
    // (exklusiver Zugriff), somit kann auch
    //         ... start_transfer() auf die UART zugreifen
    spin_lock_irqsave( &dev_lock, flags );

    // issue a read command to discover the cause of the interrupt
    rxdata = raspicomm_spi0_send( MAX3140_READ_DATA );

    if( rxdata & MAX3140_UART_R )
    {
        // data is available in the receive register
        // handle the received data
        raspicomm_rs485_received( OpenTTY, rxdata & 0x00FF );
        LOG( "raspicomm_irq recv: 0x%X", rxdata );
    }
    if( rxdata & MAX3140_UART_T )
    {
        // the transmit buffer is empty
        // get the data to send from the transmit queue
        if( queue_dequeue( &TxQueue, &txdata ) )
        {
            // send the data (RTS enabled) AHB rxdata für Log
            rxdata = raspicomm_spi0_send( MAX3140_WRITE_DATA | txdata | raspicomm_max3140_get_parity_flag( (char)txdata ) );
            LOG( "raspicomm_irq sent: 0x%X --> 0x%X", txdata, rxdata );
        }
        else
        {
            // set bits R + T (bit 15 + bit 14) and clear TM (bit 11) transmit buffer empty
            SpiConfig = (SpiConfig | MAX3140_WRITE_CONFIG) & ~MAX3140_UART_TM;
            raspicomm_spi0_send( SpiConfig );

            // give the max3140 enough time to send the data over usart
            // before disabling RTS, else the transmission is broken
            // AHB Erläuterung: Microsekunden Verzögerung:
            // 10.000.000/Baudrate: 9600 --> ca. 1 mSec
            spin_unlock_irqrestore( &dev_lock, flags ); 
            udelay( SwBacksleep ); 
            spin_lock_irqsave( &dev_lock, flags );

            // did anybody add more daya to send?
            if( queue_dequeue( &TxQueue, &txdata ) )
            {
                // send the data (RTS enabled) AHB rxdata für Log
                rxdata = raspicomm_spi0_send( MAX3140_WRITE_DATA | txdata | raspicomm_max3140_get_parity_flag( (char)txdata ) );
                LOG( "raspicomm_irq sent: 0x%X --> 0x%X", txdata, rxdata );
            }
            else
            {
                // enable receive by disabling RTS
                // (TE set so that no data is sent)
                raspicomm_spi0_send( MAX3140_WRITE_DATA_R | MAX3140_WRITE_DATA_RTS | MAX3140_WRITE_DATA_TE );
                // AHB
                LOG( "raspicomm_irq RTS disabled --> receiving..." ); 
            }
        }
    } 

    // AHB Freigabe des Locks
    spin_unlock_irqrestore( &dev_lock, flags ); 

    return IRQ_HANDLED;
}

// this function pushes a received character to the opened tty device,
// called by the interrupt function
static void raspicomm_rs485_received( struct tty_struct* tty, char c )
{
    LOG( "raspicomm_rs485_received(c=%c)", c );

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
    int bytes_written = 0;
    // a place to store the previous spin lock state
    unsigned long flags; 

    LOG( "raspicommDriver_write(count=%i)\n", count );

    spin_lock_irqsave( &dev_lock, flags );
    while( bytes_written < count )
    {
        if( queue_enqueue( &TxQueue, buf[bytes_written] ) )
        {
            bytes_written++;
        }
        else
        {
            // kein Platz mehr vorhanden --> schlafen, senden
            spin_unlock_irqrestore( &dev_lock, flags ); 
            // (mf) is this the right order and the right thing to do here? +++
            cpu_relax();

            raspicomm_start_transfer();
            spin_lock_irqsave( &dev_lock, flags );
        } 
    }
    spin_unlock_irqrestore( &dev_lock, flags ); 
    // AHB Sorge dafür, dass der TBE interrupt auf jeden Fall aktiviert wird
    // (falls nicht bereits oben bei Platzmangel)
    raspicomm_start_transfer();

    return bytes_written;
}

// called by kernel to evaluate how many bytes can be written
static int raspicommDriver_write_room( struct tty_struct *tty )
{
    return INT_MAX;
}

static void raspicommDriver_flush_buffer( struct tty_struct * tty )
{
    LOG( "raspicommDriver_flush_buffer called" );
}

static int raspicommDriver_chars_in_buffer( struct tty_struct * tty )
{
    //LOG("raspicommDriver_chars_in_buffer called");
    return 0;
}

// called by the kernel when cfsetattr() is called from userspace
static void raspicommDriver_set_termios( struct tty_struct* tty,
                struct ktermios* kt )
{
    int cflag;
    speed_t baudrate; Databits databits; Parity parity; Stopbits stopbits;

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
        ParityIsEven = !( cflag & PARODD ); 
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

