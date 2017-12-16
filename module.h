#include <linux/kernel.h>     // Needed for KERN_INFO
#include <linux/module.h>


// ****************************************************************************
// **** END Module Defines ****
// ****************************************************************************

#define SUCCESS 0 

// outputs verbose debug information
//#define DEBUG

/* if DEBUG is defined, log using printk(), else do nothing */
#ifdef DEBUG
  #define LOG(fmt, args...) do { printk( KERN_DEBUG "rpc: " fmt "\n", ## args); } while(0)
#else
  #define LOG(fmt, args...) do {} while(0);
#endif

/* log maps always to printk */
#define LOG_INFO(fmt, args...) do { printk( KERN_INFO "rpc" ": " fmt "\n", ## args); } while(0)
