#include "mbed.h"

/** Si7013 Read Temperature Command */
#define SI7013_READ_TEMP_POST   0xE0 /* Read previous T data from RH measurement command*/
#define SI7013_READ_TEMP        0xE3 /* Stand-alone read temperature command */

/** Si7013 Read RH Command */
#define SI7013_READ_RH          0xE5 /* Perform RH (and T) measurement. */
/** Si7013 Read ID */
#define SI7013_READ_ID1_1       0xFA
#define SI7013_READ_ID1_2       0x0F
#define SI7013_READ_ID2_1       0xFc
#define SI7013_READ_ID2_2       0xc9
/** Si7013 Read Firmware Revision */
#define SI7013_READ_FWREV_1     0x84
#define SI7013_READ_FWREV_2     0xB8
 
/** I2C device address for Si7013 */
#define SI7013_ADDR      0x82
/** I2C device address for Si7021 */
#define SI7021_ADDR      0x80
 
/** Device ID value for Si7013 */
#define SI7013_DEVICE_ID 0x0D
/** Device ID value for Si7020 */
#define SI7020_DEVICE_ID 0x14
/** Device ID value for Si7021 */
#define SI7021_DEVICE_ID 0x15