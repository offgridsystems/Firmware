#ifndef LCD_DRIVER_H_
#define LCD_DRIVER_H_

#include "hal-config.h"

#if (HAL_SPIDISPLAY_ENABLE == 1)

#include "bg_types.h"

/**
 *  LCD content can be updated one row at a time using function LCD_write().
 *  Row number is passed as parameter,the possible values are defined below.
 */
#define LCD_ROW_NAME         1    /* 1st row, device name */
#define LCD_ROW_STATUS       2    /* 2nd row, node status */
#define LCD_ROW_CONNECTION   3    /* 3rd row, connection status */
#define LCD_ROW_FRIEND       4    /* 4th row, friendship FRIEND status */
#define LCD_ROW_LPN          4    /* 4th row, friendship LPN status */
#define LCD_ROW_LIGHTNESS    5    /* 5th row, lightness level */
#define LCD_ROW_TEMPERATURE  6    /* 6th row, color temperature */
#define LCD_ROW_DELTAUV      7    /* 7th row, delta UV */
#define LCD_ROW_MAX          7    /* total number of rows used */

#define LCD_ROW_LEN        32   /* up to 32 characters per each row */

void LCD_init(void);
void LCD_write(char *str, uint8 row);

#endif /* HAL_SPIDISPLAY_ENABLE */

#endif /* LCD_DRIVER_H_ */
