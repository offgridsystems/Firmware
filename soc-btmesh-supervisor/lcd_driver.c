#include <stdio.h>
#include <string.h>
#include "graphics.h"
#include "lcd_driver.h"

#if (HAL_SPIDISPLAY_ENABLE == 1)

static char LCD_data[LCD_ROW_MAX][LCD_ROW_LEN];   /* 2D array for storing the LCD content */

// this is needed by the LCD driver
int rtcIntCallbackRegister(void (*pFunction)(void*),
                           void* argument,
                           unsigned int frequency)
{
  return 0;
}

/**
 * LCD initialization, called once at startup.
 */
void LCD_init(void)
{
  memset(&LCD_data, 0, sizeof(LCD_data));

  graphInit("SILICON LABORATORIES\nBluetooth Mesh Demo\n\n");

  LCD_write("initializing", LCD_ROW_STATUS);
}

/**
 * This function is used to write one line in the LCD. The parameter 'row' selects which line
 * is written, possible values are defined as LCD_ROW_xxx.
 */
void LCD_write(char *str, uint8 row)
{
  char LCD_message[LCD_ROW_MAX * LCD_ROW_LEN];
  char *pRow;
  int i;

  if (row > LCD_ROW_MAX) {
    return;
  }

  pRow  = &(LCD_data[row - 1][0]);

  sprintf(pRow, str);

  if (row == LCD_ROW_LIGHTNESS) {
    strcat(pRow, "%");
  }

  LCD_message[0] = 0;

  for (i = 0; i < LCD_ROW_MAX; i++) {
    pRow  = &(LCD_data[i][0]);
    strcat(LCD_message, pRow);
    strcat(LCD_message, "\n"); // add newline at end of reach row
  }

  graphWriteString(LCD_message);
}

#endif /* HAL_SPIDISPLAY_ENABLE */
