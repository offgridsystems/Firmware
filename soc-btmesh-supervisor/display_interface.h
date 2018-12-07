#ifndef _DISPLAY_INTERFACE_H
#define _DISPLAY_INTERFACE_H

/* radio board type ID is defined here */
#include "ble-configuration.h"

typedef void (*init_func_t)(void);
typedef void (*print_func_t)(char *str, uint8 row);

typedef struct {
  char *buff;
  int   row;
}display_interface_data_t;

typedef struct {
  init_func_t  init;
  print_func_t print;
}display_interface_t;

#if (EMBER_AF_BOARD_TYPE == BRD4104A)  \
  || (EMBER_AF_BOARD_TYPE == BRD4305A) \
  || (EMBER_AF_BOARD_TYPE == BRD4305C) \
  || (EMBER_AF_BOARD_TYPE == BRD4158A) \
  || (EMBER_AF_BOARD_TYPE == BRD4159A) \
  || (EMBER_AF_BOARD_TYPE == BRD4167A) \
  || (EMBER_AF_BOARD_TYPE == BRD4168A) \
  || (EMBER_AF_BOARD_TYPE == BRD4305D) \
  || (EMBER_AF_BOARD_TYPE == BRD4305E) \
  || (EMBER_AF_BOARD_TYPE == BRD4103A) \
  || (EMBER_AF_BOARD_TYPE == BRD4161A) \
  || (EMBER_AF_BOARD_TYPE == BRD4162A) \
  || (EMBER_AF_BOARD_TYPE == BRD4163A) \
  || (EMBER_AF_BOARD_TYPE == BRD4164A) \
  || (EMBER_AF_BOARD_TYPE == BRD4170A) \
  || (EMBER_AF_BOARD_TYPE == BRD4306A) \
  || (EMBER_AF_BOARD_TYPE == BRD4306B) \
  || (EMBER_AF_BOARD_TYPE == BRD4306C) \
  || (EMBER_AF_BOARD_TYPE == BRD4306D) \
  || (EMBER_AF_BOARD_TYPE == BRD4304A) \
  || (EMBER_AF_BOARD_TYPE == BRD4165A) \
  || (EMBER_AF_BOARD_TYPE == BRD4165B)

#include "lcd_driver.h"

#define DI_ROW_NAME         LCD_ROW_NAME
#define DI_ROW_STATUS       LCD_ROW_STATUS
#define DI_ROW_CONNECTION   LCD_ROW_CONNECTION
#define DI_ROW_FRIEND       LCD_ROW_FRIEND
#define DI_ROW_LPN          LCD_ROW_LPN
#define DI_ROW_LIGHTNESS    LCD_ROW_LIGHTNESS
#define DI_ROW_TEMPERATURE  LCD_ROW_TEMPERATURE
#define DI_ROW_DELTAUV      LCD_ROW_DELTAUV

#define DEFAULT_DISPLAY_INTERFACE \
  {                               \
    LCD_init,                     \
    LCD_write                     \
  }

#else

#define DI_ROW_NAME         0
#define DI_ROW_STATUS       0
#define DI_ROW_CONNECTION   0
#define DI_ROW_FRIEND       0
#define DI_ROW_LPN          0
#define DI_ROW_LIGHTNESS    0
#define DI_ROW_TEMPERATURE  0
#define DI_ROW_DELTAUV      0

#define DEFAULT_DISPLAY_INTERFACE \
  {                               \
    NULL,                         \
    NULL                          \
  }

#endif

void DI_Config(init_func_t init, print_func_t print);
void DI_Init(void);
void DI_Print(char *str, uint8 row);

#endif /* _DISPLAY_INTERFACE_H */
