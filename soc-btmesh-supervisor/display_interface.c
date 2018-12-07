#include <stdio.h>
#include "bg_types.h"
#include "display_interface.h"

display_interface_t display_interface = DEFAULT_DISPLAY_INTERFACE;

void DI_Config(init_func_t init, print_func_t print)
{
  display_interface.init = init;
  display_interface.print = print;
}

void DI_Init(void)
{
  if (display_interface.init == NULL) {
    printf("DI_Init: Display interface is NULL.\r\n");
    return;
  }

  display_interface.init();
}

void DI_Print(char *str, uint8 row)
{
  if (display_interface.print == NULL) {
    printf("DI_Print: %s\r\n", str);
    return;
  }

  display_interface.print(str, row);
}
