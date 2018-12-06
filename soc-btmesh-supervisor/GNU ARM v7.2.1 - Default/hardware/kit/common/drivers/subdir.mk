################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hardware/kit/common/drivers/mx25flash_spi.c \
../hardware/kit/common/drivers/udelay.c 

OBJS += \
./hardware/kit/common/drivers/mx25flash_spi.o \
./hardware/kit/common/drivers/udelay.o 

C_DEPS += \
./hardware/kit/common/drivers/mx25flash_spi.d \
./hardware/kit/common/drivers/udelay.d 


# Each subdirectory must supply rules for building sources it contributes
hardware/kit/common/drivers/mx25flash_spi.o: ../hardware/kit/common/drivers/mx25flash_spi.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32BG13P632F512GM48=1' '-D__HEAP_SIZE=0x1200' '-D__STACK_SIZE=0x1000' '-DMESH_LIB_NATIVE=1' '-DHAL_CONFIG=1' -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\CMSIS\Include" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\uartdrv\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\hardware\kit\common\bsp" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\bootloader\api" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emlib\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\protocol\bluetooth\bt_mesh\src" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\radio\rail_lib\common" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\hardware\kit\common\halconfig" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\radio\rail_lib\chip\efr32" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\common\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emlib\src" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\hardware\kit\common\drivers" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\sleep\src" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\sleep\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\halconfig\inc\hal-config" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\gpiointerrupt\inc" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"hardware/kit/common/drivers/mx25flash_spi.d" -MT"hardware/kit/common/drivers/mx25flash_spi.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

hardware/kit/common/drivers/udelay.o: ../hardware/kit/common/drivers/udelay.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32BG13P632F512GM48=1' '-D__HEAP_SIZE=0x1200' '-D__STACK_SIZE=0x1000' '-DMESH_LIB_NATIVE=1' '-DHAL_CONFIG=1' -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\CMSIS\Include" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\uartdrv\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\hardware\kit\common\bsp" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\bootloader\api" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emlib\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\protocol\bluetooth\bt_mesh\src" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\radio\rail_lib\common" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\hardware\kit\common\halconfig" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\radio\rail_lib\chip\efr32" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\common\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emlib\src" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\hardware\kit\common\drivers" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\sleep\src" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\sleep\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\halconfig\inc\hal-config" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\nick\SimplicityStudio\v4_workspace\soc-btmesh-empty\platform\emdrv\gpiointerrupt\inc" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"hardware/kit/common/drivers/udelay.d" -MT"hardware/kit/common/drivers/udelay.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


