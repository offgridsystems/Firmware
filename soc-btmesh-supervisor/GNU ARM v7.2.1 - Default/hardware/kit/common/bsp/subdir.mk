################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hardware/kit/common/bsp/bsp_stk.c 

OBJS += \
./hardware/kit/common/bsp/bsp_stk.o 

C_DEPS += \
./hardware/kit/common/bsp/bsp_stk.d 


# Each subdirectory must supply rules for building sources it contributes
hardware/kit/common/bsp/bsp_stk.o: ../hardware/kit/common/bsp/bsp_stk.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-D__HEAP_SIZE=0x1200' '-D__STACK_SIZE=0x1000' '-DMESH_LIB_NATIVE=1' '-DHAL_CONFIG=1' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\emdrv\uartdrv\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\halconfig\inc\hal-config" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\bootloader\api" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\hardware\kit\common\drivers" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\radio\rail_lib\common" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\emdrv\sleep\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\emdrv\common\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\emdrv\sleep\src" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\emlib\src" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\protocol\bluetooth\bt_mesh\src" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\radio\rail_lib\chip\efr32" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\emlib\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\CMSIS\Include" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\hardware\kit\common\bsp" -I"C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\hardware\kit\common\halconfig" -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"hardware/kit/common/bsp/bsp_stk.d" -MT"hardware/kit/common/bsp/bsp_stk.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


