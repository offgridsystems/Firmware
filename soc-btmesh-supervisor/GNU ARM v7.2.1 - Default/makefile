################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include protocol/bluetooth/lib/EFR32XG13X/GCC/subdir.mk
-include protocol/bluetooth/bt_mesh/src/subdir.mk
-include platform/emlib/src/subdir.mk
-include platform/emdrv/sleep/src/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG13P/Source/subdir.mk
-include hardware/kit/common/drivers/subdir.mk
-include hardware/kit/common/bsp/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: soc-btmesh-supervisor.axf

# Tool invocations
soc-btmesh-supervisor.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\efr32bg13p632f512gm48.ld" -Wl,--undefined,bg_application_properties,--undefined,__Vectors,--undefined,__aeabi_uldivmod,--undefined,ceil -Xlinker -no-enum-size-warning -Xlinker -no-wchar-size-warning -Xlinker --gc-sections -Xlinker -Map="soc-btmesh-supervisor.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o soc-btmesh-supervisor.axf -Wl,--start-group "./dcd.o" "./display_interface.o" "./gatt_db.o" "./init_app.o" "./init_board.o" "./init_mcu.o" "./lcd_driver.o" "./main.o" "./pti.o" "./hardware/kit/common/bsp/bsp_stk.o" "./hardware/kit/common/drivers/mx25flash_spi.o" "./hardware/kit/common/drivers/retargetserial.o" "./hardware/kit/common/drivers/udelay.o" "./platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.o" "./platform/Device/SiliconLabs/EFR32BG13P/Source/system_efr32bg13p.o" "./platform/emdrv/sleep/src/sleep.o" "./platform/emlib/src/em_cmu.o" "./platform/emlib/src/em_core.o" "./platform/emlib/src/em_cryotimer.o" "./platform/emlib/src/em_crypto.o" "./platform/emlib/src/em_emu.o" "./platform/emlib/src/em_gpio.o" "./platform/emlib/src/em_i2c.o" "./platform/emlib/src/em_msc.o" "./platform/emlib/src/em_rmu.o" "./platform/emlib/src/em_rtcc.o" "./platform/emlib/src/em_system.o" "./platform/emlib/src/em_timer.o" "./platform/emlib/src/em_usart.o" "./protocol/bluetooth/bt_mesh/src/bg_application_properties.o" "./protocol/bluetooth/bt_mesh/src/mesh_lib.o" "./protocol/bluetooth/bt_mesh/src/mesh_serdeser.o" "C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\protocol\bluetooth\lib\EFR32XG13X\GCC\libcoex.a" "C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\protocol\bluetooth\lib\EFR32XG13X\GCC\libbluetooth_mesh.a" "C:\Users\nick\SimplicityStudio\v4_workspace\dkblock\soc-btmesh-supervisor\protocol\bluetooth\lib\EFR32XG13X\GCC\binapploader.o" -lm -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: soc-btmesh-supervisor.hex'
	arm-none-eabi-objcopy -O ihex "soc-btmesh-supervisor.axf" "soc-btmesh-supervisor.hex"
	@echo ' '

	@echo 'Building bin file: soc-btmesh-supervisor.bin'
	arm-none-eabi-objcopy -O binary "soc-btmesh-supervisor.axf" "soc-btmesh-supervisor.bin"
	@echo ' '

	@echo 'Building s37 file: soc-btmesh-supervisor.s37'
	arm-none-eabi-objcopy -O srec "soc-btmesh-supervisor.axf" "soc-btmesh-supervisor.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "soc-btmesh-supervisor.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) soc-btmesh-supervisor.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
