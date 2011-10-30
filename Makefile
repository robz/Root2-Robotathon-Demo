##########################################################################
# User configuration and firmware specific object files	
##########################################################################

# The target, flash and ram of the LPC1xxx microprocessor.
# Use for the target the value: LPC11xx, LPC13xx or LPC17xx
TARGET = LPC13xx
FLASH = 32K
SRAM = 8K

# For USB HID support the LPC134x reserves 384 bytes from the sram,
# if you don't want to use the USB features, just use 0 here.
SRAM_USB = 384

VPATH = 
OBJS = main.o

##########################################################################
# Project-specific files 
##########################################################################

VPATH += project
OBJS += commands.o 

VPATH += project/commands
OBJS += cmd_chibi_addr.o cmd_chibi_tx.o
OBJS += cmd_i2ceeprom_read.o cmd_i2ceeprom_write.o cmd_lm75b_gettemp.o
OBJS += cmd_reset.o cmd_sd_dir.o cmd_sysinfo.o cmd_uart.o 
OBJS += cmd_roundedcorner.o

VPATH += project/commands/drawing
OBJS += cmd_backlight.o cmd_bmp.o cmd_button.o cmd_calibrate.o
OBJS += cmd_circle.o cmd_clear.o cmd_line.o cmd_orientation.o
OBJS += cmd_pixel.o cmd_progress.o cmd_rectangle.o cmd_text.o
OBJS += cmd_textw.o cmd_tsthreshhold.o cmd_tswait.o cmd_triangle.o

VPATH += project/robotathon_drivers
OBJS += linesensor.o encoder.o motor.o servo.o IR.o usb.o root2.o

##########################################################################
# Optional driver files 
##########################################################################

# 4K EEPROM
VPATH += drivers/eeprom drivers/eeprom/mcp24aa
OBJS += eeprom.o mcp24aa.o

##########################################################################
# Library files 
##########################################################################

VPATH += core core/adc core/cmd core/cpu core/gpio core/i2c core/pmu
VPATH += core/ssp core/systick core/timer16 core/timer32 core/uart
VPATH += core/usbhid-rom core/libc core/wdt core/usbcdc core/pwm
VPATH += core/iap
OBJS += adc.o cpu.o cmd.o gpio.o i2c.o pmu.o ssp.o systick.o timer16.o
OBJS += timer32.o uart.o uart_buf.o usbconfig.o usbhid.o stdio.o string.o
OBJS += wdt.o cdcuser.o cdc_buf.o usbcore.o usbdesc.o usbhw.o usbuser.o 
OBJS += sysinit.o pwm.o iap.o

##########################################################################
# GNU GCC compiler prefix and location
##########################################################################

CROSS_COMPILE = arm-none-eabi-
AS = $(CROSS_COMPILE)gcc
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
OUTFILE = firmware
LPCRC = ./lpcrc
CP = cp

##########################################################################
# GNU GCC compiler flags
##########################################################################
ROOT_PATH = .
INCLUDE_PATHS = -I$(ROOT_PATH) -I$(ROOT_PATH)/project

##########################################################################
# Startup files
##########################################################################

LD_PATH = lpc1xxx
LD_SCRIPT = $(LD_PATH)/linkscript.ld
LD_TEMP = $(LD_PATH)/memory.ld

ifeq (LPC11xx,$(TARGET))
  CORTEX_TYPE=m0
else
  CORTEX_TYPE=m3
endif

CPU_TYPE = cortex-$(CORTEX_TYPE)
VPATH += lpc1xxx
OBJS += $(TARGET)_handlers.o LPC1xxx_startup.o

##########################################################################
# Compiler settings, parameters and flags
##########################################################################

CFLAGS  = -c -g -Os $(INCLUDE_PATHS) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CPU_TYPE) -DTARGET=$(TARGET) -fno-builtin
ASFLAGS = -c -g -Os $(INCLUDE_PATHS) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CPU_TYPE) -D__ASSEMBLY__ -x assembler-with-cpp
LDFLAGS = -nostartfiles -mthumb -mcpu=$(CPU_TYPE) -Wl,--gc-sections
LDLIBS  = -lm
OCFLAGS = --strip-unneeded

all: firmware

%.o : %.c
	$(CC) $(CFLAGS) -o $@ $<

%.o : %.s
	$(AS) $(ASFLAGS) -o $@ $<

firmware: $(OBJS) $(SYS_OBJS)
	-@echo "MEMORY" > $(LD_TEMP)
	-@echo "{" >> $(LD_TEMP)
	-@echo "  flash(rx): ORIGIN = 0x00000000, LENGTH = $(FLASH)" >> $(LD_TEMP)
	-@echo "  sram(rwx): ORIGIN = 0x10000000+$(SRAM_USB), LENGTH = $(SRAM)-$(SRAM_USB)" >> $(LD_TEMP)
	-@echo "}" >> $(LD_TEMP)
	-@echo "INCLUDE $(LD_SCRIPT)" >> $(LD_TEMP)
	$(LD) $(LDFLAGS) -T $(LD_TEMP) -o $(OUTFILE).elf $(OBJS) $(LDLIBS)
	-@echo ""
	$(SIZE) $(OUTFILE).elf
	-@echo ""
	$(OBJCOPY) $(OCFLAGS) -O binary $(OUTFILE).elf $(OUTFILE).bin
	$(OBJCOPY) $(OCFLAGS) -O ihex $(OUTFILE).elf $(OUTFILE).hex
	-@echo ""
	$(LPCRC) firmware.bin

clean:
	rm -f $(OBJS) $(LD_TEMP) $(OUTFILE).elf $(OUTFILE).bin $(OUTFILE).hex
