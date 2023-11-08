DEBUG = 1
BUILD_DIR = build

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

SRC = \
	src/startup.c \
	src/main.c \
	src/rgb.c \
	src/RFM69.c \
	src/adc.c \
	src/uart.c \
	src/timer.c

CFLAGS += \
	-mcpu=cortex-m0plus \
	-mthumb \
	-ffreestanding \
	-nostdlib \
	-fanalyzer \
	-fanalyzer-checker=taint \
	-Og \
	-Wall \
	-Werror \
	-fdata-sections \
	-ffunction-sections \
	-DSTM32G031xx \
	-MMD -MP -MF"$(@:%.o=%.d)" \
	-lc \
	-lnosys \
	-specs=nano.specs \
	-TSTM32G031K8Tx_FLASH.ld \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
	-Wl,--gc-sections \
	-Idrivers/CMSIS/Device/ST/STM32G0xx/Include \
	-Idrivers/CMSIS/Include

ifeq ($(DEBUG), 1)
	CFLAGS += -g -gdwarf-2
endif

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(SRC:.c=.o)))
vpath %.c $(sort $(dir $(SRC)))


all: \
	$(BUILD_DIR)/sensor.elf $(BUILD_DIR)/sensor.hex \
	$(BUILD_DIR)/receiver.elf $(BUILD_DIR)/receiver.hex

$(BUILD_DIR)/%.o: %.c | Makefile $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.elf: $(OBJECTS) $(BUILD_DIR)/%.o | Makefile
	$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)
