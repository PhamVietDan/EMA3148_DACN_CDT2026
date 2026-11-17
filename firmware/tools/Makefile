# Command-line build mirroring the CubeIDE Debug config. Run from repo root: make -f tools/Makefile
# App/ is compiled with the strict warning set from PROJECT_PLAN §0.7; generated code keeps -Wall.
TARGET  := Drone
BUILD   := build
PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
SIZE    := $(PREFIX)size
OBJCOPY := $(PREFIX)objcopy

MCU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS    := -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -DSTM32_THREAD_SAFE_STRATEGY=4
INCS    := -ICore/Inc -IUSB_DEVICE/App -IUSB_DEVICE/Target \
           -IDrivers/STM32F4xx_HAL_Driver/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
           -IMiddlewares/Third_Party/FreeRTOS/Source/include \
           -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
           -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
           -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
           -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
           -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include \
           -ICore/ThreadSafe -IApp

CFLAGS_COMMON := $(MCU) -std=gnu11 -g3 -O1 $(DEFS) $(INCS) -ffunction-sections -fdata-sections \
                 -fstack-usage --specs=nano.specs -MMD -MP
CFLAGS_GEN    := $(CFLAGS_COMMON) -Wall
CFLAGS_APP    := $(CFLAGS_COMMON) -Wall -Wextra -Wdouble-promotion -Wshadow -Wformat=2

LDFLAGS := $(MCU) -TSTM32F405RGTX_FLASH.ld --specs=nosys.specs --specs=nano.specs \
           -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--gc-sections -Wl,--print-memory-usage \
           -Wl,--start-group -lc -lm -Wl,--end-group

rwildcard = $(foreach d,$(wildcard $(1:=/*)),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d))

GEN_SRCS := $(call rwildcard,Core,*.c) $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c) \
            $(call rwildcard,Middlewares,*.c) $(call rwildcard,USB_DEVICE,*.c)
APP_SRCS := $(call rwildcard,App,*.c)
ASM_SRCS := $(wildcard Core/Startup/*.s)

GEN_OBJS := $(GEN_SRCS:%.c=$(BUILD)/%.o)
APP_OBJS := $(APP_SRCS:%.c=$(BUILD)/%.o)
ASM_OBJS := $(ASM_SRCS:%.s=$(BUILD)/%.o)

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex

$(BUILD)/$(TARGET).elf: $(GEN_OBJS) $(APP_OBJS) $(ASM_OBJS)
	$(CC) $^ $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(GEN_OBJS): $(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS_GEN) $< -o $@

$(APP_OBJS): $(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS_APP) $< -o $@

$(ASM_OBJS): $(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) -c $(MCU) -x assembler-with-cpp $< -o $@

clean:
	rm -rf $(BUILD)

-include $(GEN_OBJS:.o=.d) $(APP_OBJS:.o=.d)

.PHONY: all clean
