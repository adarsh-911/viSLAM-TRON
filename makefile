CC = gcc
CFLAGS = -Wall -O2 -Iinc -Iinc/feature_detect -Iinc/feature_match -Iinc/init_map -Iinc/tracking_thread

SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

TARGET = out

SRCS := main.c $(shell find $(SRC_DIR) -name "*.c")
OBJS := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS))

all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -g -o $@ $^ -lm

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -g -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) $(TARGET)

.PHONY: all clean
