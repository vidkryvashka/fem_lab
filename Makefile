CC = gcc
CXX = clang++

SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
TARGET = $(BIN_DIR)/program

CFLAGS_W = -Wall -Wno-unused-variable -Wno-unused-function -Wno-pointer-arith
CFLAGS_I = -Iinclude $(shell pkg-config --cflags raylib) -Icomponents/raygui
CFLAGS = $(CFLAGS_W) $(CFLAGS_I) -O2

# platform dependent
RAYLIB_LINK = $(shell pkg-config --libs raylib) -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
LDFLAGS  = $(RAYLIB_LINK) -lm -lpthread -ldl

SRCS = $(shell find $(SRC_DIR) -name "*.c")
OBJS = $(SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
OBJ_SUBDIRS = $(sort $(dir $(OBJS)))

all: prepare $(TARGET)

prepare:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(OBJ_SUBDIRS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/raygui.o: components/raygui/raygui.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

compile: $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS)
	rm -rf $(BIN_DIR)/*

.PHONY: all prepare clean run

help:
	@echo "Available targets:"
	@echo "  all			: Compile the project"
	@echo "  run			: Run here"
	@echo "  clean			: Remove object files and executable"
