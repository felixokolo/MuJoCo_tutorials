PROJECT_INC = src/logic/include

MUJOCO_INC = C:/Users/Pheelix/mujoco-3.3.0-windows-x86_64/include
MUJOCO_LIB = C:/Users/Pheelix/mujoco-3.3.0-windows-x86_64/lib
MUJOCO_LINK = -lmujoco

GLFW_INC = C:/Users/Pheelix/glfw-3.4.bin.WIN64/include
GLFW_LIB = C:/Users/Pheelix/glfw-3.4.bin.WIN64/lib-mingw-w64
GLFW_LINK = -lglfw3 -luser32 -lgdi32 -lshell32 -lopengl32

COMPILER = gcc
TARGET = bin/belt.exe

SRC = src/main.c src/logic/logic.c
OBJ = $(SRC:src/%.c=obj/%.o)

CFLAGS = -I$(PROJECT_INC) -I$(MUJOCO_INC) -I$(GLFW_INC)
LDFLAGS = -L$(MUJOCO_LIB) -L$(GLFW_LIB) $(MUJOCO_LINK) $(GLFW_LINK)

all: $(TARGET)

$(TARGET): $(OBJ) | bin
	$(COMPILER) $(OBJ) -o $@ $(LDFLAGS)

obj/%.o: src/%.c | obj
	@mkdir -p $(dir $@)
	$(COMPILER) -c $< -o $@ $(CFLAGS)

bin:
	mkdir -p bin

obj:
	mkdir -p obj

clean:
	rm -rf obj bin
