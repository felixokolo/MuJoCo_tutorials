MUJOCO_INC = C:/Users/Pheelix/mujoco-3.3.0-windows-x86_64/include
MUJOCO_LIB = C:/Users/Pheelix/mujoco-3.3.0-windows-x86_64/lib \
				-lmujoco

GLFW_INC = C:/Users/Pheelix/glfw-3.4.bin.WIN64/include
GLFW_LIB = C:/Users/Pheelix/glfw-3.4.bin.WIN64/lib-mingw-w64 \
			-lglfw3 -luser32 -lgdi32 -lshell32 -lopengl32

COMPILER = gcc

all:
		$(COMPILER) -I $(MUJOCO_INC) -I $(GLFW_INC) \
		main.c \
		-L $(MUJOCO_LIB) -L $(GLFW_LIB) \
		-o belt.exe
