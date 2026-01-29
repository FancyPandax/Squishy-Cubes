# Compiler and flags
CXX = g++
CXXFLAGS = -IC:/raylib/raylib/include -std=c++17 -Wall

# Linker flags (order matters on Windows/MSYS2)
LDFLAGS = -LC:/raylib/raylib/build -lopengl32 -lgdi32 -lwinmm -lraylib

# Source and output
SRC = main.cpp
OUT = main.exe

# Default target
all: $(OUT)

# Compile
$(OUT): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(OUT) $(LDFLAGS)

# Clean
clean:
	rm -f $(OUT)

# Run
run: $(OUT)
	./$(OUT)
