# Makefile for the Angry Birds Prototype

# Compiler and Linker
CXX = g++

# --- Project Structure ---
# Directories for source and object files
SRC_DIR = src
OBJ_DIR = obj

# Name of the final executable
EXEC = angry_birds_prototype

# --- Compiler and Linker Flags ---

# Compiler flags:
# -std=c++17: Use the C++17 standard
# -Wall: Enable all warnings
# -g: Generate debugging information
# -I$(SRC_DIR): Tell the compiler where to find header files (.h)
CXXFLAGS = -std=c++17 -Wall -g -I$(SRC_DIR)

# Linker flags:
# Tell the linker to link against the SFML and Box2D libraries
# The order of SFML libraries can be important.
LDFLAGS = -lbox2d -lsfml-graphics -lsfml-window -lsfml-system


# --- File Definitions ---

# Find all .cpp files in the source directory automatically
SRCS = $(wildcard $(SRC_DIR)/*.cpp)

# Generate a list of object files (.o) in the object directory
# This maps 'src/main.cpp' to 'obj/main.o', for example.
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))


# --- Makefile Rules ---

# The 'all' rule is the default goal.
# It depends on the executable, so 'make' or 'make all' will build the project.
all: $(EXEC)

# Rule to link the object files into the final executable.
# It depends on all the object files.
$(EXEC): $(OBJS)
	@echo "Linking..."
	$(CXX) $(OBJS) -o $(EXEC) $(LDFLAGS)
	@echo "Build finished: $(EXEC)"

# Rule to compile a .cpp source file from src/ into a .o object file in obj/
# '$<' is the first prerequisite (the .cpp file).
# '$@' is the target (the .o file).
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@echo "Compiling $<..."
	@mkdir -p $(OBJ_DIR) # Create the object directory if it doesn't exist
	$(CXX) $(CXXFLAGS) -c $< -o $@

# The 'clean' rule removes all generated files.
clean:
	@echo "Cleaning up..."
	rm -rf $(OBJ_DIR) $(EXEC)

# The 'run' rule first builds the project (if needed) and then runs it.
run: all
	./$(EXEC)

# Phony targets are not actual files.
# This prevents 'make' from getting confused if a file named 'all', 'clean', or 'run' exists.
.PHONY: all clean run
