CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -pthread
INCLUDES = -I. -Ilogger -IMCprotocollib

# Source files
SOURCES = main.cpp logger/Logger.cpp MCprotocollib/MCprotocol.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Target executable
TARGET = plc_test

# Default rule
all: $(TARGET)

# Link the executable
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean build files
clean:
	rm -f $(OBJECTS) $(TARGET)
	rm -rf logs/

# Create necessary directories
dirs:
	mkdir -p logger MCprotocollib logs

# Debug build
debug: CXXFLAGS += -DDEBUG -g
debug: $(TARGET)

# Install dependencies (if needed)
install-deps:
	# Add any dependency installation commands here

# Run the program
run: $(TARGET)
	./$(TARGET)

# Run with logging enabled
run-log: $(TARGET)
	./$(TARGET) 2>&1 | tee execution.log

.PHONY: all clean dirs debug install-deps run run-log