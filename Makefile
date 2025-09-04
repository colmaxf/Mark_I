CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -pthread
INCLUDES = -I. -Ilogger -IMCprotocollib -ILidarlib

# Thêm các thư viện Boost cần thiết
LIBS = -lboost_thread -lboost_system -lpthread

# Source files
SOURCES = main.cpp \
          logger/Logger.cpp \
          MCprotocollib/MCprotocol.cpp \
          Lidarlib/Lidarlib.cpp \
          Lidarlib/Data_SDK/LakiBeamUDP.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Target executable
TARGET = control_system

# Default rule
all: $(TARGET)

# Link the executable - THÊM $(LIBS) vào đây
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean build files
clean:
	rm -f $(OBJECTS) $(TARGET)
	rm -rf logs/

# Run the program
run: $(TARGET)
	./$(TARGET)


.PHONY: all clean run 