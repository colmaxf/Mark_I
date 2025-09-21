CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -pthread 
INCLUDES = -I. -Ilogger -IMCprotocollib -ILidarlib -IServerlib -IBatterylib


# Thêm các thư viện Boost cần thiết
LIBS = -lboost_thread -lboost_system -lpthread -ldlt

# Source files
SOURCES = main.cpp \
          logger/Logger.cpp \
          MCprotocollib/MCprotocol.cpp \
          Lidarlib/Lidarlib.cpp \
          Lidarlib/Data_SDK/LakiBeamUDP.cpp \
          Serverlib/servercommunicator.cpp \
		  Batterylib/BatteryJBD.cpp \

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
	
main.o: config.h
logger/Logger.o: config.h
MCprotocollib/MCprotocol.o: config.h
Lidarlib/Lidarlib.o: config.h
Serverlib/servercommunicator.o: config.h
Batterylib/BatteryJBD.o: config.h

.PHONY: all clean run 
