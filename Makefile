CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -pthread -march=armv8-a+crc+crypto -mtune=cortex-a76
INCLUDES = -I. -Ilogger -IMCprotocollib -ILidarlib -IServerlib -IBatterylib -IIMUlib
LIBS = -lboost_thread -lboost_system -lpthread -ldl -lz -ldlt

SOURCES = main.cpp \
          SystemManager.cpp \
          logger/Logger.cpp \
          MCprotocollib/MCprotocol.cpp \
          Lidarlib/Lidarlib.cpp \
          Lidarlib/Data_SDK/LakiBeamUDP.cpp \
          Serverlib/servercommunicator.cpp \
          Batterylib/BatteryJBD.cpp \
          IMUlib/MPU9250_AGV.cpp

OBJECTS = $(SOURCES:.cpp=.o)
TARGET = control_system

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)
	@chmod 777 $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)
	rm -rf logs/

run: $(TARGET)
	./$(TARGET)

main.o: config.h SystemManager.h
SystemManager.o: SystemManager.h config.h
logger/Logger.o: config.h
MCprotocollib/MCprotocol.o: config.h
Lidarlib/Lidarlib.o: config.h
Serverlib/servercommunicator.o: config.h
Batterylib/BatteryJBD.o: config.h
IMUlib/MPU9250_AGV.o: config.h

.PHONY: all clean run