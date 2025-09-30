CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -pthread -march=armv8-a+crc+crypto -mtune=cortex-a76 -flto -ffast-math
INCLUDES = -I. -Ilogger -IMCprotocollib -ILidarlib -IServerlib -IBatterylib -I/usr/include/eigen3 -I/usr/include/lua5.3 -I~/cartographer_ws/install/include

# Loại bỏ -labsl_container và -labsl_str_format, giữ các thư viện Abseil khác
LIBS = -lboost_thread -lboost_system -lpthread -ldl -lz -ldlt -lprotobuf -llua5.3 -lceres -lcairo -lcartographer -labsl_time -labsl_base -lglog -labsl_strings -labsl_throw_delegate -labsl_synchronization -labsl_hash

SOURCES = main.cpp \
          SystemManager.cpp \
          logger/Logger.cpp \
          MCprotocollib/MCprotocol.cpp \
          Lidarlib/Lidarlib.cpp \
          Lidarlib/cartographer_standalone.cpp \
          Lidarlib/Data_SDK/LakiBeamUDP.cpp \
          Serverlib/servercommunicator.cpp \
          Batterylib/BatteryJBD.cpp

OBJECTS = $(SOURCES:.cpp=.o)

TARGET = control_system

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -L~/cartographer_ws/install/lib -L/usr/lib/aarch64-linux-gnu -o $@ $^ $(LIBS)
	@chmod 777 $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)
	rm -rf logs/

run: $(TARGET)
	sudo ./$(TARGET)

main.o: config/config.h SystemManager.h
SystemManager.o: SystemManager.h config/config.h
logger/Logger.o: config/config.h
MCprotocollib/MCprotocol.o: config/config.h
Lidarlib/Lidarlib.o: config/config.h
Serverlib/servercommunicator.o: config/config.h
Lidarlib/cartographer_standalone.o: Lidarlib/cartographer_standalone.h
Batterylib/BatteryJBD.o: config/config.h

.PHONY: all clean run