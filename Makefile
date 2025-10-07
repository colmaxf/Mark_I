CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -pthread -march=armv8-a+crc+crypto -mtune=cortex-a76 -flto -ffast-math
INCLUDES = -I. -Ilogger -IMCprotocollib -ILidarlib -IServerlib -IBatterylib -I/usr/include/eigen3 -I/usr/include/lua5.3 -I~/cartographer_ws/install/include -I/usr/local/include  # Thêm -I/usr/local/include cuối

LIBS = -Wl,--start-group \
       -lboost_thread \
       -lboost_system \
       -lboost_iostreams \
       -lboost_filesystem \
       -lpthread \
       -lprotobuf \
       -lglog \
       -lgflags \
       -llua5.3 \
       -lceres \
       -lcholmod \
       -lamd \
       -lcamd \
       -lccolamd \
       -lcolamd \
       -lspqr \
       -lcairo \
       -ldl \
       -lz \
       -Wl,--end-group \
       -Wl,--whole-archive -lcartographer -Wl,--no-whole-archive \
       -Wl,--start-group \
       -labsl_bad_optional_access \
       -labsl_strings \
       -labsl_str_format_internal \
       -labsl_throw_delegate \
       -labsl_synchronization \
       -labsl_time \
       -labsl_base \
       -labsl_hash \
       -labsl_raw_hash_set \
       -labsl_raw_logging_internal \
       -labsl_stacktrace \
       -labsl_symbolize \
       -labsl_malloc_internal \
       -labsl_debugging_internal \
       -labsl_demangle_internal \
       -labsl_flags_parse \
       -labsl_flags_usage \
       -labsl_flags_config \
       -labsl_flags_internal \
       -labsl_flags_marshalling \
       -labsl_flags_program_name \
       -labsl_int128 \
       -labsl_spinlock_wait \
       -labsl_civil_time \
       -labsl_time_zone \
       -labsl_status \
       -labsl_statusor \
       -labsl_cord \
       -labsl_cord_internal \
       -labsl_cordz_functions \
       -labsl_cordz_handle \
       -labsl_cordz_info \
       -labsl_strings_internal \
       -labsl_graphcycles_internal \
       -labsl_exponential_biased \
       -Wl,--end-group \
       -ldlt


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