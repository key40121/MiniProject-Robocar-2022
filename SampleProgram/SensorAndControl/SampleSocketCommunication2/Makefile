CPP = g++
CPPFLAGS = -g -pthread
INCLUDES = -I/usr/local/include/zmp/rc110_v2 -I/usr/local/include/urg_c -I/usr/local/include/urg_cpp
LIBS = -L/usr/lib -L/usr/local/lib/zmp/rc110_v2 -L/usr/local/lib/c_urg -lBaseboard -lRcControl -lRczCommunication -lurg_cpp -lpthread -lxml2 -lLaserRangeSensor -lurg_cpp

TARGET = SampleSocketCommunication
OBJS = SampleSocketCommunication.o ServerApp.o


all: $(TARGET) 



$(TARGET): $(OBJS)
	$(CPP) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)


clean:
	rm -rf $(OBJS) $(TARGET)


.cpp.o: $*.h
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $<



