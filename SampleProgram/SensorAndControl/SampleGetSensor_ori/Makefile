CPP = g++
CPPFLAGS = -fPIC -g -pthread
INCLUDES = -I/usr/local/include/zmp/rc110_v2
LIBS = -L/usr/local/lib/zmp/rc110_v2 -lBaseboard -lRcControl -lpthread

TARGET = SampleGetSensor
OBJS = SampleGetSensor.o


all: $(TARGET) 


$(TARGET): $(OBJS)
	$(CPP) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)


clean:
	rm      -rf $(OBJS) $(TARGET)


#libIpm.a: $(OBJS)
#	ar crv $@ $(OBJS)
#


.cpp.o: $*.h
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $<



