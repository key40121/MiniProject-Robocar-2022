CPP = g++
CPPFLAGS = -fPIC -g
INCLUDES = -I/usr/local/include/zmp/rc110_v2
LIBS = -L/usr/local/lib/zmp/rc110_v2 -lRcControl

TARGET = my_app
OBJS = main.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CPP) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)
clean:
	rm -rf $(OBJS) $(TARGET)

.cpp.o: $*.h
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $<
