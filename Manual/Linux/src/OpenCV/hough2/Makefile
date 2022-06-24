CPP = g++
CPPFLAGS = -fPIC -g
INCLUDES = -I/usr/local/include/opencv2 -I/usr/local/include/opencv
LIBS = -L/usr/local/lib/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio

TARGET = sample 
OBJS = sample.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CPP) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)
clean:
	rm -rf $(OBJS) $(TARGET)

.cpp.o: $*.h
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c $<
