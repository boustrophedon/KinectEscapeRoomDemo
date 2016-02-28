FREENECTDIR=/home/hcs/code/hackhers2016/libfreenect2
CHILITAGSDIR=/home/hcs/code/hackhers2016/chilitags

CFLAGS = -std=c++11
CFLAGS += -DLIBFREENECT2_WITH_OPENGL_SUPPORT -I$(FREENECTDIR)/include/
CFLAGS += -I$(CHILITAGSDIR)/include/

LDFLAGS = -L$(FREENECTDIR)/lib/ -lfreenect2 -Wl,-rpath=$(FREENECTDIR)/lib/
LDFLAGS += -lopencv_core -lopencv_highgui -lopencv_imgproc
LDFLAGS += -L$(CHILITAGSDIR)/build/src/ -lchilitags -Wl,-rpath=$(CHILITAGSDIR)/build/src/
all:
	g++ -o detectchili $(CFLAGS) DetectChili.cpp $(LDFLAGS)
