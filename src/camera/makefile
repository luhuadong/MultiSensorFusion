PLATFORM=aarch64
INCDIR=-I/opt/MVS/include
LIBDIR=-L$(MVCAM_COMMON_RUNENV)/$(PLATFORM)
LINKFLAGS= -Wl,-rpath=$(MVCAM_COMMON_RUNENV)/$(PLATFORM)
LIBS=-lMvCameraControl -lpthread -lrt -lgpiod

all:
	g++ CameraRecorder.cpp -o CameraRecorder $(INCDIR) $(LINKFLAGS) $(LIBDIR) $(LIBS)

clean:
	rm CameraRecorder
