PLATFORM=aarch64

all:
	gcc gpio_example.c -lgpiod -o gpio_example
	gcc timer_example.c -o timer_example -lrt
	g++ camera_trigger.cpp  -o camera_trigger -I/opt/MVS/include -Wl,-rpath=$(MVCAM_COMMON_RUNENV)/$(PLATFORM) -L$(MVCAM_COMMON_RUNENV)/$(PLATFORM) -lMvCameraControl -lpthread

clean:
	rm gpio_example timer_example camera_trigger
