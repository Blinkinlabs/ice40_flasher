# Mount this host directory at /build in the container
SRC_DIR=`pwd`/../
BUILD_DIR=/build

DOCKER_IMAGE_NAME=ice40_flasher_builder

TARGET=../main.elf

COMMON= run \
        --rm \
		--mount type=bind,source=${SRC_DIR},target=${BUILD_DIR} \
		-w ${BUILD_DIR}

all:
	@echo "Build targets: "
	@echo "docker-image         Build an image for building tools with Docker"
	@echo "build                Build the firmware"
	@echo "clean                Clean the firmware build"
	@echo "flash                Build the firmware, then flash using openocd"
	@echo "run                  (for debugging) Run a shell using the above image"
#	@echo "debug-server         Start an OpenOCD debugger, attached to the target"
#	@echo "debug-client         Connect to the debug server using gdb-multiarch"

docker-image:
	docker build -t ${DOCKER_IMAGE_NAME} -f Dockerfile .

${TARGET}:
	docker ${COMMON} \
		-it ${DOCKER_IMAGE_NAME} \
        /usr/bin/bash -c "mkdir -p build; cd build; cmake ../ -DPICO_BOARD=rp2350b_dev; make -j12"

build: ${TARGET}

clean:
	rm ${TARGET}

run:
	docker ${COMMON} \
		--privileged \
		-v /dev/bus/usb:/dev/bus/usb \
		--net=host \
		-it ${DOCKER_IMAGE_NAME} \
		/usr/bin/bash

flash: ${TARGET}
	docker ${COMMON} \
		--privileged \
		-v /dev/bus/usb:/dev/bus/usb \
		-it ${DOCKER_IMAGE_NAME} \
		/usr/local/bin/openocd \
			-f interface/cmsis-dap.cfg \
			-f target/rp2350.cfg \
			-c "adapter speed 5000" \
			-c "program build/main.elf verify ; reset ; exit"

# This should work without --net=host ?
debug-server:
	docker ${COMMON} \
		--privileged -v /dev/bus/usb:/dev/bus/usb \
		--net=host \
		-it ${DOCKER_IMAGE_NAME} \
		/usr/local/bin/openocd \
			-f interface/cmsis-dap.cfg \
			-f /build/rp2350.cfg \
			-c "adapter speed 5000"

#			-c "set USE_CORE 0" \
#			-f target/rp2350.cfg \

debug-client:
	docker ${COMMON} \
		--mount type=bind,source=`pwd`/../pico-bootrom-rp2350,target=/home/graham/dev/amy/amy-bootrom/ \
		--net=host \
		-it ${DOCKER_IMAGE_NAME} \
		/usr//bin/gdb-multiarch \
		--se=main.elf \
		-ex "target extended-remote localhost:3333"
