# Build a docker image for developing on RP2040/2350
#
# Included in the image is:
# * picosdk in /pico/pico-sdk
# * picotool in /usr/local/
# * openocd (rpi pico version) in /usr/local/
# * pico examples in /pico/pico-examples

FROM docker.io/library/debian:13 AS base

RUN apt-get -qq update -y

# Nice to have
RUN DEBIAN_FRONTEND=noninteractive \
      apt-get install -y --no-install-recommends \
            vim \
            tmux \
            wget \
            ca-certificates \
            make \
            usbutils \
            gdb-multiarch \
            git \
            cmake \
            gcc-arm-none-eabi \
            libnewlib-arm-none-eabi \
            build-essential \
            libstdc++-arm-none-eabi-newlib \
            libusb-1.0-0-dev \
            python3 \
            automake \
            autoconf \
            texinfo \
            libtool \
            libftdi-dev \
            pkg-config \
	    libjim-dev

WORKDIR /pico
RUN git clone https://github.com/raspberrypi/pico-sdk.git --branch 2.2.0
WORKDIR /pico/pico-sdk
RUN git submodule update --init

ENV PICO_SDK_PATH=/pico/pico-sdk

WORKDIR /pico
RUN git clone https://github.com/raspberrypi/picotool.git --branch 2.2.0
WORKDIR /pico/picotool/build
RUN cmake ../
RUN make && make install

WORKDIR /pico
RUN git clone https://github.com/raspberrypi/openocd.git --depth=1 --branch sdk-2.2.0
WORKDIR /pico/openocd
RUN ./bootstrap && \
    ./configure --prefix=/usr/local && \
    make -j12 && \
    make install

ENV PATH="$PATH:/usr/local/bin"

WORKDIR /pico
RUN git clone https://github.com/raspberrypi/pico-examples.git --branch sdk-2.2.0

