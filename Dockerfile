FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
	cmake build-essential pkg-config libpython3-dev python3-numpy libicu-dev \
	ninja-build libboost-all-dev git vim libtbb-dev 
RUN apt-get install -y python3-pyparsing python3-pip
RUN apt-get -y install libatlas-base-dev libsuitesparse-dev libmetis5 libmetis-dev
RUN apt-get -y install python3-pyqt5 