FROM ponylang/ponyc-ci:llvm-5.0.2

USER root

RUN apt-get update && apt-get -y install software-properties-common

RUN add-apt-repository ppa:mhier/libboost-latest

RUN apt-get update && \
    apt-get -y install cmake protobuf-compiler libblkid-dev \
    e2fslibs-dev libaudit-dev libboost1.68-dev

