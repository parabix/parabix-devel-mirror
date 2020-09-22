FROM silkeh/clang:9

USER root

RUN apt-get update && \
    apt-get -y install build-essential protobuf-compiler libblkid-dev \
    e2fslibs-dev libaudit-dev libncurses5-dev python

WORKDIR /home

RUN wget https://dl.bintray.com/boostorg/release/1.68.0/source/boost_1_68_0.tar.gz \
    && tar -zxvf boost_1_68_0.tar.gz \
    && cd boost_1_68_0/ && ./bootstrap.sh && ./b2

