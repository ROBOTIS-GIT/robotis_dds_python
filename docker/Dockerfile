FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libssl-dev \
    python3 \
    python3-pip \
    python3-dev \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

# cyclonedds
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds.git && \
    cd cyclonedds && \
    git checkout 0.10.2 && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/root/cyclonedds/install -DBUILD_EXAMPLES=ON .. && \
    cmake --build . && \
    cmake --install .

RUN echo 'export CYCLONEDDS_HOME=/root/cyclonedds/install' >> ~/.bashrc && \
    echo 'export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH' >> ~/.bashrc && \
    echo 'export PATH=$CYCLONEDDS_HOME/bin:$PATH' >> ~/.bashrc

WORKDIR /root

# robotis_dds_python

RUN git clone https://github.com/robotis-git/robotis_dds_python.git && \
    cd robotis_dds_python && \
    git checkout feature-add-service-and-callback && \
    pip3 install -e .

WORKDIR /root

# physical_ai_tools & Isaac-GR00T
RUN git clone https://github.com/ROBOTIS-GIT/physical_ai_tools.git && \
    cd physical_ai_tools && \
    git checkout feature-dds-python && \
    cd Isaac-GR00T && \
    pip3 install -e .

# --- 기본 CMD ---
CMD ["bash"]
