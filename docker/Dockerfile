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

RUN git clone https://github.com/robotis-git/robotis_dds_python.git && \
    cd robotis_dds_python && \
    git checkout feature-hee && \
    pip3 install -e .

WORKDIR /root

RUN git clone isaac groot url

RUN 빌드 빌드

# --- robotis_dds_python을 Python이 항상 인식하도록 .pth 생성 ---
# 컨테이너 실행할 때 /workspace/robotis_dds_python 이 mount 되어 있으면 자동 인식됨
# RUN echo "/workspace/robotis_dds_python" > /usr/local/lib/python3.10/dist-packages/robotis_dds_python.pth

# --- 기본 CMD ---
CMD ["bash"]
