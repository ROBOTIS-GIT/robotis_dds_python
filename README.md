# robotis_dds_python
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![CycloneDDS](https://img.shields.io/badge/CycloneDDS-0.10.x-green.svg)](https://github.com/eclipse-cyclonedds/cyclonedds)

A Python SDK for publishing and subscribing to topics using CycloneDDS without importing rclpy. This library provides a lightweight alternative to ROS 2 Python API while maintaining compatibility with ROS 2 message types.

## Features

- **Lightweight**: No ROS 2 runtime dependencies
- **Compatible**: Works with standard ROS 2 message types
- **Easy to use**: Simple publisher/subscriber API
- **Extensible**: Support for custom message types

## Installation

### Prerequisites

- Python 3.8 or higher
- CycloneDDS 0.10.2

### Install from source

1. Before using this package, you need to have CycloneDDS installed on your system. You can build it from source as follows:

```bash
# Install build dependencies
sudo apt update
sudo apt install -y git build-essential cmake libssl-dev

# Clone CycloneDDS source
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds
git checkout 0.10.2

# Create a build directory
mkdir build && cd build

# Build and install CycloneDDS
cmake -DCMAKE_INSTALL_PREFIX=$HOME/cyclonedds/install -DBUILD_EXAMPLES=ON ..
cmake --build .
cmake --install .
```
* Note: You can change $HOME/cyclonedds/install to any directory you prefer.

2. After installation, make sure to set the environment variables so that Python can locate the CycloneDDS libraries:
```bash
# For bash
echo 'export CYCLONEDDS_HOME=$HOME/cyclonedds/install' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PATH=$CYCLONEDDS_HOME/bin:$PATH' >> ~/.bashrc

source ~/.bashrc

# For zsh
echo 'export CYCLONEDDS_HOME=$HOME/cyclonedds/install' >> ~/.zshrc
echo 'export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH' >> ~/.zshrc
echo 'export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH' >> ~/.zshrc
echo 'export PATH=$CYCLONEDDS_HOME/bin:$PATH' >> ~/.zshrc

source ~/.zshrc
```

Finally, install robotis_dds_python:

```bash
# Clone the repository
git clone https://github.com/robotis-git/robotis_dds_python.git

# Install the package
cd robotis_dds_python
pip install -e .
```

## Examples

Check the [`example/`](example/) directory for complete working examples:

- [`trajectory_publisher.py`](example/trajectory_publisher.py) - Publishing joint trajectory messages
- [`trajectory_subscriber.py`](example/trajectory_subscriber.py) - Subscribing to joint trajectory messages

## Creating Custom Messages

To add support for new ROS 2 message types:

1. Navigate to the IDL directory:
   ```bash
   cd robotis_dds_python/robotis_dds_python/idl
   ```

2. Generate Python bindings from IDL:
   ```bash
   idlc -l py -I /opt/ros/jazzy/share/ /opt/ros/jazzy/share/{your_package}/msg/{YourMessage}.idl
   ```

3. Update the generated files and `__init__.py` as needed.

## API Reference

### topic_manager

#### topic_writer(topic_name, topic_type, qos)

Creates a DDS DataWriter for publishing messages.

**Parameters:**
- `topic_name` (str): Name of the topic
- `topic_type` (Type): Message type class
- `qos` (Qos, optional): Quality of Service settings

**Returns:** DataWriter instance

#### topic_reader(domain_id, topic_name, topic_type, qos)

Creates a DDS DataReader for subscribing to messages.

**Parameters:**
- `topic_name` (str): Name of the topic
- `topic_type` (Type): Message type class
- `qos` (Qos, optional): Quality of Service settings

**Returns:** DataReader instance

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
