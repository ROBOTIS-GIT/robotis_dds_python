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
- CycloneDDS 0.10.x

### Install from source

```bash
# Clone the repository
git clone https://github.com/robotis-git/robotis_dds_python.git
cd robotis_dds_python

# Install the package
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
   idlc -l py -I /opt/ros/jazzy/share/ /opt/ros/jazzy/share/your_package/msg/{YourMessage}.idl
   ```

3. Update the generated files and `__init__.py` as needed.

## API Reference

### topic_writer(domain_id, topic_name, topic_type, qos=None)

Creates a DDS DataWriter for publishing messages.

**Parameters:**
- `domain_id` (int): DDS domain ID (default: 0)
- `topic_name` (str): Name of the topic
- `topic_type` (Type): Message type class
- `qos` (Qos, optional): Quality of Service settings

**Returns:** DataWriter instance

### topic_reader(domain_id, topic_name, topic_type, qos=None)

Creates a DDS DataReader for subscribing to messages.

**Parameters:**
- `domain_id` (int): DDS domain ID (default: 0)  
- `topic_name` (str): Name of the topic
- `topic_type` (Type): Message type class
- `qos` (Qos, optional): Quality of Service settings

**Returns:** DataReader instance

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
