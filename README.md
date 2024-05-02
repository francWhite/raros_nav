![logo](doc/img/logo-banner.png)

# raros_nav
[![licence](https://img.shields.io/github/license/francWhite/raros_nav)](https://github.com/francWhite/raros_nav/blob/main/LICENSE)

This repository contains the navigation stack for the raros robot controller, which was developed as part of a bachelor thesis at the Lucerne University of Applied Sciences and Arts.

## Table of contents
- [Architecture](#architecture)
- [Project Structure](#project-structure)
- [Installation](#installation)
  - [Controller](#controller_install)
  - [Microcontroller](#microcontroller_install)
- [Usage](#usage)
  - [Controller](#microcontroller_usage)
  - [Microcontroller](#microcontroller_usage)
- [Development](#development)
  - [Prerequisites](#prerequisites)
  - [Build](#build)
  - [Run](#run)
- [License](#license)


## Architecture
TODO

## Project Structure
The project is divided into two main components:
- `src/ros2_packages`: ROS2 packages for the master controller (RaspberryPi) containing the navigation stack
- `src/arduino`: Code for the microcontroller (Arduino Zero)

## Installation
<a name="controller_install"></a>
### Controller
All required subsystems for the master controller (RaspberryPi) are available as docker images on [GitHub Container Registry](https://github.com/francWhite?tab=packages&repo_name=raros_nav).
The easiest way to get started is to use the [docker-compose.yml](https://github.com/francWhite/raros_nav/blob/main/docker-compose.yaml)
file in the root directory of this repository.

**Prerequisites**:
- Ubuntu 22.04 Server (other Linux distributions may work as well, but are not tested)
- [docker](https://docs.docker.com/engine/install/) is installed
- `./config` directory exists for the volume mount

**Instructions**:
```shell
cd [path/to/docker-compose.yaml]    # change directory to where the docker-compose.yaml file is located
mkdir config                        # create config directory, only required on first run
docker-compose up -d                # start the containers in detached mode
```

Ensure that the serial port for the communication with the microcontroller is configured correctly in the `docker-compose.yaml` file.
The default is `/dev/ttyACM0`.

<a name="microcontroller_install"></a>
### Microcontroller
The code for the microcontroller (Arduino zero) is available in the [arduino](https://github.com/francWhite/raros_nav/tree/main/src/arduino)
directory of this repository. To build and flash the software onto the microcontroller, clone the repository and use the `flash.sh` script located in root of said directory.

**Prerequisites**:
- [PlatformIO CLI](https://docs.platformio.org/en/stable/core/installation/index.html) is installed
- Arduino is connected to the computer via USB

**Instructions**:
```shell
git clone https://github.com/francWhite/raros_nav.git     # clone the repository
cd raros_nav/src/arduino                                  # navigate to the micro_ros directory
chmod +x flash.sh                                         # make the script executable
./flash.sh                                                # build and flash the software onto the microcontroller
```

## Usage

<a name="controller_usage"></a>
### Controller
Start the docker containers as described in the [installation](#controller_install) section.

TODO describe Rviz2

<a name="microcontroller_usage"></a>
### Microcontroller
The microcontroller is connected to the controller via a serial connection (default is `/dev/ttyACM0`). After flashing the software onto the microcontroller,
the Arduino will automatically connect to the controller. The onboard LED will light up continuously when the connection is established.

## Development

<a name="prerequisites"></a>
### Prerequisites
TODO

<a name="build"></a>
### Build
TODO

<a name="run"></a>
### Run
TODO


## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
