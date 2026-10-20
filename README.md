# recruiting-sw-control Setup

This guide explains how to set up the environment and run the simulation.

## 1. Prerequisites

Install the following software:
* **WSL2 with Ubuntu** - _Required on Windows_.
* **Docker Desktop** - _Enable **WSL integration** for your Ubuntu distribution in Docker Desktop settings_.
* **Visual Studio Code**
* **Git** - _available inside WSL Ubuntu environment_.

Install the following Visual Studio Code extensions:
* **WSL**
* **Dev Containers**
* **C/C++**

> The repository includes a `.devcontainer` configuration that defines the development container and its environment.

> The extension `Comment Anchor` is highly recommended to navigate the code better using anchors

Make sure Docker Desktop is running before continuing

## 2. Open VS Code in WSL

Open VS Code, press `Ctrl + Shift + P` and run `WSL: Connect to WSL` and select your **Ubuntu** distribution.

## 3. Clone the Repository

Now open a terminal in VS Code and clone the repository from **GitHub** by running:
```sh
mkdir -p ~/<NAME OF YOUR FOLDER>
cd ~/<NAME OF YOUR FOLDER>

git clone https://github.com/FApew/recruiting-sw-control
cd recruiting-sw-control
```
> Alternatively, use VS Code integrated feature to clone the repository and enter its URL: 
https://github.com/FApew/recruiting-sw-control

## 4. Build and Open the Dev Container

With the repository open in VS Code, press `Ctrl + Shift + P` to run `Dev Containers: Rebuild and Reopen in Container`.

Wait for Docker to build the image and start the container. The first build may take a few minutes.

> If the command fails, be sure to have Docker Desktop running, WSL integration enabled for Ubuntu and Dev Containers extension installed in VS Code.

## 5. Configure the Virtual CAN Interface

The simulator communicates with the control software through a virtual **CAN interface** called _`vcan0`_.

The Dev Container uses host networking and has the **`NET_ADMIN`** capability, so the interface can be configured directly inside the container.

Open a terminal inside the Dev Container and run:
```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

To verify that the interface is available, run:
```sh
ip link show vcan0
```
> The virtual CAN interface may need to be recreated after a WSL or Docker restart. If the simulator reports that _`vcan0`_ doesn't exist anymore, repeat the previous steps to configure _`vcan0`_.

## 6. Run the Simulation

To build the workspace and launch the simulator, RViz, and your control nodes, run the following command from the repository root inside the Dev Container:
```sh
./run.sh
```
> If `./run.sh` returns _**Permission denied**_, run `chmod +x run.sh` once from the repository root.

The script builds the workspace using `colcon build --symlink-install` and then launches `eagle_control/control.launch.py`.

The launch file starts the simulator, RViz and the control nodes:
1. `centerlineNode` - computes and publishes the centerline from the cones detected.

2. `controlNode` - reads vehicle speed through CAN and calculates the control commands to send to the car.

> Make sure `vcan0` is available before starting the simulation.

To stop the simulation safely, press `Ctrl + C` in the terminal.