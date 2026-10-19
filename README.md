# recruiting-sw-control Setup

This file contains the steps to follow to setup the environment.

## 1. Prerequisites

Install the following software:
* **WSL2 with Ubuntu** - _Required on Windows_.
* **Docker Desktop** - Enable **WSL integration** for Ubuntu in Docker Desktop settings.
* **Visual Studio Code**
* **Git** - _available inside WSL Ubuntu environment_.

Install the following VS Code extensions:
* **WSL**
* **Dev Containers**
* **C/C++**

> The repository includes its own `.devcontainer` configuration file that defines the development container.

Make sure Docker Desktop is running before continuing

## 2. Open VS Code in WSL

Open VS Code, press `Ctrl + Shift + P` and run `WSL: Connect to WSL` and select your **Ubuntu** distribution.

## 3. Clone the repo

Now open a terminal in VS Code and clone the repository from **GitHub** by using directly cmd as following:
```sh
mkdir -p ~/<NAME OF YOUR FOLDER>

cd ~/<NAME OF YOUR FOLDER>
git clone [https://github.com/FApew/recruiting-sw-control](https://github.com/FApew/recruiting-sw-control)
cd recruiting-sw-control
```
> Alternatively, use VS Code integrated feature to clone the repository and enter its URL.

## 4. Open the Dev Container

Once the repository is successfully cloned, press `Ctrl + Shift + P` and run `Dev Containers: Reopen in Container` to open the project inside Docker Container in VS Code.

Wait for Docker to build the image and start the container. The first build may take a few minutes.

> If the command fails, be sure to have Docker Desktop running and Dev Containers extension installed in VS Code.

## 5. Configure vCAN Interface

The simulator communicates with the control software through a virtual **CAN interface** called *`vcan0`*.

The Dev Container uses host networking and has the **`NET_ADMIN`** capability, so the interface can be created from inside the container.

Run the following commands in the VS Code terminal inside the container:

```
sudo modprobe vcan

if ! ip link show vcan0 >/dev/null 2>&1; then
    sudo ip link add dev vcan0 type vcan
fi

sudo ip link set up vcan0
```

To verify that the interface is available, run:

```sh
ip link show vcan0
```

If `vcan0` already exists, the commands will reuse it instead of trying to create it again.

> The vCAN interface does not necessarily survive a WSL restart. If you restart WSL and get `No such device` errors, run the commands again. If `modprobe vcan` fails, check whether the WSL kernel supports the vCAN module.

## 6. Run the Simulation

To build the workspace and launch the simulator, RViz, and your control nodes, run the following command from the repository root inside the Dev Container:

```sh
./run.sh
```

The script builds the workspace using `colcon build --symlink-install` and then launches `eagle_control` `control.launch.py`. This launch file starts the five simulator nodes, RViz, `centerlineNode`, and `controlNode`.

To stop the simulation safely, press `Ctrl + C` in the terminal.

> Make sure `vcan0` is available before starting the simulation.