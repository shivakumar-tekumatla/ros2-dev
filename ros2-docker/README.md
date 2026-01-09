# ROS 2 Humble Docker Setup

This directory contains Docker configuration files for setting up a ROS 2 Humble development environment.

## What It Does

This Docker setup creates a containerized ROS 2 Humble environment with:
- Official ROS 2 Humble base image
- Essential development tools (build-essential, cmake, git, etc.)
- Python 3 support with pip
- Colored bash prompt for better visibility
- Automatic mounting of your project directory (`/home/shiva/ros2-dev`)
- Non-root `ros` user for safe development
- Pre-configured ROS 2 environment in bashrc

## Files

- **Dockerfile** - Defines the Docker image with ROS 2 Humble and dependencies
- **build.sh** - Builds the Docker image
- **start.sh** - Starts/restarts the Docker container
- **bash.sh** - Launches an interactive bash shell in the running container
- **docker-options** - Configuration file for customizing image name, container name, and Docker options

## Quick Start

### 1. Build the Docker Image

```bash
cd ros2-docker
./build.sh
```

This will create a Docker image named `ros2-humble:latest` using your current username.

### 2. Start the Container

```bash
./start.sh
```

This will:
- Remove any existing container with the same name
- Create a new container named `ros2-humble-container`
- Mount your project directory at `/home/ros/ros2-dev`
- Keep the container running in the background

### 3. Launch Interactive Shell

```bash
./bash.sh
```

This will open an interactive bash shell inside the running container. You'll see:
```
ros@ros2-humble-container:~/ros2-dev$
```

## Usage Examples

### Building ROS 2 Packages

Inside the container:

```bash
cd ~/ros2-dev
mkdir -p my_ws/src
cd my_ws
colcon build
```

### Running ROS 2 Commands

```bash
source /opt/ros/humble/setup.bash
ros2 --version
ros2 run <package> <executable>
```

### Installing Additional Packages

Inside the container, use apt:

```bash
sudo apt-get update
sudo apt-get install ros-humble-<package-name>
```

## Customization

### Changing Container Name or Image Name

Edit `docker-options` and modify:
```bash
CONTAINER_NAME="your-container-name"
IMAGE_NAME="your-image-name"
```

Then rebuild:
```bash
./build.sh
./start.sh
```

### Adding Docker Options

Add custom Docker run options in `docker-options`:
```bash
RUN_OPTIONS="--publish 8080:8080 -e MY_VAR=value"
```

Then restart the container:
```bash
./start.sh
./bash.sh
```

## Troubleshooting

### Container Won't Start

Check container status:
```bash
docker ps -a
docker logs ros2-humble-container
```

### Files Not Visible in Container

Verify the mount:
```bash
docker inspect ros2-humble-container | grep -A 10 "Mounts"
```

Files should be mounted at `/home/ros/ros2-dev` inside the container.

### Permission Issues

The container runs as the `ros` user. If you encounter permission issues with mounted files, ensure the host files have appropriate permissions:
```bash
chmod 755 ~/ros2-dev
```

## Stopping and Removing

### Stop the Container

```bash
docker stop ros2-humble-container
```

### Remove the Container

```bash
docker rm ros2-humble-container
```

### Remove the Image

```bash
docker rmi ros2-humble:latest
```

## Environment Variables

The following environment variables are set in the container:
- `ROS_DISTRO=humble`
- `LANG=C.UTF-8`
- `LC_ALL=C.UTF-8`

## References

- [ROS 2 Documentation](https://docs.ros.org/)
- [Docker Documentation](https://docs.docker.com/)
- [ROS 2 Humble Release](https://docs.ros.org/en/humble/)
