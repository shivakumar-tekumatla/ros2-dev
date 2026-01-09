# ROS 2 Humble Docker Setup

This directory contains Docker configuration files for setting up a ROS 2 Humble development environment.

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
- Mount your project directory at `/home/${USER}/ros2-dev`
- Keep the container running in the background

### 3. Launch Interactive Shell

```bash
./bash.sh
```

This will open an interactive bash shell inside the running container. You'll see:
```
${USER}@ros2-humble-container:~/ros2-dev$
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

## Troubleshooting

### Container Won't Start

Check container status:
```bash
docker ps -a
docker logs ros2-humble-container
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