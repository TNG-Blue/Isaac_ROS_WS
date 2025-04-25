# Docker Support

The Isaac ROS Workspace uses Docker for isolation, consistency, and GPU acceleration.

## Build the Docker Image
```bash
docker build -t isaac_ros_ws:latest -f docker/Dockerfile .
```

## Run a Container
```bash
docker run --rm -it \
  --gpus all \
  --net host \
  --privileged \
  -v /dev:/dev \
  -v $(pwd):/workspaces/isaac_ros_ws \
  isaac_ros_ws:latest
```

## Compose File (Optional)
Use `docker-compose.yml` for managing multi-container setups (Jetson + PC).

---