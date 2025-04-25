# Troubleshooting

Common issues and solutions for setting up or running the Isaac ROS Workspace.

## Docker Permission Denied
**Error:** `cannot open /dev/video0: Permission denied`
**Fix:** Add user to docker group:
```bash
sudo usermod -aG docker $USER
```
Then logout and login again.

## Colcon Build Fails
- Ensure all ROS dependencies are installed via `rosdep`
- Check for `COLCON_IGNORE` in non-ROS folders

## No GPU Detected in Container
- Verify `--gpus all` flag
- Make sure `nvidia-docker2` is installed

## Kubernetes Pod Not Scheduling
- Check `kubectl describe pod <pod-name>` for nodeSelector or taint issues
- Confirm GPU node has correct label and driver installed

