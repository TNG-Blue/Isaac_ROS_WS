# Kubernetes Orchestration

Deploy Isaac ROS components across multiple Jetson devices using Kubernetes for scalable GPU-based robotics.

## Components
- `Deployment` for each Jetson device
- `Service` using `LoadBalancer` with MetalLB
- `ConfigMap` for ROS 2 environment vars

## Example
```bash
kubectl apply -f k8s/configmap.yaml
kubectl apply -f k8s/deployment-jetson.yaml
kubectl apply -f k8s/service.yaml
```

## GPU Scheduling
Use `nvidia.com/gpu` resource limits with `nodeSelector` for GPU-aware pods.

---
