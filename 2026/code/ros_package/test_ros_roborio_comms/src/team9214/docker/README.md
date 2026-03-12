# Dockerfile

## Building ARM image on x86_64 host os

### Enable Docker Builds

```bash
docker buildx create --use --name multi-platform-builder
docker buildx inspect --bootstrap
```

### Build the image with the correct platform

```bash
docker buildx build --platform linux/arm64 -t your-image-name -f docker/Dockerfile
```

