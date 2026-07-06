from frc_apriltags import CameraViewer


if __name__ == "__main__":
    viewer = CameraViewer()
    viewer.run(host="127.0.0.1", port=8000)
