
Official ROS2 package implementation for "A Wearable Real-Time 2D/3D Eye-Gaze Interface to Realize Robot Assistance for Quadriplegics"

## Installation
First, clone this repo and make sure the docker engine is installed and you are inside the `/docker` directory. Also, don't forget to enable display by:

```
cd docker && xhost +
```

You can pull our image from dockerhub by:

```
docker pull savicktso/gaze_estimation_3d:latest
```
This image is about 20 GB large.

We built services for quick start, You may run services separately, like:
```
docker compose run --rm tobii_stream
```

to start the tobii glasses node with image and gaze point publishers.

Beside other node-services like `zedm_init`, `rviz2`, we have set the `all-in-one` service for one-line launch:

```
docker compose run --rm all_in_one
```

You may display the processed images and estimation results by: (this is going to decrease the system frequency.)

```
docker compose run --rm all_in_one_display
```

