# AEROTRAIN SUMMER SCHOOL - HRI day

## Preparing the environment
**Before the start of the workshop, the student should have proper setup the laptop with the following:**
1. Docker
    - Install Docker on Ubuntu: [Docker installation guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
    - In case the laptop has an Nvidia GPU, it's nice to have as well installed Nvidia Docker: [Nvidia container toolkit installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
    - Check docker is properly setup on laptop: open a terminal and run`docker run hello-world`
2. Foxglove
    - [Download Foxglove](https://foxglove.dev/download)
3. Terminator
    - `sudo apt-get install terminator`

## Download support files 
For convenience, a script file is provided that handles downloading the necessary data and Docker images, placing them in the correct location on your laptop. The script uses [gdown](https://github.com/wkentaro/gdown) for automatic file download.
```
./download.sh
```

## Plan for the day

- Introduction to summer school 
- Camera theory
- Target detection
- [ROS Tutorial](ros_tutorial/README.md)


## Troubleshooting

Consider opening an Issue if you have [troubles](https://github.com/AERO-TRAIN/summer_school_perception_day/issues) with the exercises of the repo.\
Feel free to use the [Discussions](https://github.com/AERO-TRAIN/summer_school_perception_day/discussions) tab to exchange ideas and ask questions.