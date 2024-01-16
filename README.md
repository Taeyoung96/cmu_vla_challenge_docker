# CMU VLA challenge  

This repository is heavily adapted from [jizhang-cmu/cmu_vla_challenge_unity](https://github.com/jizhang-cmu/cmu_vla_challenge_unity).  

For detailed repository setup, see [jizhang-cmu/cmu_vla_challenge_unity](https://github.com/jizhang-cmu/cmu_vla_challenge_unity).

## Make Docker environment  

**1. Enter the `/docker` folder and make a docker image.**
```
git clone https://github.com/Taeyoung96/cmu_vla_challenge_docker.git
```
```
cd cmu_vla_challenge_docker/docker
```
```
docker build -t cmu_vla .
```

**2. Make docker container (same path as above)**

In `/docker`,  
```
sudo chmod -R 777 container_run.sh
```
```
./container_run.sh <container_name> <image_name:tag>
```
**:warning: You should change {container_name}, {docker image} to suit your environment.**  

```
./container_run.sh cmu_vla_container cmu_vla:latest 
```

If you have successfully created the docker container, the terminal output will be similar to the below.
```
================VLA Unity Docker Env Ready================
root@taeyoung-cilab:~/catkin_ws#
```

**3. Build and run cmu_vla_challenge**

Inside the docker container, build and run the package.  
```
catkin_make
```
```
source devel/setup.bash
```

```
cd src/cmu_vla_challenge_unity
```

```
./system_bring_up.sh
```

## Make bag file with namespace

**1. Record the bag file.**

```
rosbag record -o robot1 /camera/image /camera/semantic_image /semantic_scan /tf /tf_static
```

**2. Using script to add namespace to bag file**  

In another terminal, turn on `roscore` and run `python bag/rosbag_namespace.py`

