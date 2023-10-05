# Kinova-arm

Code created for various manipulation tasks using the 6 Degree of Freedom Kinova Gen3.
Maintained by the Özay group at the University of Michigan.

This code is based upon the work done by Kwesi Rutledge in the repository
https://github.com/kwesiRutledge/OzayGroupExploration.git
To pull changes from this repository, you can add it as remote location with
    
    git remote add upstream https://github.com/kwesiRutledge/OzayGroupExploration.git

Then when you want to pull changes:

    git pull upstream main
    
If unwanted files are added when merging with upstream, then you will have to remove them. This workflow is flawed, but will work for now.


## Getting Started

There are two possible methods for getting started with `kinova-arm`.
1. Docker
2. Local Install

The Docker method for getting started appears to be more robust (won't break as often), but
it is not necessary if you are working on the lab laptop.


### Docker

In this section we will discuss how to: build the Drake-Kinova Docker Image and run a container with it.

In order to control the 6 Degree of Freedom Kinova Gen3 in the Özay group, [Drake](https://drake.mit.edu/) is used along with the [Kinova Kortex API](https://github.com/Kinovarobotics/kortex) and [kinova_drake](https://github.com/vincekurtz/kinova_drake) (a library built by Vince Kurtz).

If you are interested in getting set up with all of the software that you need to control our robot, do the following:
1. Pull this git repository.
2. From the repository's main directory, run a shell script to create the docker image: `./shell-scripts/create-drake-v2-docker-image.sh`.
3. Start the docker container: `./shell-scripts/run-drake-v2-docker-container.sh`


### Local Install

1. Create the project folder (e.g., `~/kinova`)
2. Create a virtual environment for the project. (e.g., `python -m venv kinova-venv`)
3. Into your project folder clone both `kinova-arm` and `kinova-drake`.
    1. The currently recommended version of `kinova-drake`:
        https://github.com/ozay-group/kinova-arm
    2. The currently recommended version of `kinova-arm`:
        https://github.com/kwesiRutledge/kinova_drake
        
4. Locally Install `kinova-arm`
    1. Make sure your virtual environment is activated.
    2. Enter the `kinova-arm` directory.
    3. Run the following commands:
        
        ```python
        pip install --upgrade pip
        pip install -r install/requirements.txt
        pip install -e .
        ```
        
    4. This will install `kinova-arm` into the virtual environment. (This tells your computer where `kinova-arm` is whenever you run `import kinova_arm`.)
    5. Example for how to import `kinova-arm` is shown here (note that this file does not work without installing `kinova_drake` first):
        https://github.com/ozay-group/kinova-arm/blob/0435f84263984a8dc83ea478042248593882f281/scripts/debug/velocity_control1/velocity_command_test.py#L34
        

5. Locally Install `kinova_drake`
    1. Make sure your virtual environment is activated.
    2. Enter the `kinova-arm` directory.
    3. Run the following commands:
        
        ```python
        pip install -r requirements.txt
        pip install -e .
        ```
        
    4. This will install `kinova_drake` into the environment. (This tells your computer where `kinova_drake` is whenever you run `import kinova_drake`.)
    5. An example of how to import this is listed above. (4e)

6. Install `kortex_api`
    1. Install `kortex_api 2.6.0` which is compatible with state-of-art `kinova_drake` (instead of 3.2.0). .whl file can be found in the link below
        https://github.com/Kinovarobotics/kortex.git
        
        [JFrog](https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-public/kortex)
        
    2. The 2.6.0 version of `kortex_api` includes dependency on `protobuf==3.5.1`, which is not compatible with python 3.10+. Hence, force install `protobuf==3.19.4` after installing `kortex_api 2.6.0`

7. Install Intel RealSense SDK 2.0
    1. Follow the instruction to install at the link below:   
        https://github.com/IntelRealSense/librealsense


### Developing Code for Kinova

Make sure that the container named drake-container is running, use an editor like VS Code to begin developing.

In VS Code, you can attach your application to the running container, giving you access to all of the libraries installed in the container after you built it.

## FAQ

### `pip install -r requirements.txt` doesn't work and I'm installing on Mac OS X with an M-series chip and some parts fail.

We are unsure about why this happens.
To complete installation without some of the vision libraries that are causing the issue, comment out
the following lines in requirements.txt:
```
open3d          # Point Cloud
pyrealsense2    # RealSense
dt_apriltags    # Apriltags
```

## Additional Info

- [Hardware Instructions](./doc/kinova-hardware-instructions.md)