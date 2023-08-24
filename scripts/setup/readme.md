# Setup

This set of scripts is used to setup the computer for various different types of use cases of `kinova-arm`.

## Docker

If you are interested in using Docker, then you should do the following to
(i) create the Docker image and (ii) to run the container.

### Creating the docker image

Enter the `shell-scripts/create-drake-image` directory and create the
version of the image that you're interested in. For example, to run version
4, run:

```shell
source v4.sh
```

Note: If you are on a Windows machine then you should only use the batch files
with the `.bat` extension.

### Runner the docker container
Enter the `shell-scripts/run-drake-container` directory and run
the command for starting the container. For example, to run version
4 run:
```shell
v4.sh
```

Note: If you are on a Windows machine then you should only use the batch files
with the `.bat` extension.