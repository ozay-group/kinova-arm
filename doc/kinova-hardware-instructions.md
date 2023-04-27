# Instructions for Hardware Setup

These instructions should outline how to:
- Turn On the Kinova Gen3 Robot
- Connect to It

## Instructions for Turning On The Robot

1. Check the emergency stop button (red button on top of yellow and black base). Turn/twist it to the right to verify that it has been released.
2. Verify that the robot is plugged in (The power supply should have a blue LED glowing on it).
3. Press and hold the silver power button (on the back of the robot’s base) for three seconds.
4. Watch the start-up light sequence:
    1. Booting = flashing blue
    2. Initialization = steady blue + amber
    3. Ready = steady green

## Connecting to the Robot

The options that we've currently explored for connecting to the robot are:

- Via Kinova Web App (Works on all OS)
- Via XBox Controller
- Via Kinova API (Solution Works on Mac OS X, Linux)
- Via kinova_drake (Solution Works only on Linux)

The last two options provide a way to programmatically control the arm. We will discuss how to use each of these methods in this document.

### Kinova Web App

- Mac OS X Instructions
    1. Go to `Network` in System Preferences.
    2. There should be a connection named `USB 10/100/1000 LAN` which should be connected/active.
    3. Change the properties of the connection.
        1. Change 'Configure IPv4' to be determined 'Manually'
        2. Set IP Address to be `192.168.1.11`.
        3. Choose the subnet `255.255.255.0`.
        4. Press Apply.
    4. Type into the address bar of the browser: `http://192.168.1.10`.
    5. Log in using the admin credentials:
        1. User: `admin`
        2. Password: `admin`

### XBox Controller

TODO: Add instructions for connecting to the robot via the XBox Controller.

### Kinova API

If you would like to use the Kinova Python API, these are the recommended instructions. This approach was verified on Kwesi's MacBook Pro (13-inch, 2020, Four Thunderbolt 3 ports).

1. Download Docker [[Getting Started with Docker](https://www.docker.com/get-started)].
2. Download Visual Studio Code [[Visual Studio](https://code.visualstudio.com/)].
3. From inside Visual Studio Code, download the extensions:
    1.  `Remote - Containers` and
    2.  `Docker` 
4. Clone the OzayGroupExploration repository [[GitHub](https://github.com/kwesiRutledge/OzayGroupExploration)].
5. From the `OzayGroupExploration` directory run the following shell scripts:
    1. `./shell-scripts/create-drake-v1-docker-image.sh`
    2. `./shell-scripts/run-drake-v1-docker-container.sh`
6. Now that the Docker container is running, you will start Visual Studio Code and attach it to the container.
    1. One way to do this is to open Visual Studio Code:
        1. Find the Docker logo in the left hand side of the window and click it.
        2. On the following screen, there should be a list of containers with their statuses under the `Containers` tab.
        3. The container `drake-image-v1` should be running. Right click on it and press the `Attach Visual Studio Code` option.
        4. A new window should open up that is attached to the container. (You will notice that on the left side of the screen you will have access to all of the files that come pre-installed in the container.)
7. (Bonus Step) If you want to check that the code is working, you can try to run the file `101-move_angular_and_cartesian.py` in the `kortex > api_python > examples > 102-Movement_high_level` directory when the robot is connected. It should make the robot execute a sequence of movements.

## Appendices

There are some extra programs which are helpful for working with Docker, Drake and the robot. Their installation instructions are mentioned below.

### Installing the Docker on Linux

Installing Docker on Linux somewhat varies depending on which OS you have.

On the lab's Ubuntu computer, I've used these instructions and they've worked well:

- [Installing Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
- [Adding the Current User to a Privileged Docker group](https://docs.docker.com/engine/install/linux-postinstall/) (so you don't need to use `sudo` every time you run docker)

### Installing Visual Studio Code (and the Docker Extension)

Ideally, Docker will be installed before installing Visual Studio Code.

Visual Studio Code can be found and installed here: [[Visual Studio](https://code.visualstudio.com/)]

After installation, there are a few extensions that help make it easier to work with Docker containers. Search in the extensions bar for the following:

- Docker
- Remote - Containers