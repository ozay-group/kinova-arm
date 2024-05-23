# Learning to slide (Zexiang's implementation)

This folder contains the scripts that implement the learning2slide demo shown in [this video](https://youtu.be/-_BFBs1N4os).

## Usage

To run this demo, take the following steps

- Turn on the arm and place the car at a location reachable from the arm. Remove unnecessary items from the table.
- Make sure the camera is placed at the right position where the region of interest can be viewed.
- On the laptop, activate the virtual environment and then run the script `learn2slide.py` under this directory.

## Implementation Breakdown

The script `learn2slide.py` mainly performs the following steps repeatedly:

1. Detect the position of the car via AprilTag. The code for AprilTag detection can be found in `AprilTrack.py`.

2. Control the arm to move the car to the right position and slide the car. Corresponding code can be found in `static_controller.py`. 

3. Track the motion of the car while it is slided via AprilTag. The code for AprilTag tracking can be found in `AprilTrack.py`.

4. Learn the function that maps the car initial velocity to its sliding distance.

5. Repeat steps 1-4 until the relation between the initial velocity and the sliding distance is learned (currently only the sliding distance w.r.t the maximal initial velocity (0.4 m/s) is learned). 

6. Command the arm to slide the car to the target region based on the learned mapping.


## Troubleshooting

- If the arm cannot grab the car properly, it is most likely that the camera needs to re-calibrate. Go to the directory [`scripts/camera_calibration`](../../scripts/camera_calibration). Follow the instructions there to perform a camera calibration. Once done, a new `camera_extrinsics.npy` will be generated, which should override the old one under this directory.

- If the AprilTag detection is not working, you can debug the AprilTag detection module by running the script `AprilTrack.py`. Some testing code is provided at the end of the script.