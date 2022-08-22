# Test 2 of the April Tag-Based Vision Calibration

## Description
The first test of the calibration was done using a cube in the arm of the gripper. It produced strange results that
were definitely incorrect. The cause of the incorrectness could have been due to the mismatch of frames (between the gripper and the tag).
This test is meant to avoid this issue.

In this test, we will view the same tag from two cameras: the wrist camera on the kinova arm and the external camera. We will compare the pose
of the tag as determined by the Kinova arm + our calibration constant with the pose of the tag as determined by the external realsense + our calibration constant. The two poses should be similar.