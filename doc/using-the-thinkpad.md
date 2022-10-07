# Using the ThinkPad Laptop for Control/Simulation

A new thinkpad laptop (KR: Can someone please insert the name and date produced here?) was introduced into the laboratory for exclusive use with the Kinova Gen3.

Because of the difficulty in using the Docker-based setup for that arm, Kwesi configured it to work directly with the arm using Python Virtual Environments. In other words, you DO NOT need to use a Docker Container to use any of the functions for the arm from this laptop.

Instead, you only need to access the arm's virtual environment.

## Starting the Virtual Environment for Manipulation

0. (Optional) Navigate to the directory where Kinova-arm is. (KR: Is this `Documents > manipulation`?)
1. Activate the Virtual Environment by typing `source kinova-arm/virtual-environments/venv2/bin/activate`.
2. The terminal prompt should now be changed, and should show the virtual environment name in parentheses `(venv2)`. (KR: Can someone confirm this?)