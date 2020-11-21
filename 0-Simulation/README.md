From Robotics initiation class materials by Passault Grégoire, Olivier Ly and Remi Fabre, licensed under a [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/), and modified by our team.

# Simulation
A simple simulation based on PyBullet.

# Requirements
Tested on Python 3.6 but should work on any recent version of Python :
```bash
pip3 install numpy pygame pybullet onshape-to-robot transforms3d scipy
```

# Usage
There are several simulation files but generaly ```python3 sim_hexa.py -h``` gives some info on how to use them.
Make sure the file ```kinematics.py``` is in this folder.
```bash
python3 sim2.py --mode direct
```
```bash
python3 sim2.py --mode inverse
```
```bash
python3 sim2.py --mode circle
```
```bash
python3 sim2.py --mode triangle
```
```bash
python3 sim_hexa -m ultrawalk
```
```bash
python3 sim_hexa -m walk
```
```bash
python3 sim_hexa -m rotation
```
```bash
python3 sim_hexa -m inverse-all
```
```bash
python3 sim_hexa -m inverse
```
```bash
python3 sim_hexa -m robot-ik
```

# One demo video 
https://youtu.be/w3psAbh3AoM
```bash
python3 sim_hexa.py --mode frozen-direct
```
