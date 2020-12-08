# Simulation
An hexapod simulation based on PyBullet.

# Requirements
Tested on Python 3.6 but should work on any recent version of Python :
```bash
pip3 install numpy pygame pybullet onshape-to-robot transforms3d scipy
```
Make sure the files ```kinematics.py``` and ```constants.py``` are in this folder.

## constants
In this file :
- When **sim2.py** is used, for arm simulation, need to put ```ARM_SIMULATION``` at the location provided by the mode choice
- When **sim_hexa.py** is used, for hexapod simulation, need to put ```PHANTOMX_SIMULATION``` at the location provided by the mode choice


After the choice of the mode, save the file and you can go use the sim file you want.

_By default, the ARM_SIMULATION is selected, for hexapod simulation_


## kinematics 
Except if you know what you do, there is nothing to change in this file.

It contains the mathematical and geometrical functions for simulations.


## sim2
This file is specially used for basic simulations on a single arm and is not useful for hexapod simulation use.
```bash
python3 sim2.py -m direct
```
```bash
python3 sim2.py -m inverse
```
```bash
python3 sim2.py -m circle
```
```bash
python3 sim2.py -m triangle
```


## sim_hexa
```python3 sim_hexa.py -h``` gives some info on how to use the file for the hexapod simulation.
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
