# Simulation
An hexapod simulation based on PyBullet.

# Requirements
Tested on Python 3.6 but should work on any recent version of Python :
```bash
pip3 install numpy pygame pybullet onshape-to-robot transforms3d scipy
```
## sim_hexa
```python3 sim_hexa.py -h``` gives some info on how to use the file in the hexapod simulation.
Make sure the file ```kinematics.py``` is in this folder.
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

## sim2
This file is specially used for basic simulations on a single arm and is not useful for hexapod simulation use
