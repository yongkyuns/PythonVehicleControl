# PythonVehicleControl
![alt text](https://raw.githubusercontent.com/yongkyuns/PythonVehicleControl/master/img.png)
PythonVehicleControl is an environment for experimentation of control algorithms and data visualization of autonomous vehicles. With an increasing level of driver-assistance features being adopted in mainstream vehicles, it is a challenge for software engineers to test prototype control algorithms for these systems and quickly analyze the measurement data produced by them in an intuitive scheme (i.e. by visualizing raw sensor measurements,
as well as intermediate output of the software in a 3D environment). This project addresses some of these challenges by providing a simulation environment and data visualization layer which is modular and easily configurable for the task of designing autonomous vehicles. Since the project is developed in python, it also allows prototyping and evaluation of machine-learning concepts for these systems.

## Requirements
This project is being developed with Python 3 and uses PythonRobotics (https://github.com/AtsushiSakai/PythonRobotics) as a submodule. For visualization, pyqtgraph is used. For a detailed list of dependencies, please refer to the requirements.txt file.

## Installation
To install this project, first download with the following url.
```bash
git clone --recurse-submodules https://github.com/yongkyuns/PythonVehicleControl.git
```
Move to the downloaded folder.
```bash
cd PythonVehicleControl
```
Then install the required packages.
```bash
pip install -r requirements.txt 
pip install git+https://github.com/pyqtgraph/pyqtgraph.git@develop
```
Now we are ready to run!

## Usage
To run the simulator, use the following shell commands in the project directory:
```bash
python simulator.py
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)