# PythonVehicleControl
![alt text](https://raw.githubusercontent.com/yongkyuns/PythonVehicleControl/master/img.png)
PythonVehicleControl is an environment for experimentation of control algorithms and data visualization of autonomous vehicles. This project provides:
    1. Simulation environment (i.e. dynamics model, path planning & control algorithms)
    2. Declarative visualization (i.e. animated 3D objects and 2D graphs)
    3. Interactive GUI
The project is written in python and is a casual attempt to solve some of the daily challenges in desigining control algorithms for autonomous vehicles and visualizing their data in a modulalar and reconfigurable manner.

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
python main.py
```
For a more detailed documentation, please refer to https://pythonvehiclecontrol.readthedocs.io/en/master/#

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)