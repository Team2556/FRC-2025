# RobotPython2025
 Fresh start ; created for the 2025 season; transition to python


 # FRC Robot Project

This repository contains the code for our FRC robot using Python, PyRobot, and WPILib.

## Setup

0. Create a python based profile in your VS Code: [optional profile file](RobotPy.code-profile)

1. Make sure you have Python 3.12.5
    ```sh
    python --version
    ------------------------ 
    OUTPUT: Python 3.12.5
1a. If not go to [Other Resources] and download it.

2. Clone the repository:
   ```sh
   git clone https://github.com/Team2556/RobotPython2025.git
   cd RobotPython2025
3. ??automatic for newcomer?? Activate the virtual environment:
    ```sh
    On Windows: 
    venv\Scripts\activate
    On macOS/Linux: 
    source venv/bin/activate
4. ??automatic for newcomer?? Install the dependencies:
    ```sh
    pip install . #the dot references your current directory
<!-- 

5. initialize robotpy
    ```sh

 3. Create folders named 'subsytems' and 'tests'
    ```sh
    mkdir subsytems
    mkdir tests
    py -3 -m robotpy init -->
<!--

7. move robot.py file created by initilization to 'src' folder
    move robot2.py src\ -->

4. Run the robot code:
    ```sh
    python src/robot.py

# Repository Structure
- We plan to use the following structure.
    <!-- ├── src/
    │   ├── ?maybe? __init__.py
    │   ├── robot.py
    ├── ?maybe? config/
    │   └── config.yaml
    -->
    ```sh
    FRC-Robot-Project/
    ├── robot.py
    ├── constants.py
    ├── subsystems/
    │   ├── __init__.py
    │   └── example_subsystem.py 
    ├── commands/
    │   ├── __init__.py
    │   └── example_command.py 
    ├── autonomous/
    │   ├── __init__.py
    │   └── example_command.py 
    ├── pathplanner/deploy/pathplanner/
    │   ├── autos
    │   └── paths
    ├── tests/
    │   ├── __init__.py
    │   └── test_robot.py
    ├── .gitignore
    ├── README.md
    ├── pyproject.toml
    └── venv/

# Other Resources
- Download python (3.12.5): https://www.python.org/downloads/windows/
- Python tutorial: https://docs.python.org/3.12/tutorial/index.html
- More instructions from WPI: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html
- deploy Python to RoboRIO: https://docs.wpilib.org/en/stable/docs/software/python/subcommands/deploy.html
- Command based programming: https://docs.wpilib.org/en/latest/docs/software/commandbased/what-is-command-based.html


# VS Code Extensions
TODO: Refer to our main github readme or sync here....
In the VS Code Extensions sidbar, these codes should help you find usefull extensions (well.. Git and Python are required)
- ms-python.python
- github.vscode-pull-request-github
- github.copilot
- ms-python.black-formatter
- njpwerner.autodocstring
- tamasfe.even-better-toml
- streetsidesoftware.code-spell-checker

## ORIGINAL Setup (FYI)

1. Clone the repository:
   ```sh
   git clone https://github.com/Team2556/RobotPython2025.git
   cd RobotPython2025
2. Create a vertual environment (from a terminal; in the repo directry on your pc)
    ```sh
    python -m venv venv
3. Activate the virtual environment:
    ```sh
    On Windows: 
    venv\Scripts\activate
    On macOS/Linux: 
    source venv/bin/activate
<!-- 4. Install the dependencies:
    ```sh
    pip install . #the dot references your current directory -->

5. initialize robotpy
    ```sh
    py -m robotpy init

<!-- 6. Create folders named 'src/subsytems' and 'tests'
    ```sh
    mkdir src/subsytems
    mkdir tests

7. move robot.py file created by initilization to 'src' folder
    move robot2.py src\ -->

6. Run the robotpy sync to get the RoboRIO python
    ```sh 
    py -m robotpy sync
8. Run the robot code:
    ```sh
    python src/robot.py