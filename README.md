# Sustainable-Sonar-System-for-Deep-Scattering-Layer-Monitoring
IEEE Hackathon project for sustainable marine sonar systems and eco-friendly signal processing.

## File Structure Validation
Before running, ensure your `"Simulation (phase 2)"` directory contains the following essential files:
*   `Working interactive proof of concept.py` — The main Python GUI and animation script for the buoy and ROV
*   `sonar.m` — The MATLAB script for signal comparison
*   `wave.xlsx` — The Excel sheet containing for real wave caracteristics based on verified sources


## Setup Instructions
### Prerequisites
*   **Python:** Version 3.8 or higher.
*   **MATLAB:** version R2023a or higher 
*   **Microsoft Excel / OpenOffice:** Required to view or modify the input/output data sheets
### 1. Clone the Repository
Open your terminal or command prompt and run:
```bash
git clone https://github.com/assilmhetlii-oss/Sustainable-Sonar-System-for-Deep-Scattering-Layer-Monitoring.git
cd Sustainable-Sonar-System-for-Deep-Scattering-Layer-Monitoring
```
### 2. Install Dependencies
```bash
pip install numpy matplotlib
```
*Note: Linux users may need to run `sudo apt-get install python3-tk` if tkinter is missing.*


## Execution Instructions
### Step 1: Process Data in MATLAB 
1. Open MATLAB and navigate to the `Simulation (phase 2)` folder.
2. Run 'sonar.m'
### Step 2: Configure Inputs via Excel 
1. Open `wave.xlsx` in Excel.
### Step 3: Launch the Python Simulation
Navigate to the directory and launch the GUI:
```bash
cd "Simulation (phase 2)"
python Working interactive proof of concept.py
```


## Expected Behavior
### Python simulation
*   The terminal will display `"Program started"`.
*   A Tkinter GUI window will open.
*   An interactive 3D Matplotlib graph will render inside the GUI window.
### Matlab simulation
*  A window will open containing two graphs


## Parameters and Configuration
No parameters should be modified

## Assumption
The simulation model is built upon the following technical and theoretical constraints:
* **stationnary and uniform Deep scaterring layer at a fixed depth**
* **Sinusoidal waves**


## Reproduction Steps

To verify that the simulation operates correctly and matches expected benchmarks:
1.  Launch the program using the **Execution Instructions**.
2.  Leave all GUI parameters at their default values.
3.  Click the "Start" button in the interactive GUI for the python simulation , and run the Matlab script
4.  **Expected Result:** The 3D animation should run smoothly without crashing
                         A matlab window will appear containing two graphs
