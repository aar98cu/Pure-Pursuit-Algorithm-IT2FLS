🚜 Pure Pursuit Algorithm Simulation
This repository contains the code for simulating a vehicle's trajectory using different lateral and velocity control methods under varying conditions. The simulation is built using DearPyGUI for the interface and includes options for noise simulation to replicate real-world conditions.

📂 Project Structure
```bash
Copiar código
.
├── pure_pursuit.py          # Main interface and simulation control
├── functions.py             # Core functions and control methods
├── config.py                # Configuration and parameters
├── vehicle.py               # Vehicle model and dynamics
├── results/                 # Directory for simulation results
└── README.md                # This readme file
```
🚀 Getting Started
Prerequisites
Ensure you have Python 3.8+ installed along with the following libraries:

DearPyGUI
Numpy
Matplotlib
You can install the necessary packages using pip:

bash
Copiar código
pip install dearpygui numpy matplotlib
Running the Simulation
Clone the repository:

bash
Copiar código
git clone https://github.com/yourusername/pure-pursuit-simulation.git
cd pure-pursuit-simulation
Run the simulation:

bash
Copiar código
python pure_pursuit.py
Configuration
The simulation can be configured by modifying the config.py file. You can adjust parameters such as the vehicle's dimensions, control methods, noise settings, and more.

Simulation Controls
Path Selection: Choose from predefined paths.
Steering Control: Select the method for steering control (Geometric, Error-based, IT2FLS, etc.).
Velocity Control: Select the method for velocity control (PI control, IT2FLS, etc.).
Noise Simulation: Enable or disable noise to simulate real-world inaccuracies.
Maximum Speed & Deltas: Adjust the maximum speed and the delta values for steering and speed.
Results
Simulation results are saved in the results/ directory, with file names indicating the control method, path, noise setting, and maximum speed. You can analyze these results using the built-in plotting functions or externally in tools like Excel or Python scripts.

📊 Analyzing Multiple Simulations
The simulation can be run multiple times to generate statistical data for comparison. Modify the run_multiple_simulations function in functions.py to execute the simulation repeatedly and analyze the results.

✨ Features
Multiple lateral and velocity control methods.
Realistic noise simulation to mimic real-world conditions.
Statistical analysis and visualization of simulation results.
Customizable interface with DearPyGUI.
🛠️ To-Do
 Add support for additional control algorithms.
 Improve real-time visualization of the simulation.
 Implement a more sophisticated analysis pipeline.
📝 License
This project is licensed under the MIT License - see the LICENSE file for details.

🤝 Contributing
Contributions, issues, and feature requests are welcome! Feel free to check the issues page for open issues or create a new one.

📧 Contact
For any questions, feel free to reach out at adriancuba1998@gmail.com