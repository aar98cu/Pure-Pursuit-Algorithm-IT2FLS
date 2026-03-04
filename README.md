# 🚜 Pure Pursuit Framework: IT2FLS & T1FLS Research Suite

This repository contains an advanced simulation environment for **Pure Pursuit** path tracking algorithms, specifically focused on **Interval Type-2 Fuzzy Logic Controllers (IT2FLS)** and **Type-1 FLC**. Designed for academic and doctoral research, it provides high-fidelity visualization, noise simulation, and batch benchmarking.

## � Project Structure

Organized following software engineering best practices for research projects:

```bash
.
├── app.py                # Modern Web-based Dashboard (NiceGUI + Plotly)
├── pure_pursuit.py       # Legacy Native GUI (DearPyGui)
├── requirements.txt      # Project dependencies
├── src/                  # Core source code
│   ├── config.py         # Hyperparameters & Path data
│   ├── functions.py      # FLS Models & Control logic
│   ├── vehicle.py        # Kinematic models & Geometry
│   └── themes.py         # UI Styling & Palettes
├── data/                 # Extended trajectory datasets (CSV/JSON)
├── results/              # Simulation outputs
│   ├── simple/           # Single run logs and plots
│   └── multi/            # Batch benchmark statistics
└── docs/                 # Mathematical background & Thesis notes
```

## 🚀 Getting Started

### Prerequisites
- Python 3.9+
- Dependencies: `nicegui`, `plotly`, `dearpygui`, `numpy`, `pyautogui`

```bash
pip install -r requirements.txt
```

### Running the Environment

#### 1. Research Dashboard (Recommended)
A modern web-based interface with zoomable plots, light/dark mode, and detailed strategy descriptions.
```bash
python app.py
```
*Access via browser at `http://localhost:8080`*

#### 2. Native Desktop Simulation
High-performance native window implementation.
```bash
python pure_pursuit.py
```

## 🧠 Control Strategies Included

- **Geometric Lateral Control**: Traditional Pure Pursuit implementation.
- **T1FLC**: Type-1 Fuzzy Logic with optimized Gaussian membership functions.
- **IT2FLC (2MF/3MF)**: Interval Type-2 Fuzzy Logic for handling extreme uncertainty and non-linearities.
- **PI Distance/Velocity**: Classical linear control benchmarks.

## 📊 Uncertainty & Noise
The framework allows simulating real-world sensor inaccuracies through:
- Pre-allocated Gaussian noise matrices.
- Steering dynamic restrictions (Transfer functions).
- Measurement time-step jitter.

## 📝 Research & Citations
*This project is part of a Doctoral Research initiative on Autonomous Vehicle Control.*
For mathematical details on the IT2FLS implementation, please refer to the `docs/` folder.

---
**Lead Developer:** M.Sc. in Mechatronics Adrian Alberto Rodriguez  
**Contact:** [adriancuba1998@gmail.com](mailto:adriancuba1998@gmail.com)
