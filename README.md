# EV Powertrain Automated Test Suite & Control Verification

A MATLAB/Simulink-based electric vehicle (EV) powertrain model coupled with an automated test suite (`run_ev_tests.m`). This project demonstrates Model-Based Design (MBD), closed-loop control verification, Stateflow state machine integration, and automated Software-in-the-Loop (SIL) style verification workflows.

---

## 🚀 Key Features

* **Automated Test Framework:** Executes multiple complex driving and fault scenarios in a single automated script run.
* **Robust Fail-Safe Architecture:** Input validation guards ensuring strict execution constraints (e.g., minimum simulation time T_sim >= 60 s, mandatory safety activation).
* **Stateflow Error Handling:** Dynamic fault injection and state monitoring.
* **Interactive CLI Menu:** Allows selective execution and plotting of individual tests.
* **Advanced Data Visualization:** Automated generation of dark-mode optimized multi-signal plots (Speed, Thermal state, Armature Current) with high-performance data downsampling.

---

## 🧪 Test Scenarios Included

1. **Normal Operation (Test 1):** Validates steady-state performance, thermal stability, and current draws under nominal flat road conditions.
2. **Fault Injection (Test 2):** Tests system resilience and fault code generation via Stateflow logic upon triggering error flags.
3. **Uphill Driving (Test 3):** Simulates powertrain performance and torque demand under an 8% road incline.
4. **Closed-Loop Deceleration Step Response (Test 4):** Evaluates closed-loop controller speed tracking and transient response during rapid reference step changes.
5. **Speed Limit Regulation (Test 5):** Verifies dynamic speed profile tracking (e.g., speed restriction between 45s and 55s).

---

## 🛠️ Tech Stack & Tools

* **Environment:** MATLAB & Simulink
* **Control Logic:** Stateflow
* **Scripting:** MATLAB Automation Scripts, Time-Series / Matrix Signal Management
* **Visualization:** Custom figure handling with dark theme aesthetics

---

## ⚙️ Getting Started

1. Clone the repository:
   git clone https://github.com/your-username/ev-powertrain-simulation.git

2. Open MATLAB and navigate to the project directory.

3. Run the automated test suite in the MATLAB Command Window:
   run_ev_tests

4. Follow the interactive prompts in the command window to input desired vehicle speed, simulation time (>= 60 s), and select output plots.