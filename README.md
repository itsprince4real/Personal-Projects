# Personal Aerospace & UAV Development Projects

This repository contains design files, embedded control firmware, aerodynamic analyses, and prototyping documentation for custom Unmanned Aerial Vehicle (UAV) systems developed by **Prince** (M.Sc. Aerospace Engineering, Technical University of Munich).

---

## 📌 Project Overview

| Project | System Type | Status | Core Technologies |
| :--- | :--- | :--- | :--- |
| **Tandem-Rotor UAV** | Dual-Rotor Vertical Flight | **Completed** | ESP32, C++, IMU Sensors, PID Loops, Servo Gimbals, Composite Fabrication |
| **F.E.X.1** | Fixed-Wing Flying Wing | **Active Development** | XFLR5 (3D Panel Method), Embedded C++, ESP32, Elevon Mixing, Aerodynamic Polars |

---

## 🚁 1. Tandem-Rotor UAV Prototype

### Status: Completed ✅

A multi-iteration experimental prototype designed to evaluate thrust-vectoring gimbals, dynamic coupling, and pitch/roll stabilization in a dual-rotor tandem vertical flight configuration.

```
Tandem-Rotor-UAV/
├── ESP32_Codes/              # Embedded C++ control loops & sensor drivers
├── Media/                    # Flight test videos & structural iteration photos
└── Tandem_Rotor_UAV_Report.pdf # One-page project executive summary
```

### Key Engineering Milestones:
* **Version 1 (Proof of Concept):** Designed baseline dual-rotor thrust configuration and dual-axis thrust-vectoring gimbal. Identified key structural flexibility and CG offset challenges.
* **Version 2 (Structural Rigidity):** Redesigned composite frame architecture to eliminate mechanical flex and minimize vibration-induced IMU signal noise.
* **Version 3 (Flight Dynamics & Stabilization):** Implemented single-axis thrust vectoring gimbal with dedicated pitch and roll PID loops. Successfully demonstrated short hover stabilization (1–2s) and evaluated ground-effect interaction.

### Key Insights:
* **Hardware:** Structural frame stiffness directly affects IMU noise floor, requiring physical damping before digital filtering.
* **Control:** Cross-coupling dynamics between tandem rotors require tight pitch/roll loop integration to prevent dynamic divergence.

---

## ✈️ 2. Fixed-Wing Elevon Experiment 1 (F.E.X.1)

### Status: Active Development 🔄

F.E.X.1 is a foam-based flying-wing UAV created to bridge numerical aerodynamic modeling with physical embedded flight control software.

```
FEX1_Fixed_Wing/
├── Firmware_Control/         # Embedded C++ elevon mixing & control routines
├── XFLR5_Results/            # Aerodynamic polars (Excel data & generated plots)
│   ├── CL_vs_Alpha.png
│   ├── CD_vs_Alpha.png
│   ├── CL_CD_ratio.png
│   ├── CM_vs_Alpha.png
│   └── XFLR5_Polars_Data.xlsx
├── Media/                    # Airframe assembly & hardware CAD captures
└── FEX1_Report.pdf           # One-page technical summary & polars analysis
```

### Development Roadmap & Current Progress:

1. **Aerodynamic Analysis (Completed) 🟢:**
   * Executed 3D Panel Method analyses in **XFLR5** across three operational airspeeds ($5\text{ m/s}$, $10\text{ m/s}$, and $15\text{ m/s}$) and control surface deflection ranges ($\delta_e = -10^\circ, -5^\circ, +5^\circ, +10^\circ$).
   * Extracted $C_L\text{ vs. }\alpha$, $C_D\text{ vs. }\alpha$, $C_L/C_D\text{ vs. }\alpha$, and $C_M\text{ vs. }\alpha$ polar curves to determine static pitching margins and zero-lift pitching moments ($C_{M0}$).

2. **Control Logic & Firmware (In Progress) 🟡:**
   * Developing C++ elevon mixing algorithms on an **ESP32** microcontroller to translate roll ($\phi$) and pitch ($\theta$) control commands into coordinated dual-servo deflection outputs
   * Integrating MPU6050 IMU sensor feedback for pitch/roll rate stabilization.

3. **Physical Hardware & Flight Testing (Upcoming) 🔴:**
   * Completed physical foam airframe construction and control linkage mounting.
   * Bench-testing servo deflection limits and center-of-gravity (CG) alignment prior to maiden hand-launch flight tests.

---

## 🛠 Tools & Technologies

* **Aerodynamic Modeling & Simulation:** XFLR5, MATLAB
* **Embedded Software & Microcontrollers:** Embedded C++, Arduino IDE, ESP32, MPU6050 IMU
* **Hardware & Prototyping:** Composite Fabrication, Servo Gimbals, Foam Airframes, Thrust-Vectoring Integration

---

## 📬 Contact Information

**Prince**  
M.Sc. Aerospace Student 
Technical University of Munich (TUM) 
