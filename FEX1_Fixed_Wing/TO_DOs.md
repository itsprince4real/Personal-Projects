# F.E.X.1 Development Roadmap & Status

## Completed Milestones
- [x] Airframe geometry CAD design & physical foam fabrication
- [x] 3D aerodynamic polar analysis in XFLR5 ($C_L, C_D, C_M$ across $5\text{ m/s}$, $10\text{ m/s}$, $15\text{ m/s}$)
- [x] Elevon surface deflection limits & static pitch stability trim verification

---

## Active & Upcoming Tasks

### 1. Elevon Mixing Algorithm Development
- Implement C++ mixing logic on ESP32 microcontroller.
- Map pilot control inputs to PWM servo signals.

### 2. Thrust Control & ESC Integration
- Configure electronic speed controller (ESC) pulse-width modulation limits.
- Set up motor safety arming sequences and throttle mapping.

### 3. Hardware Integration & Bench Testing
- Perform dynamic bench testing to verify surface deflections and eliminate mechanical binding.

### 4. Field Flight Testing Campaign
- Execute initial hand-launch trim tests.
- Record real-time telemetry log for post-flight stability analysis against XFLR5 predictions.
