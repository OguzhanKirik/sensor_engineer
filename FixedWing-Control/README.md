
## Fixed-Wing Simulation Controller Implementation for Scenarios

![Fixed Wing Simulation](image/sim.png)

### Longitudinal Scenarios

1. **Trim** – Find a fixed throttle value that allows the aircraft to maintain level flight without elevator input.
2. **Altitude Hold** – Use a pitch inner loop and altitude outer loop controller to maintain a commanded altitude.
3. **Airspeed Hold** – Use throttle PI control to maintain a target airspeed while Unity controls altitude.
4. **Steady Climb** – Use pitch PI control (with full throttle) to maintain airspeed during a steady climb.
5. **Longitudinal Challenge** – Implement a state machine that switches between altitude-hold and climb/descend modes to fly through altitude-based gates.

---

### Lateral/Directional Scenarios

6. **Stabilized Roll Angle** – Use PD roll control to return the aircraft from a 45° bank to wings-level.
7. **Coordinated Turn** – Use PI rudder control to reduce sideslip and achieve coordinated turning.
8. **Constant Course/Yaw Hold** – Use PI control to drive the aircraft heading to a commanded yaw/course.
9. **Straight Line Following** – Convert cross-track error into a course command to follow a straight flight path.
10. **Orbit Following** – Generate course and feed-forward roll commands to track a circular orbit.
11. **Lateral/Directional Challenge** – Use a state machine combining line-following and orbit-following to fly through lateral gates.

---

### Full-System Scenarios

12. **Full 3D Challenge** – Use combined longitudinal and lateral controllers to follow 3D waypoints with line and orbit transitions.
13. **Flying Car Challenge** – Control VTOL takeoff, transition to fixed-wing flight, navigate long-range flight, and transition back to VTOL for landing.

---

## Setup Environment

### 1. Install the Udacidrone Library

Ensure Udacidrone (version **0.3.4 or later**) is installed:


pip install -U git+https://github.com/udacity/udacidrone.git

### 2. Download and Install the Unity Fixed-Wing Simulator

🔗 Simulator Download:
https://github.com/udacity/FCND-FixedWing/releases

> **Note:** This project uses the official **Udacity Fixed-Wing Simulator** and includes **partial control code** intended for educational and practice purposes.
