# T1 Ranger PNP Characterization Data Requirements

This checklist defines the data needed to replace approximate parameters with a
more defensible T1 Ranger PNP model.

The priorities are ordered by impact on simulation accuracy.

## Known Public Baseline Now Reflected In Config

The default configuration now represents a T1 Ranger PNP hover-mode tricopter
surrogate.

Known from the supplied parts list and public product data:

| Parameter | Current value used | Notes |
| --- | --- | --- |
| Wingspan | `0.73 m` | public T1 Ranger PNP spec |
| Length | `0.645 m` | public T1 Ranger PNP spec |
| Height | `0.14 m` | public T1 Ranger PNP spec |
| Empty weight | `0.451 kg` | documented under `hardware_notes`; not used as simulation mass |
| Takeoff mass range | `0.6-0.75 kg` | documented under `hardware_notes` |
| Simulation mass | `1.4 kg` | user-measured gross VTOL mass with selected battery/configuration |
| Battery selected | Gens Ace `4S 14.8 V 5000 mAh 45C`, T-style connector | nominal `74 Wh`; theoretical label current `225 A` |
| Motors | `3x FX-1806 2000KV` | left tilt, right tilt, rear lift in hover baseline |
| ESCs | `3x FX-25A 25A` | electrical/current limits documented only |
| Propellers | `3x Gemfan 5126` | command-to-thrust map still unknown |
| Tilt servos | `2x metal gear tilt servos` | future front `alpha` dynamics |
| Flight controller | `FX-405 / ArduPilot 4.3.5 factory setup` | use Mission Planner for parameter export |
| Speed envelope | `<20 km/h` minimum, `>120 km/h` maximum | document only until logs or aerodynamic model support it |
| Range/endurance claims | `>25 km`, `>65 min` | do not use for validation until battery, payload, wind, and cruise logs are known |

Derived placeholders now in `config/default_params.json`:

| Parameter | Placeholder method | Replace with |
| --- | --- | --- |
| `vehicle.inertia_kgm2` | rectangular bounding-box estimate from `1.4 kg`, span, length, height | measured inertia or CAD estimate corrected by measured mass |
| `vehicle.rotors[*].position_m` | symmetric hover-stable estimate around CG | measured hub coordinates relative to CG |
| `vehicle.rotors[*].max_thrust_n` | `7.0 N` placeholder sized above the `4.58 N` equal-share hover requirement | static thrust stand data for FX-1806 + 5126 prop + selected 4S battery |
| `vehicle.rotors[*].torque_coefficient` | rough yaw reaction placeholder | thrust/torque stand or log-derived yaw response |
| Drag terms | small damping placeholders | glide/flight-log system identification |

Scope note: the current model uses the two front tilt motors as vertical hover
rotors. It does not yet include the transition angle `alpha`, `alpha_dot`, or
servo dynamics. Add those only after the hover model can be explained, simulated,
and compared against simple logs.

## Parameter Collection Checklist

Measure or investigate these next, in this order:

1. **Mass and balance**

   - Confirm the `1.4 kg` gross mass with battery, receiver, payload, wiring,
     props, and hatches exactly as flown.
   - Measure CG from the nose, from the wing root/chord reference if available,
     and vertically if practical.
   - Weigh the selected 4S 5000 mAh battery by itself.
   - Record alternate mass/CG values for smaller batteries, because the public
     aircraft range is much lighter than the current gross mass.

2. **Rotor geometry**

   - Measure each motor hub position relative to CG:
     `r_i = [x_i, y_i, z_i]^T`.
   - Measure hover thrust axis for each rotor:
     `a_i = [a_x, a_y, a_z]^T`.
   - For the two front tilt motors, measure `alpha_min`, `alpha_max`, neutral
     hover angle, and the sign convention.

3. **Propulsion**

   - Static thrust curve for FX-1806 2000KV + 5126 prop on the selected 4S
     battery: PWM/throttle vs thrust.
   - Current, voltage, and RPM at each thrust point.
   - Max safe continuous thrust per motor, not only burst thrust.
   - Motor/ESC time constant from a step command.
   - Reaction torque or yaw response data for estimating `torque_coefficient`.

4. **Tilt and servo dynamics**

   - PWM-to-tilt-angle calibration for both front tilt servos.
   - Tilt rate limit `alpha_dot_max`.
   - Delay/backlash/deadband.
   - Whether both tilt servos move symmetrically or need separate calibration.

5. **Aerodynamics**

   - Main wing chord, wing area `S`, aspect ratio, and approximate airfoil if
     known.
   - Tail areas and moment arms.
   - Approximate `CL_alpha`, `CD0`, induced drag factor, and stall angle.
   - Airspeed, attitude, throttle, and altitude logs during transition.

6. **Autopilot and logs**

   - Full Mission Planner `.param` export.
   - DataFlash logs for restrained motor tests, hover, climb/descent, and any
     already-safe transition.
   - RC output/PWM logs so the simulation can reconstruct real actuator inputs.
   - Battery voltage/current logs for voltage sag correction.

## Priority 1: Physical Parameters

These are the first measurements to collect because they directly affect hover,
trim, controller gains, and state-space matrices.

| Data | Needed detail | Model location |
| --- | --- | --- |
| All-up mass | Battery installed, payload installed, takeoff-ready mass in kg | `vehicle.mass_kg` |
| Center of gravity | CG location relative to chosen body origin, with battery/payload installed | rotor moment arms and future config |
| Inertia matrix | `Jxx`, `Jyy`, `Jzz`; ideally also products of inertia `Jxy`, `Jxz`, `Jyz` | `vehicle.inertia_kgm2`; later full `3 x 3` inertia |
| Rotor positions | `x, y, z` of each rotor hub relative to CG | `vehicle.rotors[*].position_m` |
| Rotor axes | thrust direction vectors at hover and transition tilt positions | `vehicle.rotors[*].axis_body`; future tilt model |
| Wing geometry | span, chord, area, aspect ratio, airfoil if known | transition/aero model |
| Tail geometry | horizontal/vertical tail area, moment arms, incidence if measurable | transition/aero model |

Minimum acceptable start:

- takeoff-ready mass
- CG
- diagonal inertia estimate
- rotor hub coordinates relative to CG

Best version:

- full inertia matrix from bifilar/trifilar pendulum or CAD with measured mass
  corrections
- repeated CG/inertia measurements for several battery positions

## Priority 2: Propulsion And Actuators

The current model treats rotor thrust commands as direct Newtons. Real vehicles
do not behave that cleanly.

| Data | Needed detail | Model use |
| --- | --- | --- |
| Motor/prop thrust curve | thrust vs PWM/throttle/RPM at relevant battery voltages | command-to-thrust map |
| Torque curve | reaction torque vs thrust or RPM | yaw torque coefficient |
| Electrical data | voltage, current, power during thrust tests | battery sag and efficiency |
| Motor time constant | step response from command to thrust/RPM | actuator dynamics |
| ESC limits | min/max PWM, update rate, saturation behavior | input constraints |
| Servo/tilt response | angle vs command, rate limit, delay, backlash | tilt-gondola generalized coordinate |
| Control surface response | elevator/aileron/rudder angle vs command | fixed-wing transition model |

Minimum acceptable start:

- static thrust stand data for each prop/motor type
- max thrust per rotor at nominal battery voltage
- approximate motor first-order time constant

## Priority 3: Aerodynamics

Aerodynamics will dominate accuracy during transition and forward flight.

| Data | Needed detail | Model use |
| --- | --- | --- |
| Lift curve | `CL(alpha)` over expected angle-of-attack range | wing lift |
| Drag curve | `CD(alpha)` or drag polar `CD = CD0 + k CL^2` | forward-flight drag |
| Pitching moment | `Cm(alpha)` and elevator/control-surface influence | pitch dynamics |
| Side-force/yaw data | `CY(beta)`, `Cn(beta)` if available | crosswind and yaw model |
| Propwash effects | lift/control changes with rotors running | hover/transition coupling |
| Transition schedule | tilt angle, airspeed, and mode blend vs time or command | transition model validation |

Minimum acceptable start:

- wing area, span, chord
- estimated `CL_alpha`
- approximate `CD0` and induced drag factor
- flight-log airspeed/attitude/throttle data through transition

Best version:

- wind-tunnel data, CFD, or system identification from repeated flight logs

## Priority 4: Sensors And Autopilot Logs

The fastest path to useful accuracy is matching simulation output against real
logs.

| Data | Needed detail | Model use |
| --- | --- | --- |
| ArduPilot parameter export | full `.param` file from Mission Planner | real tuning, limits, mixer clues |
| DataFlash logs | hover, climb, descent, transition, fixed-wing cruise | validation and system ID |
| IMU data | accelerometer and gyro rates, vibration levels | dynamics residuals |
| Baro/GPS/airspeed | altitude, groundspeed, airspeed, wind estimate | trajectory and aero validation |
| RC/output logs | commanded PWM and actuator outputs | input reconstruction |
| Battery logs | voltage/current during maneuvers | thrust correction |

Minimum useful log set:

1. Static motor spin-up test with props installed only if safe and restrained.
2. Manual or stabilized hover with small stick inputs.
3. Vertical climb/descent.
4. Gentle yaw/roll/pitch excitation.
5. Transition to forward flight and back, if already safe and routine.

Do not perform risky flights just to collect data. Use existing logs first.

## Priority 5: Environment And Test Conditions

| Data | Needed detail | Model use |
| --- | --- | --- |
| Air density estimate | temperature, pressure, altitude | thrust and aero scaling |
| Wind | speed, direction, gusts | log comparison |
| Battery state | battery type, cell count, voltage under load | thrust scaling |
| Payload state | camera, mounts, payload mass/location | CG and inertia |

## Configuration Changes To Make Later

The current `config/default_params.json` can represent only a simplified
diagonal-inertia rigid-body model. After collecting the data above, upgrade it
in stages:

1. Replace `mass_kg`, `inertia_kgm2`, rotor positions, rotor axes, and
   `max_thrust_n`.
2. Add a full inertia matrix field while preserving the current diagonal field
   for backward compatibility.
3. Add propulsion maps: command-to-thrust, command-to-torque, voltage scaling,
   and motor time constants.
4. Add tilt/servo parameters: tilt angle limits, rate limits, delay, and
   command calibration.
5. Add aerodynamic parameter blocks for wing, tail, fuselage, and transition
   blending.
6. Add sensor/noise parameters derived from logs for EKF and residual tests.

## Accuracy Flaws This Data Addresses

- approximate mass and inertia
- guessed rotor geometry
- direct thrust command assumption
- no motor/ESC/battery dynamics
- simplified yaw reaction torque
- simplified drag
- incomplete transition aerodynamics
- no propwash/control-surface coupling
- no sensor noise, delay, bias, or vibration model
- no validation loop against real Mission Planner/DataFlash logs
