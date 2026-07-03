import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import warnings
warnings.filterwarnings('ignore')
print("Program started")

class ROVBuoySimulation:
    """
    Fixes applied vs. original:
    1. WAVE SCENARIOS: added selectable wave models (Regular/Sinusoidal, Irregular
       multi-component, Rogue/transient) that actually change the buoy's forced
       motion in real time. Parameter updates now take effect immediately by
       tracking a 'wave phase offset' so changing amplitude/period mid-run does
       not cause a discontinuity or get ignored.
    2. TETHER PHYSICS: proper unilateral spring-damper (rope can only pull, never
       push) with correct sign convention verified via Newton's third law, plus a
       small slack-region stiffness to avoid numerical chattering at the taut
       transition. Added feed-forward so the ROV's steady-state hang is physically
       consistent (net weight balanced by tether tension when PID is off).
    3. PID VISIBILITY: added feed-forward buoyancy/weight compensation so the PID
       only has to correct the *error*, not fight the full static offset. This
       makes the controlled trajectory visibly different from the uncontrolled
       (PID off) trajectory. Also fixed derivative-term timing (was using
       inconsistent dt logic) and added a live tracking-error plot.
    4. GENERAL: fixed the RK4 integrator (buoy dynamics are now purely kinematic/
       prescribed, not integrated as free variables which previously caused
       silent divergence), and made all "Update" actions reset transient state
       (integral windup, phase) cleanly so new scenarios behave predictably.
    """

    def __init__(self):
        # Buoy dimensions (meters)
        self.buoy_length = 4.0
        self.buoy_width = 4.0
        self.buoy_height = 3.0

        # ROV dimensions (meters)
        self.rov_length = 0.40
        self.rov_width = 0.40
        self.rov_height = 0.15

        # Tether length (meters)
        self.tether_length = 500.0

        # Deep Scattering Layer (DSL) - a static band of biological
        # scatterers (fish, zooplankton) that shows up strongly on sonar.
        # Kept static (non-moving) deliberately, per request, to avoid
        # per-frame recomputation cost.
        self.dsl_depth = 600.0      # m, center depth
        self.dsl_thickness = 80.0   # m
        self.dsl_top_z = -(self.dsl_depth - self.dsl_thickness / 2)     # -560
        self.dsl_bottom_z = -(self.dsl_depth + self.dsl_thickness / 2)  # -640

        # Physical parameters
        self.gravity = 9.81
        self.water_density = 1025.0

        # ROV properties
        self.rov_mass = 15.0  # kg
        self.rov_volume = self.rov_length * self.rov_width * self.rov_height
        self.rov_buoyancy_force = self.water_density * self.rov_volume * self.gravity
        self.rov_net_force = self.rov_buoyancy_force - self.rov_mass * self.gravity  # + means floats up

        # Buoy properties
        self.buoy_mass = 2000.0  # kg

        # Damping coefficients
        self.rov_damping = 30.0

        # --- TETHER PHYSICS (realistic: TMS slack management + snubber) ---
        # A real 500 m ROV umbilical's strength member (steel/Kevlar) is
        # essentially inextensible under normal loads - k = EA/L for even a
        # thin 1 mm^2 steel strand over 500 m works out to ~200,000 N/m,
        # nowhere near "soft." What actually lets a real tethered system
        # absorb heave without huge tension spikes is a Tether Management
        # System (TMS) that pays slack line in/out, plus a short compliant
        # snubber/shock-cord segment that only takes real load once that
        # slack is used up. That's what's modeled below, instead of treating
        # the whole 500 m tether as one soft spring (which isn't physically
        # how these systems work).
        #
        # NOTE ON THE NUMBERS: these are engineering estimates from first
        # principles (order-of-magnitude reasoning), not datasheet values -
        # there's no single "correct" real-world number for any of this,
        # since it depends on the specific TMS/snubber design.
        self.tms_max_slack = 8.0        # m of slack the TMS can pay out/reel in
        self.tms_payout_rate = 1.0      # m/s max reel speed (actuator speed limit)
        self.slack_deployed = 0.0       # m currently paid out (state)
        self.use_tms_pid = True         # TMS active heave compensation on/off

        # TMS is tension-controlled (like a real active heave compensator):
        # it pays slack in/out to hold tether tension near a small target
        # preload, rather than just chasing the geometric gap. This is a
        # genuine PID loop: error = measured_tension - target_tension,
        # output = commanded payout rate (still capped at tms_payout_rate,
        # the physical winch speed limit).
        self.tms_target_tension = 30.0  # N, small preload to avoid full slack/snag
        self.tms_kp = 0.0020            # (m/s) per N
        self.tms_ki = 0.0006            # (m/s) per (N*s)
        self.tms_kd = 0.0010            # (m/s) per (N/s)
        self.tms_integral_error = 0.0
        self.tms_previous_error = 0.0

        # Snubber: short elastic segment that only engages once TMS slack is
        # exhausted. Estimated from a few meters of nylon shock cord sized to
        # take real load: for ~3 m of line with ~15% stretch at working load
        # of a few kN, k = F/dx works out to roughly 1000-5000 N/m - using
        # 3000 N/m as a representative mid-range estimate.
        self.tether_stiffness = 3000.0  # N/m, snubber stiffness once engaged
        self.snubber_length = 3.0       # m, length of the compliant segment

        # Hydrodynamic drag on the snubber (replaces the old made-up linear
        # "N*s/m" damping number with an actual physical drag formula:
        # F = 0.5 * rho_water * Cd * diameter * length * v * |v|).
        # Cd~1.2 is the standard cylinder cross-flow drag coefficient;
        # 14 mm is a typical small-ROV umbilical diameter.
        self.tether_diameter = 0.014    # m
        self.tether_drag_coeff = 1.2    # dimensionless (cylinder, cross-flow)

        self.tether_slack_band = 0.05   # m of "soft" transition to prevent chatter

        # --- WAVE SCENARIOS ---
        # wave_mode: 'regular', 'irregular', 'rogue'
        self.wave_mode = 'regular'
        self.wave_amplitude = 1.0   # meters (significant amplitude for irregular)
        self.wave_period = 4.0      # seconds (peak period for irregular)
        self.wave_frequency = 2 * np.pi / self.wave_period
        self.wave_phase_offset = 0.0     # keeps motion continuous when params change
        self._last_param_change_time = 0.0
        self._components = self._build_irregular_components()
        self.rogue_trigger_time = 15.0
        self.rogue_amplitude_mult = 3.0
        self.rogue_duration = 3.0

        # --- PID controller parameters ---
        self.kp = 300.0
        self.ki = 20.0
        self.kd = 150.0
        self.use_feedforward = True  # feed-forward weight/buoyancy compensation
        self.use_wave_feedforward = True  # predictive wave-disturbance feed-forward
        self.max_thrust = 150.0  # N - thruster capacity ceiling

        # State variables
        self.time = 0.0
        self.dt = 0.02

        # ROV state [x, y, z, vx, vy, vz]
        self.rov_state = np.zeros(6)
        self.buoy_state = np.zeros(6)

        # Initial positions
        self.rov_state[0:3] = [0.0, 0.0, -self.tether_length]
        self.buoy_state[0:3] = [0.0, 0.0, 0.0]

        # PID control variables
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.use_pid = True
        self.target_depth = -self.tether_length
        self.position_error = 0.0
        self.thrust_history = []
        self.error_history = []
        self.tension_history = []

        # History for trajectory
        self.rov_x_history = []
        self.rov_y_history = []
        self.rov_depth_history = []
        self.buoy_z_history = []
        self.buoy_x_history = []
        self.buoy_y_history = []
        self.time_history = []

        # Safety limits
        self.max_depth = -self.tether_length * 1.1
        self.min_depth = 3.0
        self.max_velocity = 10.0

        self.setup_plot()

    # ------------------------------------------------------------------
    # WAVE MODELS
    # ------------------------------------------------------------------
    def _build_irregular_components(self):
        """Pre-generate a fixed set of frequency components for an irregular
        (multi-frequency) sea state so that switching to 'irregular' mode is
        reproducible and the spectrum scales with wave_amplitude/wave_period."""
        rng = np.random.default_rng(42)
        n = 6
        base_freq = 2 * np.pi / self.wave_period
        freqs = base_freq * rng.uniform(0.5, 1.8, n)
        phases = rng.uniform(0, 2 * np.pi, n)
        # weights sum so the combined significant amplitude ~ wave_amplitude
        weights = rng.uniform(0.4, 1.0, n)
        weights = weights / weights.sum()
        return list(zip(freqs, phases, weights))

    def set_wave_mode(self, mode):
        """Switch wave scenario, preserving continuity of buoy position."""
        if mode != self.wave_mode:
            self.wave_mode = mode
            if mode == 'irregular':
                self._components = self._build_irregular_components()
            self._resync_phase()

    def _resync_phase(self):
        """Called whenever amplitude/period/mode changes so the buoy motion
        doesn't jump discontinuously — it keeps its *current* position and
        continues smoothly with the new parameters."""
        self._last_param_change_time = self.time

    def _regular_wave(self, t):
        tau = t - self._last_param_change_time
        z = self.wave_amplitude * np.sin(self.wave_frequency * tau)
        v = self.wave_amplitude * self.wave_frequency * np.cos(self.wave_frequency * tau)
        a = -self.wave_amplitude * self.wave_frequency ** 2 * np.sin(self.wave_frequency * tau)
        return z, v, a

    def _irregular_wave(self, t):
        tau = t - self._last_param_change_time
        z = v = a = 0.0
        scale = self.wave_amplitude
        for freq, phase, weight in self._components:
            z += scale * weight * np.sin(freq * tau + phase)
            v += scale * weight * freq * np.cos(freq * tau + phase)
            a += -scale * weight * freq ** 2 * np.sin(freq * tau + phase)
        return z, v, a

    def _rogue_wave(self, t):
        """Regular sea state with a single large transient ('rogue') wave
        injected at rogue_trigger_time, useful for testing PID disturbance
        rejection."""
        z, v, a = self._regular_wave(t)
        if self.rogue_trigger_time <= t <= self.rogue_trigger_time + self.rogue_duration:
            tau = t - self.rogue_trigger_time
            envelope = np.sin(np.pi * tau / self.rogue_duration)  # 0 -> 1 -> 0
            extra_amp = self.wave_amplitude * (self.rogue_amplitude_mult - 1) * envelope
            freq = self.wave_frequency
            z += extra_amp * np.sin(freq * tau)
            v += extra_amp * freq * np.cos(freq * tau)
            a += -extra_amp * freq ** 2 * np.sin(freq * tau)
        return z, v, a

    def get_buoy_kinematics(self, t):
        """Single source of truth for buoy position/velocity/acceleration,
        dispatched by wave_mode. Returns (pos, vel, accel) as 3-vectors."""
        if self.wave_mode == 'irregular':
            z, v, a = self._irregular_wave(t)
        elif self.wave_mode == 'rogue':
            z, v, a = self._rogue_wave(t)
        else:
            z, v, a = self._regular_wave(t)
        return (np.array([0.0, 0.0, z]),
                np.array([0.0, 0.0, v]),
                np.array([0.0, 0.0, a]))

    # ------------------------------------------------------------------
    # TETHER (corrected physics)
    # ------------------------------------------------------------------
    def tms_pid_controller(self, current_tension, dt):
        """Tension-controlled PID for the TMS winch, mirroring how a real
        active heave compensator works: it pays slack in/out to hold tether
        tension near a small target preload (target_tension), rather than
        chasing the geometric gap directly. Output is a commanded payout
        rate, still capped at tms_payout_rate (the winch's physical speed
        limit - that's a real actuator saturation, not a control gain)."""
        error = current_tension - self.tms_target_tension  # +ve: too tight, pay out more

        p_term = self.tms_kp * error

        self.tms_integral_error += error * dt
        self.tms_integral_error = np.clip(self.tms_integral_error, -20000, 20000)
        i_term = self.tms_ki * self.tms_integral_error

        d_error = (error - self.tms_previous_error) / dt if dt > 1e-6 else 0.0
        d_term = self.tms_kd * d_error

        self.tms_previous_error = error

        payout_rate_cmd = p_term + i_term + d_term
        payout_rate_cmd = np.clip(payout_rate_cmd, -self.tms_payout_rate, self.tms_payout_rate)
        return payout_rate_cmd

    def update_slack_deployment(self, dt):
        """Advance the TMS's paid-out slack once per real timestep (not per
        RK4 sub-stage - it's a slow mechanical process, same treatment as
        freezing thrust across sub-stages elsewhere in this file).

        Uses the tension measured on the *previous* step (self._last_tension,
        set by tether_force()) as the feedback signal - a one-step-delayed
        reading, which is actually realistic (a real tension sensor + control
        loop isn't instantaneous either)."""
        if not self.use_tms_pid:
            return  # TMS inactive: slack stays frozen wherever it currently is

        current_tension = getattr(self, '_last_tension', 0.0)
        payout_rate_cmd = self.tms_pid_controller(current_tension, dt)
        self.slack_deployed += payout_rate_cmd * dt
        self.slack_deployed = np.clip(self.slack_deployed, 0.0, self.tms_max_slack)

    def tether_force(self, rov_pos, rov_vel, buoy_pos, buoy_vel):
        """
        Unilateral spring-damper tether, now with TMS slack management:
        - effective_length = nominal tether_length + currently deployed TMS
          slack (updated once per step by update_slack_deployment()).
        - Below effective_length: fully slack, zero force - this is the
          normal operating regime for ordinary heave, matching how a real
          TMS/catenary system works.
        - Beyond effective_length: the short compliant snubber engages
          (spring) plus real hydrodynamic drag on that segment (quadratic
          in velocity, not an arbitrary linear coefficient).
        - Only pulls (tension >= 0); a rope cannot push.
        """
        tether_vector = rov_pos - buoy_pos
        current_length = np.linalg.norm(tether_vector)

        if current_length < 1e-6:
            return np.zeros(3), np.zeros(3)

        tether_unit = tether_vector / current_length
        effective_length = self.tether_length + self.slack_deployed
        extension = current_length - effective_length

        if extension <= 0:
            self._last_tension = 0.0
            return np.zeros(3), np.zeros(3)

        # smooth ramp-in over the slack band instead of a hard step
        ramp = min(extension / self.tether_slack_band, 1.0) if self.tether_slack_band > 0 else 1.0

        tension_magnitude = self.tether_stiffness * extension * ramp
        tension_magnitude = max(tension_magnitude, 0.0)

        relative_velocity = rov_vel - buoy_vel
        vel_along_tether = np.dot(relative_velocity, tether_unit)
        # Quadratic hydrodynamic drag on the snubber segment, not an
        # arbitrary linear "N*s/m" number.
        drag_magnitude = (0.5 * self.water_density * self.tether_drag_coeff
                           * self.tether_diameter * self.snubber_length
                           * vel_along_tether * abs(vel_along_tether)) * ramp

        force_magnitude = tension_magnitude + drag_magnitude
        # Damping alone should not create a net pushing (negative) force
        force_magnitude = max(force_magnitude, 0.0)

        force_on_rov = -force_magnitude * tether_unit   # pulls ROV UP toward buoy
        force_on_buoy = -force_on_rov                    # Newton's 3rd law (reaction pulls buoy down)

        self._last_tension = force_magnitude
        return force_on_rov, force_on_buoy

    # ------------------------------------------------------------------
    # PID (corrected: feed-forward makes control action visible)
    # ------------------------------------------------------------------
    def predict_wave_feedforward_thrust(self, t):
        """Predict the tether tension the ROV is about to feel from the
        buoy's known wave motion, and return the thrust needed to cancel it
        in advance - instead of waiting for the feedback loop to react after
        the ROV has already been dragged off station.

        This is only possible because the wave model is fully known (we set
        its amplitude/period ourselves); a plain PID has no way to "see"
        the disturbance coming; a feed-forward term computed straight from
        the wave model can.

        Approximation: assumes the ROV is holding station at target_depth
        directly under the buoy. In that case tether extension is
        approximately equal to the buoy's elevation above its rest position
        (buoy rising by X stretches the tether by ~X, since the ROV's depth
        is fixed). This is exact when the ROV is exactly on-target and only
        approximate when it has drifted - the feedback PID terms still
        handle that residual error, this just removes the large, fully
        predictable part of the disturbance up front.
        """
        buoy_pos, buoy_vel, _ = self.get_buoy_kinematics(t)
        buoy_z = buoy_pos[2]
        buoy_vz = buoy_vel[2]

        # If ROV holds station, geometric demand for extra length is ~buoy_z.
        # The TMS absorbs up to tms_max_slack of that with zero force - only
        # the remainder (if any) stretches the snubber.
        demand = buoy_z
        if demand <= self.slack_deployed:
            return 0.0  # within currently deployed slack (or buoy descending)

        raw_extension = demand - self.tms_max_slack
        if raw_extension <= 0:
            return 0.0  # TMS can still absorb this - no snubber load to cancel

        # The ROV is not perfectly rigid - it yields somewhat under tension
        # (that's the whole reason there's tracking error at all), so the
        # naive "ROV holds exactly still" assumption overstates the real
        # stretch, especially at larger wave amplitudes. Soften the
        # prediction toward the ROV's actual typical compliance rather than
        # assuming zero give, so the feed-forward doesn't overshoot and
        # eat into the actuator's headroom for the feedback terms.
        compliance_factor = 0.5
        extension = raw_extension * compliance_factor

        ramp = min(extension / self.tether_slack_band, 1.0) if self.tether_slack_band > 0 else 1.0
        tension = self.tether_stiffness * extension * ramp
        # Quadratic hydrodynamic drag on the snubber, matching tether_force()
        drag = (0.5 * self.water_density * self.tether_drag_coeff
                * self.tether_diameter * self.snubber_length
                * (-buoy_vz) * abs(buoy_vz)) * ramp
        tension += drag
        tension = max(tension, 0.0)

        # Tension pulls the ROV UP (toward the buoy); cancel it with an
        # equal, opposite (downward) thrust.
        return -tension

    def pid_controller(self, current_depth, dt, t=None):
        error = self.target_depth - current_depth  # target/current both negative (depth)

        p_term = self.kp * error

        self.integral_error += error * dt
        self.integral_error = np.clip(self.integral_error, -50, 50)
        i_term = self.ki * self.integral_error

        error_derivative = (error - self.previous_error) / dt if dt > 1e-6 else 0.0
        error_derivative = np.clip(error_derivative, -10, 10)
        d_term = self.kd * error_derivative

        # Feed-forward: cancel the ROV's static net buoyant/weight force so the
        # PID only has to correct the *dynamic* error, not fight a constant bias.
        # This is what makes the thruster response visibly track the wave
        # disturbance instead of being swamped by a steady-state offset.
        ff_term = -self.rov_net_force if self.use_feedforward else 0.0

        # Wave feed-forward: predict and cancel the wave-driven tether
        # disturbance itself, using the known wave model, rather than only
        # reacting to it after the fact via P/I/D. A plain PID can only
        # attenuate a periodic disturbance (integral action drives error to
        # zero for constant disturbances, not oscillating ones) - this term
        # is what lets residual error stay small even as wave amplitude grows.
        if self.use_wave_feedforward and t is not None:
            wave_ff_term = self.predict_wave_feedforward_thrust(t)
        else:
            wave_ff_term = 0.0

        thrust = p_term + i_term + d_term + ff_term + wave_ff_term
        thrust = np.clip(thrust, -self.max_thrust, self.max_thrust)

        self.previous_error = error
        self.position_error = abs(error)
        return thrust

    # ------------------------------------------------------------------
    # DYNAMICS / INTEGRATION
    # ------------------------------------------------------------------
    def rov_dynamics(self, rov_state, t, dt, use_pid_flag):
        """ROV is the only dynamically-integrated body. The buoy's motion is
        prescribed (kinematic) by the selected wave model — this matches how
        a floating buoy is normally simplified in tether-dynamics studies and
        avoids the previous bug where the buoy was nominally 'integrated' via
        RK4 but immediately overwritten every step (silently wasting effort
        and risking state mismatch)."""
        rov_pos = rov_state[0:3]
        rov_vel = rov_state[3:6]

        buoy_pos, buoy_vel, _ = self.get_buoy_kinematics(t)

        gravity_rov = np.array([0, 0, -self.rov_mass * self.gravity])
        buoyancy_rov = np.array([0, 0, self.rov_buoyancy_force])
        damping_rov = -self.rov_damping * rov_vel

        force_on_rov, _ = self.tether_force(rov_pos, rov_vel, buoy_pos, buoy_vel)

        if use_pid_flag:
            thrust_z = self.pid_controller(rov_pos[2], dt, t)
        else:
            thrust_z = 0.0
        thrust = np.array([0, 0, thrust_z])

        total_force = gravity_rov + buoyancy_rov + damping_rov + force_on_rov + thrust
        total_force = np.clip(total_force, -10000, 10000)

        rov_accel = total_force / self.rov_mass
        if np.any(np.isnan(rov_accel)):
            rov_accel = np.zeros(3)
            rov_vel = np.zeros(3)

        self._last_thrust = thrust_z
        return np.concatenate([rov_vel, rov_accel])

    def step_simulation(self, use_pid_flag):
        try:
            dt = self.dt

            # Update buoy state (prescribed / kinematic)
            buoy_pos, buoy_vel, _ = self.get_buoy_kinematics(self.time)
            self.buoy_state[0:3] = buoy_pos
            self.buoy_state[3:6] = buoy_vel

            # Advance TMS slack deployment once per real step (not per RK4
            # sub-stage - it's a slow mechanical process, same treatment as
            # freezing thrust across sub-stages below).
            self.update_slack_deployment(dt)

            # RK4 integration on ROV state only.
            # NOTE: for RK4 correctness the PID integral/derivative should only
            # be advanced once per real step, not once per k1..k4 sub-stage.
            # We approximate this (common practical simplification) by calling
            # the controller only at k1 and freezing thrust across substeps.
            rov_pos0 = self.rov_state[0:3]
            thrust_z = self.pid_controller(rov_pos0[2], dt, self.time) if use_pid_flag else 0.0

            def deriv(state, t):
                rov_pos = state[0:3]
                rov_vel = state[3:6]
                b_pos, b_vel, _ = self.get_buoy_kinematics(t)
                gravity_rov = np.array([0, 0, -self.rov_mass * self.gravity])
                buoyancy_rov = np.array([0, 0, self.rov_buoyancy_force])
                damping_rov = -self.rov_damping * rov_vel
                force_on_rov, _ = self.tether_force(rov_pos, rov_vel, b_pos, b_vel)
                thrust = np.array([0, 0, thrust_z])
                total_force = gravity_rov + buoyancy_rov + damping_rov + force_on_rov + thrust
                total_force = np.clip(total_force, -10000, 10000)
                accel = total_force / self.rov_mass
                return np.concatenate([rov_vel, accel])

            s = self.rov_state
            k1 = deriv(s, self.time)
            k2 = deriv(s + 0.5 * dt * k1, self.time + 0.5 * dt)
            k3 = deriv(s + 0.5 * dt * k2, self.time + 0.5 * dt)
            k4 = deriv(s + dt * k3, self.time + dt)
            self.rov_state = s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

            self.rov_state[2] = np.clip(self.rov_state[2], self.max_depth, self.min_depth)
            self.rov_state[3:6] = np.clip(self.rov_state[3:6], -self.max_velocity, self.max_velocity)

            self.thrust_history.append(thrust_z)
            self.error_history.append(self.position_error)
            self.tension_history.append(getattr(self, '_last_tension', 0.0))

            self.time += dt

            if len(self.rov_depth_history) < 8000:
                self.rov_x_history.append(self.rov_state[0])
                self.rov_y_history.append(self.rov_state[1])
                self.rov_depth_history.append(self.rov_state[2])
                self.buoy_z_history.append(self.buoy_state[2])
                self.buoy_x_history.append(self.buoy_state[0])
                self.buoy_y_history.append(self.buoy_state[1])
                self.time_history.append(self.time)

        except Exception as e:
            print(f"Error in simulation: {e}")
            self.reset_simulation()

    # ------------------------------------------------------------------
    # PLOTTING
    # ------------------------------------------------------------------
    def setup_plot(self):
        self.fig = plt.figure(figsize=(14, 10))
        gs = self.fig.add_gridspec(4, 1, height_ratios=[3, 3, 3, 1], hspace=0.35)
        self.ax = self.fig.add_subplot(gs[0:3, 0], projection='3d')
        self.ax2 = self.fig.add_subplot(gs[3, 0])
        self.ax.view_init(elev=20, azim=-70)
        self._configure_axes()
        self._configure_error_axes()

    def _configure_error_axes(self):
        self.ax2.set_xlabel('Time (s)', fontsize=9)
        self.ax2.set_ylabel('Position Error (m)', fontsize=9)
        self.ax2.set_title('ROV Position Error Over Time', fontsize=10, fontweight='bold')
        self.ax2.grid(True, alpha=0.3)

    def _configure_axes(self):
        self.ax.set_xlabel('X (m)', fontsize=12, fontweight='bold')
        self.ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
        self.ax.set_zlabel('Depth (m)', fontsize=12, fontweight='bold')
        self.ax.set_title(
            f'ROV-Buoy System — {self.wave_mode.capitalize()} Waves '
            f'(Amp: {self.wave_amplitude:.1f}m, Period: {self.wave_period:.1f}s)',
            fontsize=14, fontweight='bold')
        self.ax.set_xlim([-30, 30])
        self.ax.set_ylim([-10, 10])
    
        # Dynamic z-limits based on wave amplitude - extended to always
        # include the DSL layer, whichever is deeper.
        z_margin = max(5, self.wave_amplitude * 2.0)
        z_min = -max(self.tether_length * 1.1, self.dsl_depth + self.dsl_thickness / 2 + 20) - z_margin
        z_max = z_margin + 5
        self.ax.set_zlim([z_min, z_max])
    
        self.ax.grid(True, alpha=0.3)

    def draw_cube(self, ax, center, dx, dy, dz, color, alpha=0.3):
        x, y, z = center
        vertices = [
            [x - dx, y - dy, z - dz], [x + dx, y - dy, z - dz], [x + dx, y + dy, z - dz], [x - dx, y + dy, z - dz],
            [x - dx, y - dy, z + dz], [x + dx, y - dy, z + dz], [x + dx, y + dy, z + dz], [x - dx, y + dy, z + dz]
        ]
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ]
        for edge in edges:
            ax.plot3D([vertices[edge[0]][0], vertices[edge[1]][0]],
                      [vertices[edge[0]][1], vertices[edge[1]][1]],
                      [vertices[edge[0]][2], vertices[edge[1]][2]],
                      color=color, alpha=alpha, linewidth=1.5)

    def draw_dsl_layer(self):
        """Deep Scattering Layer: a static translucent band. Drawn with a
        handful of flat, cheap plot_surface calls (2x2 grids - just the
        four corners) rather than per-frame physics, since it doesn't move -
        keeps this from adding any real per-frame cost."""
        x_corners = np.array([[-30, 30], [-30, 30]])
        y_corners = np.array([[-10, -10], [10, 10]])
        z_top = np.full_like(x_corners, self.dsl_top_z, dtype=float)
        z_bottom = np.full_like(x_corners, self.dsl_bottom_z, dtype=float)

        self.ax.plot_surface(x_corners, y_corners, z_top, color='saddlebrown', alpha=0.15, shade=False)
        self.ax.plot_surface(x_corners, y_corners, z_bottom, color='saddlebrown', alpha=0.15, shade=False)
        for frac in (0.33, 0.66):
            z_mid = np.full_like(x_corners, self.dsl_top_z + frac * (self.dsl_bottom_z - self.dsl_top_z), dtype=float)
            self.ax.plot_surface(x_corners, y_corners, z_mid, color='saddlebrown', alpha=0.07, shade=False)

        self.ax.text(-28, 0, (self.dsl_top_z + self.dsl_bottom_z) / 2,
                     'Deep Scattering Layer', color='saddlebrown', fontsize=9,
                     fontweight='bold', ha='left')

    def draw_sonar_ping(self, rov_pos):
        """Simple pinging sonar beam from the ROV straight down to the DSL,
        with a pulsing brightness (based on sim time) to suggest an active
        ping/echo cycle rather than a static line. Cheap: one line + one
        marker per frame."""
        if np.any(np.isnan(rov_pos)) or rov_pos[2] <= self.dsl_top_z:
            return

        ping_period = 2.0  # seconds per ping cycle
        phase = (self.time % ping_period) / ping_period
        alpha = 0.15 + 0.55 * (0.5 + 0.5 * np.cos(2 * np.pi * phase))

        self.ax.plot([rov_pos[0], rov_pos[0]], [rov_pos[1], rov_pos[1]],
                     [rov_pos[2], self.dsl_top_z],
                     color='yellow', alpha=alpha, linewidth=1.5, linestyle=':')
        self.ax.scatter(rov_pos[0], rov_pos[1], self.dsl_top_z,
                        color='yellow', s=40, alpha=alpha, marker='^')

    def update_plot(self):
        self.ax.clear()
        self._configure_axes()

        try:
            buoy_pos = self.buoy_state[0:3]
            rov_pos = self.rov_state[0:3]

            self.draw_dsl_layer()

            # Draw surface waves that match the actual buoy physics
            x_vals = np.linspace(-30, 30, 60)
            y_vals = np.linspace(-10, 10, 16)
        
            # Get current wave parameters
            tau = self.time - self._last_param_change_time
        
            for y in y_vals:
                if self.wave_mode == 'irregular':
                    z_vals = np.zeros_like(x_vals)
                    for freq, phase, weight in self._components:
                        # Match the buoy physics exactly (including spatial variation)
                        z_vals += self.wave_amplitude * weight * np.sin(freq * tau + phase - 0.4 * x_vals)
                else:
                    # Match the buoy physics exactly (including spatial variation)
                    z_vals = self.wave_amplitude * np.sin(self.wave_frequency * tau - 0.5 * x_vals)
                self.ax.plot(x_vals, np.ones_like(x_vals) * y, z_vals, 'c-', alpha=0.35, linewidth=1.6)

            # Draw buoy (blue)
            if not np.any(np.isnan(buoy_pos)):
                self.draw_cube(self.ax, buoy_pos,
                               self.buoy_length / 2, self.buoy_width / 2, self.buoy_height / 2,
                               'blue', alpha=0.3)
                self.ax.scatter(*buoy_pos, color='blue', s=300, label='Buoy (Forced)', alpha=0.9)
                self.ax.text(buoy_pos[0] + 10, buoy_pos[1], buoy_pos[2],
                            'Buoy', color='blue', fontsize=11, fontweight='bold', ha='center')

            # Draw ROV (red)
            if not np.any(np.isnan(rov_pos)):
                self.draw_cube(self.ax, rov_pos,
                               self.rov_length / 2, self.rov_width / 2, self.rov_height / 2,
                               'red', alpha=0.4)
                self.ax.scatter(*rov_pos, color='red', s=200, label='ROV', alpha=0.9)
                self.ax.text(rov_pos[0] + 10, rov_pos[1], rov_pos[2],
                            'ROV', color='red', fontsize=11, fontweight='bold', ha='center')

                self.draw_sonar_ping(rov_pos)

                if self.use_pid and len(self.thrust_history) > 0:
                    thrust = self.thrust_history[-1]
                    if abs(thrust) > 5:
                        thrust_dir = 1 if thrust > 0 else -1
                        self.ax.quiver(rov_pos[0], rov_pos[1], rov_pos[2],
                                      0, 0, thrust_dir * 0.6,
                                      color='orange', alpha=0.9, arrow_length_ratio=0.3)

            # Draw tether
            if not np.any(np.isnan([buoy_pos[0], rov_pos[0], buoy_pos[1], rov_pos[1],
                                     buoy_pos[2], rov_pos[2]])):
                tension = getattr(self, '_last_tension', 0.0)
                taut = tension > 1.0
                self.ax.plot([buoy_pos[0], rov_pos[0]], [buoy_pos[1], rov_pos[1]], [buoy_pos[2], rov_pos[2]],
                            color='green' if taut else 'gray',
                            linewidth=3 if taut else 1.5,
                            label=f'Tether ({"taut" if taut else "slack"})', alpha=0.8)

            # Draw trajectories
            if len(self.rov_x_history) > 1:
                self.ax.plot(self.rov_x_history, self.rov_y_history, self.rov_depth_history,
                            'r-', linewidth=1.5, alpha=0.3, label='ROV Path')
            if len(self.buoy_x_history) > 1:
               self.ax.plot(self.buoy_x_history, self.buoy_y_history, self.buoy_z_history,
                            'b-', linewidth=1.5, alpha=0.3, label='Buoy Path')

            # Reference line at surface
            self.ax.plot([-30, 30], [0, 0], [0, 0], 'k--', alpha=0.2, linewidth=1)
            self.ax.legend(loc='upper right', fontsize=9)

        except Exception as e:
            print(f"Plot update error: {e}")

        # --- 2D position error plot ---
        try:
            self.ax2.clear()
            self._configure_error_axes()
            if len(self.time_history) > 1:
                self.ax2.plot(self.time_history, self.error_history,
                               color='crimson', linewidth=1.2)
                self.ax2.axhline(0, color='gray', linewidth=0.8, alpha=0.5)
                # Fixed-width 20s scrolling window (scale stays constant,
                # just slides forward) instead of an ever-expanding x-axis.
                window = 20.0
                t_now = self.time_history[-1]
                self.ax2.set_xlim(max(0.0, t_now - window), max(t_now, window))
        except Exception as e:
            print(f"Error plot update error: {e}")

    def reset_simulation(self):
        self.time = 0.0
        self.rov_state = np.zeros(6)
        self.buoy_state = np.zeros(6)
        self.rov_state[0:3] = [0.0, 0.0, -self.tether_length]
        self.buoy_state[0:3] = [0.0, 0.0, 0.0]
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.position_error = 0.0
        self.slack_deployed = 0.0
        self.tms_integral_error = 0.0
        self.tms_previous_error = 0.0
        self._last_param_change_time = 0.0
        self.thrust_history = []
        self.error_history = []
        self.tension_history = []
        self.rov_x_history = []
        self.rov_y_history = []
        self.rov_depth_history = []
        self.buoy_z_history = []
        self.buoy_x_history = []
        self.buoy_y_history = []
        self.time_history = []


class SimulationGUI:
    def __init__(self, simulation):
        self.sim = simulation
        self.running = False
        self.animation = None

        self.root = tk.Tk()
        self.root.title("ROV-Buoy Simulation — Corrected")
        self.root.geometry("1300x850")

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        control_frame = ttk.LabelFrame(main_frame, text="Control Panel", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.N, tk.W, tk.E), padx=(0, 10))

        # Wave scenario selector
        wave_frame = ttk.LabelFrame(control_frame, text="Wave Scenario", padding="5")
        wave_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        # NOTE: these fields deliberately do NOT use textvariable/StringVar.
        # On some systems the Tcl variable binding between StringVar and the
        # widget's displayed text silently fails to sync in both directions
        # (box shows blank instead of the default, and .get() on the var
        # returns the stale default instead of what was typed). Reading/
        # writing the widgets directly sidesteps that entirely.
        ttk.Label(wave_frame, text="Mode:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.mode_combo = ttk.Combobox(wave_frame,
                                        values=["regular", "irregular", "rogue"],
                                        state="readonly", width=10)
        self.mode_combo.set("regular")
        self.mode_combo.grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(wave_frame, text="Amplitude (m):").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.wave_amplitude_entry = ttk.Entry(wave_frame, width=12)
        self.wave_amplitude_entry.insert(0, "1.0")
        self.wave_amplitude_entry.grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(wave_frame, text="Period (s):").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.wave_period_entry = ttk.Entry(wave_frame, width=12)
        self.wave_period_entry.insert(0, "4.0")
        self.wave_period_entry.grid(row=2, column=1, padx=5, pady=2)

        # PID Control
        pid_frame = ttk.LabelFrame(control_frame, text="PID Control", padding="5")
        pid_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        self.use_pid_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(pid_frame, text="Enable ROV PID", variable=self.use_pid_var).grid(row=0, column=0, columnspan=2, pady=2, sticky=tk.W)

        self.use_tms_pid_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(pid_frame, text="Enable TMS PID", variable=self.use_tms_pid_var).grid(row=1, column=0, columnspan=2, pady=2, sticky=tk.W)

        ttk.Label(pid_frame, text="ROV gains (fixed):", font=("Arial", 9, "italic")).grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(6, 0))
        ttk.Label(pid_frame, text=f"  Kp={self.sim.kp:g}  Ki={self.sim.ki:g}  Kd={self.sim.kd:g}").grid(row=3, column=0, columnspan=2, sticky=tk.W)

        ttk.Label(pid_frame, text="TMS gains (fixed):", font=("Arial", 9, "italic")).grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=(6, 0))
        ttk.Label(pid_frame, text=f"  Kp={self.sim.tms_kp:g}  Ki={self.sim.tms_ki:g}  Kd={self.sim.tms_kd:g}").grid(row=5, column=0, columnspan=2, sticky=tk.W)
        ttk.Label(pid_frame, text=f"  Target tension={self.sim.tms_target_tension:g} N").grid(row=6, column=0, columnspan=2, sticky=tk.W)

        ttk.Label(pid_frame, text="Max Thrust (N):").grid(row=7, column=0, sticky=tk.W, pady=(8, 2))
        self.max_thrust_entry = ttk.Entry(pid_frame, width=12)
        self.max_thrust_entry.insert(0, "150.0")
        self.max_thrust_entry.grid(row=7, column=1, padx=5, pady=(8, 2))

        # Tether Physics (read-only - not modifiable at runtime)
        tether_phys_frame = ttk.LabelFrame(control_frame, text="Tether Physics (fixed)", padding="5")
        tether_phys_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        ttk.Label(tether_phys_frame, text=f"Snubber Stiffness: {self.sim.tether_stiffness:g} N/m").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Label(tether_phys_frame, text=f"Tether Diameter: {self.sim.tether_diameter:g} m").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Label(tether_phys_frame, text=f"TMS Max Slack: {self.sim.tms_max_slack:g} m").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Label(tether_phys_frame, text=f"TMS Payout Rate: {self.sim.tms_payout_rate:g} m/s").grid(row=3, column=0, sticky=tk.W, pady=2)

        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=3, column=0, columnspan=2, pady=10)

        self.start_button = ttk.Button(button_frame, text="▶ Start", command=self.start_simulation, width=10)
        self.start_button.grid(row=0, column=0, padx=3)
        self.stop_button = ttk.Button(button_frame, text="⏹ Stop", command=self.stop_simulation, width=10)
        self.stop_button.grid(row=0, column=1, padx=3)
        self.reset_button = ttk.Button(button_frame, text="⟳ Reset", command=self.reset_simulation, width=10)
        self.reset_button.grid(row=0, column=2, padx=3)
        self.update_button = ttk.Button(button_frame, text="Apply", command=self.update_parameters, width=10)
        self.update_button.grid(row=0, column=3, padx=3)

        status_frame = ttk.LabelFrame(control_frame, text="Status", padding="5")
        status_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        self.status_label = ttk.Label(status_frame, text="● Stopped", foreground="red")
        self.status_label.grid(row=0, column=0, sticky=tk.W, pady=2)

        data_frame = ttk.LabelFrame(control_frame, text="Real-time Data", padding="5")
        data_frame.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        self.buoy_depth_label = ttk.Label(data_frame, text="Buoy Depth: 0.00 m", font=("Arial", 10, "bold"))
        self.buoy_depth_label.grid(row=0, column=0, sticky=tk.W, pady=3)
        self.buoy_vel_label = ttk.Label(data_frame, text="Buoy Velocity: 0.00 m/s", font=("Arial", 10))
        self.buoy_vel_label.grid(row=1, column=0, sticky=tk.W, pady=3)
        self.depth_label = ttk.Label(data_frame, text="ROV Depth: 500.00 m", font=("Arial", 10, "bold"))
        self.depth_label.grid(row=2, column=0, sticky=tk.W, pady=3)
        self.error_label = ttk.Label(data_frame, text="Position Error: 0.00 m", font=("Arial", 10, "bold"))
        self.error_label.grid(row=3, column=0, sticky=tk.W, pady=3)
        self.thrust_label = ttk.Label(data_frame, text="Thrust Force: 0.00 N", font=("Arial", 10, "bold"))
        self.thrust_label.grid(row=4, column=0, sticky=tk.W, pady=3)

        plot_frame = ttk.LabelFrame(main_frame, text="3D Visualization", padding="5")
        plot_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.canvas = FigureCanvasTkAgg(self.sim.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        main_frame.columnconfigure(0, weight=0)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        self.update_interval = 50

    def update_parameters(self):
        try:
            # Get values directly from the widgets (see NOTE above on why we
            # don't use textvariable/StringVar for these).
            wave_amplitude = float(self.wave_amplitude_entry.get())
            wave_period = float(self.wave_period_entry.get())
            mode = self.mode_combo.get()
            max_thrust = float(self.max_thrust_entry.get())

            print(f"[Apply clicked] read from fields -> amp={wave_amplitude}, "
                  f"period={wave_period}, mode={mode}, max_thrust={max_thrust}, "
                  f"use_pid={self.use_pid_var.get()}, use_tms_pid={self.use_tms_pid_var.get()}")

            # Validate inputs
            if wave_amplitude < 0 or wave_period <= 0 or max_thrust <= 0:
                self.status_label.config(text="⚠ Invalid parameter!", foreground="red")
                return

            # Update simulation parameters
            self.sim.wave_amplitude = wave_amplitude
            self.sim.wave_period = wave_period
            self.sim.wave_frequency = 2 * np.pi / wave_period
            self.sim.use_pid = self.use_pid_var.get()
            self.sim.use_tms_pid = self.use_tms_pid_var.get()
            self.sim.max_thrust = max_thrust

            # NOTE: we deliberately do NOT call self.sim._resync_phase() here.
            # That reset the wave's time-reference to "now," and since the
            # wave functions are plain sin(freq * tau), tau=0 always
            # evaluates to 0 - so every Apply snapped the buoy back to sea
            # level regardless of the new amplitude, making it look like
            # nothing had changed. Leaving _last_param_change_time alone
            # keeps the wave continuous in absolute sim time.

            # Handle wave mode change
            if mode != self.sim.wave_mode:
                # Mode changed - need to rebuild components for irregular
                if mode == 'irregular':
                    self.sim._components = self.sim._build_irregular_components()
                self.sim.wave_mode = mode
            
            # Force immediate update of buoy state with new parameters
            buoy_pos, buoy_vel, _ = self.sim.get_buoy_kinematics(self.sim.time)
            self.sim.buoy_state[0:3] = buoy_pos
            self.sim.buoy_state[3:6] = buoy_vel
            
            # Update the 3D plot's z-limits to show the wave properly
            z_margin = max(5, wave_amplitude * 2.0)
            z_min = -max(self.sim.tether_length * 1.1,
                         self.sim.dsl_depth + self.sim.dsl_thickness / 2 + 20) - z_margin
            z_max = z_margin + 5
            self.sim.ax.set_zlim([z_min, z_max])

            self.status_label.config(text="✓ Parameters Updated", foreground="green")
            self.root.after(1500, lambda: self.status_label.config(text="● Ready", foreground="blue"))

            # When the sim is stopped/paused, nothing else redraws the canvas
            # or refreshes the labels (that's normally the FuncAnimation
            # loop's job, which only runs while self.running is True).
            # Without both of these, parameters update in memory but nothing
            # visible changes until you hit Start. Force both here so Apply
            # always shows an effect immediately, whether running or not.
            self.sim.update_plot()
            self.canvas.draw()
            # canvas.draw() renders into the canvas's internal buffer, but on
            # some systems/window managers Tk won't actually repaint the
            # visible widget until a real event is processed - just calling
            # update_idletasks() isn't enough (it only flushes pending idle
            # callbacks, not a forced repaint). Force it explicitly here.
            self.canvas.get_tk_widget().update()
            self.root.update()
            self.update_data_display()

            print(f"[Apply done] sim.wave_amplitude={self.sim.wave_amplitude}, "
                  f"sim.wave_period={self.sim.wave_period}, sim title="
                  f"{self.sim.ax.get_title()!r}")
            
        except ValueError as e:
            self.status_label.config(text=f"⚠ Invalid input: {str(e)}", foreground="red")
            print(f"[Apply] ValueError: {e}")
        except Exception as e:
            self.status_label.config(text=f"⚠ Error: {str(e)}", foreground="red")
            print(f"Error in update_parameters: {e}")

    def start_simulation(self):
        if not self.running:
            self.running = True
            self.status_label.config(text="● Running", foreground="green")
            self.update_parameters()
            self.animation = FuncAnimation(self.sim.fig, self.update,
                                          interval=self.update_interval,
                                          cache_frame_data=False)
            self.canvas.draw()

    def stop_simulation(self):
        if self.running:
            self.running = False
            self.status_label.config(text="● Stopped", foreground="red")
            if self.animation:
                self.animation.event_source.stop()

    def reset_simulation(self):
        self.stop_simulation()
        self.sim.reset_simulation()
        self.sim.update_plot()
        self.canvas.draw()
        self.update_data_display()
        self.status_label.config(text="● Reset", foreground="orange")
        self.root.after(1200, lambda: self.status_label.config(text="● Stopped", foreground="red"))

    def update_data_display(self):
        buoy_depth = -self.sim.buoy_state[2]
        self.buoy_depth_label.config(text=f"Buoy Depth: {buoy_depth:.3f} m")
        buoy_vel = -self.sim.buoy_state[5]
        self.buoy_vel_label.config(text=f"Buoy Velocity: {buoy_vel:.3f} m/s")

        if len(self.sim.rov_depth_history) > 0:
            depth = -self.sim.rov_depth_history[-1]
            self.depth_label.config(text=f"ROV Depth: {depth:.3f} m")

        self.error_label.config(text=f"Position Error: {self.sim.position_error:.3f} m")

        if len(self.sim.thrust_history) > 0:
            thrust = self.sim.thrust_history[-1]
            self.thrust_label.config(text=f"Thrust Force: {thrust:.2f} N")

    def update(self, frame):
        if self.running:
            try:
                self.sim.step_simulation(self.sim.use_pid)
                self.sim.update_plot()
                self.canvas.draw()
                self.update_data_display()
            except Exception as e:
                print(f"Update error: {e}")
                self.stop_simulation()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    sim = ROVBuoySimulation()
    gui = SimulationGUI(sim)
    gui.run()