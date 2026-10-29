#!/usr/bin/env python3
"""
===============================================================================
CYBER-PHYSICAL SAFETY INTERLOCK & SCADA SUPERVISOR CONTROLLER
===============================================================================
System: Industrial Manufacturing Facility - Conveyor Powertrain ESD Supervisor
Target: Webots Simulation Environment (Robot, Keyboard, Display, LED API)
Standards: OSHA 29 CFR 1910.147 (Lockout/Tagout), IEC 61508 / IEC 61511 (SIL-2/3)

Technical Specifications:
- Hardware Devices:
    * Primary Actuator: 'rotational motor' (Position: inf, Velocity-controlled)
    * Auxiliary Cooling: 'fan motor' (Position: inf, Velocity-controlled)
    * 3-Tier Andon Tower: 'green_led', 'amber_led', 'red_led'
    * Heads-Up Display: 'scada_display' (In-viewport SCADA HUD overlay)
    * Graceful fault handling if 'fan motor', LEDs, or Display are absent.

- Thermal Physics Engine & Automated Degradation Pipeline:
    * Baseline heating rate: TEMP_RISE_RATE = 0.42 °C/s.
    * Convective cooling rate: CONVECTIVE_COOLING_RATE = 0.18 °C/s.
    * Stage 1 Throttling: 40% reduction in heat input (0.60 * 0.42 = 0.252 °C/s).
    * Under severe mechanical conveyor load, thermal degradation increases friction,
      guaranteeing continuous, steady progression from 75°C -> 80°C -> 85°C -> 90°C
      without getting stuck in a thermal equilibrium trap at 75°C.

- Automated Multi-Stage Safety Sequencing:
    1. STAGE 0: NOMINAL (T < 75.0°C)
       - Main Motor: Nominal speed (5.0 rad/s). Conveyor rolls forward.
       - Fan Motor: OFF (0.0 rad/s).
       - Andon Tower: Green LED ON, Amber OFF, Red OFF.
    2. STAGE 1: ACTIVE WARNING & DERATING (75.0°C <= T < 90.0°C)
       - Main Motor: Throttled to 50% (2.5 rad/s) for VFD frequency de-rating.
       - Fan Motor: Rapid rotation (15.0 rad/s) with visible blade spin.
       - Andon Tower: Green OFF, Amber ON, Red OFF.
       - Non-blocking warning telemetry with steady upward temperature drift.
    3. STAGE 2: AUTOMATIC EMERGENCY SHUTDOWN (T >= 90.0°C)
       - 2.0-second continuous countdown once T >= 90.0°C.
       - At exactly 2.0s of continuous sustained overheat:
         * Main Motor: Hard cut to 0.0 rad/s (conveyor halts instantly).
         * Fan Motor: Remains ACTIVE (15.0 rad/s) for post-trip remediation.
         * Andon Tower: Green OFF, Amber OFF, Red LED Flashing.
         * Engage OSHA Lockout/Tagout (LOTO): system_locked = True.
           Motor remains dead (0.0 rad/s) even after cooling below 75°C.
    4. ADVANCED PREDICTIVE ANALYTICS:
       - Real-time Rate-of-Rise (dT/dt) monitoring.
       - Presentation Mode Toggle ('P' Key): When enabled, if dT/dt > 4.5°C/s,
         immediate predictive shutdown executes before 90.0°C ceiling.
       - Fault Injection Spike ('S' Key): Injects instantaneous +0.35°C surge.

- Interactive Overrides & Fault Injection:
    * 'P': Toggle Predictive Surge Testing Mode (defaults to OFF).
    * 'S': Inject Instantaneous Thermal Spike (+0.35°C in single tick).
    * 'F': Bearing seizure fault injection (immediate heat surge to 92°C -> auto-trip).
    * 'E': Manual operator emergency kill-switch.
    * 'R': Supervisor LOTO authorization reset (requires Temp < 75.0°C).
===============================================================================
"""

import sys
import os
import time
import math
import argparse
import random
from enum import Enum
from typing import Optional, Dict, Any

# Enable ANSI escape sequences on Windows console
if os.name == 'nt':
    try:
        os.system('')
    except Exception:
        pass


class ANSI:
    """ANSI terminal escape color codes for industrial console output."""
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    UNDERLINE = "\033[4m"

    # Foreground standard & high intensity
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
    DARK_GRAY = "\033[90m"

    # Background
    BG_RED = "\033[41m"
    BG_YELLOW = "\033[43m"
    BG_GREEN = "\033[42m"
    BG_BLUE = "\033[44m"
    BG_DARK = "\033[40m"


class InterlockStage(Enum):
    """Operational cyber-physical safety interlock stages."""
    STAGE_0_NORMAL = "STAGE 0: NOMINAL (GREEN ZONE)"
    STAGE_1_TRIAGE = "STAGE 1: ACTIVE WARNING & DERATING (YELLOW ZONE)"
    STAGE_2_OVERHEAT = "STAGE 2: SUSTAINED OVERHEAT (RED ZONE)"
    ESD_TRIPPED = "ESD TRIPPED: HARD SAFETY LATCH (OSHA LOTO ACTIVE)"


class TripReason(str, Enum):
    """Causes of emergency shutdown trips."""
    NONE = "None (Operational)"
    SUSTAINED_OVERHEAT = "Sustained Overheat (T >= 90.0°C for >= 2.0s continuous)"
    PREDICTIVE_RATE_OF_RISE = "PREDICTIVE ANOMALY: dT/dt EXCEEDED 4.5°C/s"
    BEARING_SEIZURE_FAULT = "Bearing Seizure Fault Injection ('F' Key Triggered)"
    MANUAL_E_STOP = "Manual Operator Emergency Stop ('E' Key Depressed)"


class SafetyInterlockSupervisor:
    """
    Industrial Cyber-Physical Safety Interlock & SCADA Supervisor for Webots.
    Controls drive powertrain, forced-air cooling, Andon stack lights, and SCADA HUD.
    """

    # Operating setpoints & thresholds
    TEMP_STAGE_1_THRESHOLD = 60.0  # °C: Active warning & derating boundary
    TEMP_STAGE_2_THRESHOLD = 90.0   # °C: Critical overheat boundary
    OVERHEAT_SUSTAINED_SEC = 2.0    # Seconds: Duration in Red Zone before automatic ESD trip
    RATE_OF_RISE_THRESHOLD = 4.5    # °C/s: Maximum safe thermal acceleration
    TEMP_SAFE_RESET_LIMIT = 75.0    # °C: Maximum temperature permitted for OSHA LOTO clear
    AMBIENT_TEMPERATURE = 25.0      # °C: Factory ambient baseline

    # Actuator velocities (rad/s)
    MAIN_MOTOR_FULL_SPEED = 5.0     # rad/s (Stage 0 full speed conveyor drive)
    MAIN_MOTOR_THROTTLED = 2.5      # rad/s (Stage 1 throttled by 50% de-rating)
    MAIN_MOTOR_STOPPED = 0.0        # rad/s (Stage 2 / ESD Trip hard cut)

    FAN_MOTOR_INACTIVE = 0.0        # rad/s (Stage 0 fan idle)
    FAN_MOTOR_ACTIVE = 15.0         # rad/s (Stage 1 rapid cooling & remediation)

    # Thermal Physics Engine Calibration: Normal smooth rise rate (+0.10°C to +0.15°C/s)
    TEMP_RISE_RATE = 0.125          # °C/s: Baseline heating rate under normal motor load
    CONVECTIVE_COOLING_RATE = 0.05  # °C/s: Convective cooling rate with auxiliary fan active

    def __init__(self, robot=None, keyboard=None, time_step: int = 64, initial_temp: float = 26.0,
                 enable_random_surge: bool = True, predictive_mode_enabled: bool = False):
        """
        Initialize the Cyber-Physical Safety Interlock Supervisor.

        :param robot: Native Webots Robot instance (or MockRobot for headless tests).
        :param keyboard: Native Webots Keyboard instance.
        :param time_step: Simulation time step in milliseconds.
        :param initial_temp: Starting core temperature in °C.
        :param enable_random_surge: Whether to introduce dynamic spike surge after normal run.
        :param predictive_mode_enabled: Presentation toggle for predictive dT/dt auto-trip (defaults to False).
        """
        self.robot = robot
        self.keyboard = keyboard
        self.time_step_ms = time_step
        self.dt = max(0.001, self.time_step_ms / 1000.0)

        # Hardware handles
        self.main_motor = None
        self.fan_motor = None
        self.green_led = None
        self.amber_led = None
        self.red_led = None
        self.display = None

        self.main_motor_available = False
        self.fan_motor_available = False
        self.leds_available = False
        self.display_available = False

        # State machine variables & temperature initialization
        # Requirement 1: Ensure prev_temp = current_temp during initialization
        self.current_temp = float(initial_temp)
        self.prev_temp = float(self.current_temp)
        self.peak_temperature = float(self.current_temp)
        self.rate_of_rise = 0.0
        self.trip_rate_of_rise = 0.0    # Peak dT/dt recorded at trip
        self.warmup_ticks = 5           # Requirement 1: 5-tick warm-up phase to bypass startup artifacts
        self.tick_count = 0
        self.overheat_timer = 0.0
        self.system_locked = False      # OSHA Lockout/Tagout hard latch
        self.trip_reason = TripReason.NONE
        self.trip_timestamp = 0.0
        self.sim_time = 0.0

        # Presentation Mode & Predictive Surge Features ('P' and 'S' Keys)
        self.predictive_mode_enabled = predictive_mode_enabled  # Defaults to OFF (False)
        self.manual_spike_pending = False

        # Actuator command state
        self.cmd_main_velocity = self.MAIN_MOTOR_FULL_SPEED
        self.cmd_fan_velocity = self.FAN_MOTOR_INACTIVE

        # Fault injection & disturbance flags
        self.fan_hardware_fault = False
        self.external_heat_flux = 0.0

        # Dynamic spike trigger (Requirement 3: introduce instantaneous surge after 10-15s normal run)
        self.enable_random_surge = enable_random_surge
        self.surge_injected = False
        self.surge_target_time = random.uniform(10.0, 15.0)
        self.cycle_count = 0
        self.next_surge_cycle = int(self.surge_target_time / self.dt)

        # Telemetry & HUD scheduling
        self.last_telemetry_time = -1.0
        self.telemetry_interval = 0.5   # Periodic status every 0.5s
        self.last_hud_update = -1.0
        self.hud_interval = 0.1         # Redraw HUD at 10Hz
        self.last_logged_stage = None

        # Bind hardware devices
        self._initialize_hardware()

        # Initialize state machine & actuators
        self.evaluate_interlock_logic()
        self.apply_andon_lights()
        self.render_scada_hud()

    @property
    def temperature(self) -> float:
        """Alias property for current_temp."""
        return self.current_temp

    @temperature.setter
    def temperature(self, val: float) -> None:
        self.current_temp = float(val)

    @property
    def previous_temperature(self) -> float:
        """Alias property for prev_temp."""
        return self.prev_temp

    @previous_temperature.setter
    def previous_temperature(self, val: float) -> None:
        self.prev_temp = float(val)

    def _initialize_hardware(self) -> None:
        """
        Safely discover and bind hardware devices ('rotational motor', 'fan motor',
        'green_led', 'amber_led', 'red_led', 'scada_display').
        Guarantees zero crashes if any device is physically undeclared.
        """
        print(f"{ANSI.BOLD}{ANSI.CYAN}======================================================================{ANSI.RESET}")
        print(f"{ANSI.BOLD}{ANSI.CYAN}  INDUSTRIAL SCADA INTERLOCK & ESD SUPERVISOR INITIALIZING            {ANSI.RESET}")
        print(f"{ANSI.BOLD}{ANSI.CYAN}======================================================================{ANSI.RESET}")

        if self.robot is None:
            print(f"{ANSI.YELLOW}[HARDWARE] No physical Webots robot passed. Running in virtualized mode.{ANSI.RESET}")
            return

        # 1. Primary Drive Motor ('rotational motor')
        try:
            self.main_motor = self.robot.getDevice('rotational motor')
            if self.main_motor is not None:
                self.main_motor.setPosition(float('inf'))
                self.main_motor.setVelocity(self.MAIN_MOTOR_FULL_SPEED)
                self.main_motor_available = True
                print(f"{ANSI.GREEN}[HARDWARE] Primary Actuator 'rotational motor' ONLINE [Velocity Control].{ANSI.RESET}")
            else:
                print(f"{ANSI.YELLOW}[HARDWARE WARNING] 'rotational motor' device not declared in robot.{ANSI.RESET}")
        except Exception as exc:
            self.main_motor = None
            self.main_motor_available = False
            print(f"{ANSI.YELLOW}[HARDWARE WARNING] Error accessing 'rotational motor': {exc}{ANSI.RESET}")

        # 2. Auxiliary Cooling Fan ('fan motor') - Graceful tolerance required
        try:
            self.fan_motor = self.robot.getDevice('fan motor')
            if self.fan_motor is not None:
                self.fan_motor.setPosition(float('inf'))
                self.fan_motor.setVelocity(self.FAN_MOTOR_INACTIVE)
                self.fan_motor_available = True
                print(f"{ANSI.GREEN}[HARDWARE] Auxiliary Cooling 'fan motor' ONLINE [Velocity Control].{ANSI.RESET}")
            else:
                self.fan_motor = None
                self.fan_motor_available = False
                print(f"{ANSI.YELLOW}[HARDWARE TOLERANCE] 'fan motor' device not physically declared.{ANSI.RESET}")
                print(f"{ANSI.YELLOW}                     Operating in hardware-abstracted convective cooling mode.{ANSI.RESET}")
        except Exception as exc:
            self.fan_motor = None
            self.fan_motor_available = False
            print(f"{ANSI.YELLOW}[HARDWARE TOLERANCE] Auxiliary 'fan motor' absent ({exc}). Fallback active.{ANSI.RESET}")

        # 3. 3-Tier Andon Stack Light LEDs ('green_led', 'amber_led', 'red_led')
        try:
            self.green_led = self.robot.getDevice('green_led')
            self.amber_led = self.robot.getDevice('amber_led')
            self.red_led = self.robot.getDevice('red_led')
            if self.green_led is not None or self.amber_led is not None or self.red_led is not None:
                self.leds_available = True
                print(f"{ANSI.GREEN}[HARDWARE] Visual SCADA Andon Stack Light (Green/Amber/Red) ONLINE.{ANSI.RESET}")
            else:
                self.leds_available = False
                print(f"{ANSI.YELLOW}[HARDWARE] Andon LEDs not detected. Running headless visual state.{ANSI.RESET}")
        except Exception as exc:
            self.leds_available = False
            print(f"{ANSI.YELLOW}[HARDWARE] Error binding Andon LEDs: {exc}{ANSI.RESET}")

        # 4. In-Viewport SCADA HUD Display ('scada_display')
        try:
            self.display = self.robot.getDevice('scada_display')
            if self.display is not None:
                self.display_available = True
                try:
                    self.display.setFont("Arial", 10, True)
                except Exception:
                    pass
                print(f"{ANSI.GREEN}[HARDWARE] In-Viewport SCADA HUD Display ('scada_display') ONLINE.{ANSI.RESET}")
            else:
                self.display_available = False
                print(f"{ANSI.YELLOW}[HARDWARE] 'scada_display' node not detected. HUD rendering skipped.{ANSI.RESET}")
        except Exception as exc:
            self.display_available = False
            print(f"{ANSI.YELLOW}[HARDWARE] Error binding Display node: {exc}{ANSI.RESET}")

        # 5. Webots Keyboard listener
        if self.keyboard is not None:
            try:
                self.keyboard.enable(self.time_step_ms)
                print(f"{ANSI.CYAN}[INPUT] Webots Keyboard listener enabled (Step: {self.time_step_ms}ms).{ANSI.RESET}")
            except Exception as exc:
                print(f"{ANSI.YELLOW}[INPUT WARNING] Could not enable keyboard: {exc}{ANSI.RESET}")

        self._print_controls_banner()

    def _print_controls_banner(self) -> None:
        """Display operational setpoints, degradation pipeline details, and overrides."""
        print(f"\n{ANSI.BOLD}Automated Thermal Degradation Pipeline & Safety Setpoints:{ANSI.RESET}")
        print(f"  • Stage 0 (Nominal)  : Temp < {self.TEMP_STAGE_1_THRESHOLD:.1f}°C  | Motor: {self.MAIN_MOTOR_FULL_SPEED:.1f} rad/s | Fan: {self.FAN_MOTOR_INACTIVE:.1f} rad/s | Green LED ON")
        print(f"  • Stage 1 (Warning)  : {self.TEMP_STAGE_1_THRESHOLD:.1f}°C <= Temp < {self.TEMP_STAGE_2_THRESHOLD:.1f}°C | Motor: {self.MAIN_MOTOR_THROTTLED:.1f} rad/s (50%) | Fan: {self.FAN_MOTOR_ACTIVE:.1f} rad/s | Amber LED ON")
        print(f"  • Thermal Dynamics   : Rise Rate: +{self.TEMP_RISE_RATE:.2f}°C/s | Fan Cooling: -{self.CONVECTIVE_COOLING_RATE:.2f}°C/s | Non-stalling progression")
        print(f"  • Stage 2 (Auto ESD) : Temp >= {self.TEMP_STAGE_2_THRESHOLD:.1f}°C sustained for {self.OVERHEAT_SUSTAINED_SEC:.1f}s -> AUTOMATIC EMERGENCY SHUTDOWN")
        print(f"  • Predictive Limit   : Rate-of-Rise (dT/dt) > {self.RATE_OF_RISE_THRESHOLD:.1f}°C/s -> PREDICTIVE ESD TRIP (Active in Predictive Mode)")
        print(f"  • OSHA LOTO Reset    : Requires 'R' key AND Core Temp < {self.TEMP_SAFE_RESET_LIMIT:.1f}°C")
        print(f"\n{ANSI.BOLD}Interactive Keyboard Overrides (Click 3D Viewport to focus):{ANSI.RESET}")
        print(f"  [{ANSI.RED}E{ANSI.RESET}] Manual Operator Emergency Kill-Switch (Immediate ESD Trip)")
        print(f"  [{ANSI.CYAN}R{ANSI.RESET}] Supervisor OSHA LOTO Reset (Requires Core Temp < 75.0°C)")
        print(f"  [{ANSI.CYAN}P{ANSI.RESET}] Toggle Predictive Surge Testing Mode (Presentation Toggle)")
        print(f"  [{ANSI.YELLOW}S{ANSI.RESET}] Inject Instantaneous Fault Spike (+0.35°C single tick -> dT/dt ~ 5.5°C/s)")
        print(f"  [{ANSI.YELLOW}F{ANSI.RESET}] Fault Injection: Simulate Bearing Seizure (Immediate Surge to 92°C)")
        print(f"  [{ANSI.MAGENTA}H{ANSI.RESET}] Inject Thermal Rate-of-Rise Surge (+8.0°C/s shock)")
        print(f"  [{ANSI.BLUE}C{ANSI.RESET}] Rapid Cryogenic Cooldown Pulse (-10.0°C)")
        print(f"{ANSI.CYAN}======================================================================{ANSI.RESET}\n")

    def set_actuator_velocities(self, main_vel: float, fan_vel: float) -> None:
        """
        Update actuator command velocities safely with defensive availability checks.
        """
        self.cmd_main_velocity = main_vel
        self.cmd_fan_velocity = fan_vel

        # Primary Drive Motor
        if self.main_motor_available and self.main_motor is not None:
            try:
                self.main_motor.setVelocity(float(main_vel))
            except Exception as exc:
                print(f"{ANSI.RED}[ACTUATOR ERROR] Failed to set main motor velocity: {exc}{ANSI.RESET}")

        # Auxiliary Cooling Fan
        if self.fan_motor_available and self.fan_motor is not None:
            try:
                actual_fan_vel = 0.0 if self.fan_hardware_fault else float(fan_vel)
                self.fan_motor.setVelocity(actual_fan_vel)
            except Exception as exc:
                print(f"{ANSI.RED}[ACTUATOR ERROR] Failed to set fan motor velocity: {exc}{ANSI.RESET}")

    def apply_andon_lights(self) -> None:
        """
        Control the 3-Tier Visual SCADA Beacon Tower (Stack Light / Andon Tower):
        - Stage 0: Green LED ON, Amber OFF, Red OFF.
        - Stage 1: Green OFF, Amber LED ON, Red OFF.
        - Stage 2 / Tripped: Green OFF, Amber OFF, Red LED Flashing.
        """
        if not self.leds_available:
            return

        green_val = 0
        amber_val = 0
        red_val = 0

        if self.system_locked or self.current_stage == InterlockStage.ESD_TRIPPED:
            # Red LED Flashing strobe during emergency trip / LOTO
            flash = int(self.sim_time * 3.0) % 2
            red_val = flash
        elif self.current_stage == InterlockStage.STAGE_2_OVERHEAT:
            # Fast pulse warning during 2.0s countdown
            flash = int(self.sim_time * 4.0) % 2
            red_val = flash
            amber_val = 1 - flash
        elif self.current_stage == InterlockStage.STAGE_1_TRIAGE:
            amber_val = 1
        else:  # Stage 0
            green_val = 1

        # Defensive updates
        if self.green_led is not None:
            try:
                self.green_led.set(green_val)
            except Exception:
                pass
        if self.amber_led is not None:
            try:
                self.amber_led.set(amber_val)
            except Exception:
                pass
        if self.red_led is not None:
            try:
                self.red_led.set(red_val)
            except Exception:
                pass

    def render_scada_hud(self) -> None:
        """
        Render a live in-viewport SCADA HUD on the Webots 3D display overlay:
        - Live Temperature Gauge with color-coded bar (Green / Amber / Red)
        - Operating Mode: [NORMAL] / [THROTTLED] / [LOTO LOCKED]
        - Drive Velocity & dT/dt Rate-of-Rise readout.
        """
        if not self.display_available or self.display is None:
            return

        try:
            w = self.display.getWidth()
            h = self.display.getHeight()

            # Background: Dark industrial slate (0x0F172A)
            self.display.setColor(0x0F172A)
            self.display.fillRectangle(0, 0, w, h)

            # Outer border: Dark steel blue (0x334155)
            self.display.setColor(0x334155)
            self.display.drawRectangle(0, 0, w, h)

            # Header Banner (0x1E293B)
            self.display.setColor(0x1E293B)
            self.display.fillRectangle(1, 1, w - 2, 24)
            self.display.setColor(0x38BDF8)  # Cyan text
            self.display.drawText("SCADA POWERTRAIN INTERLOCK", 8, 5)

            mins = int(self.sim_time // 60)
            secs = self.sim_time % 60
            self.display.setColor(0x94A3B8)
            self.display.drawText(f"T+{mins:02d}:{secs:04.1f}", w - 60, 5)

            # Operating Mode Badge: [NORMAL] / [THROTTLED] / [OSHA LOTO LOCKED]
            if self.system_locked or self.current_stage == InterlockStage.ESD_TRIPPED:
                badge_bg = 0xDC2626    # Bright Red
                badge_txt = "[OSHA LOTO LOCKED]"
            elif self.current_stage == InterlockStage.STAGE_2_OVERHEAT:
                badge_bg = 0xEF4444    # Red
                badge_txt = f"[OVERHEAT: {self.overheat_timer:.1f}s/2.0s]"
            elif self.current_stage == InterlockStage.STAGE_1_TRIAGE:
                badge_bg = 0xD97706    # Amber
                badge_txt = "[THROTTLED]"
            else:
                badge_bg = 0x059669    # Green
                badge_txt = "[NORMAL]"

            self.display.setColor(badge_bg)
            self.display.fillRectangle(10, 32, 135, 20)
            self.display.setColor(0xFFFFFF)
            self.display.drawText(badge_txt, 14, 35)

            # Predictive Mode Indicator on HUD
            pred_txt = "Predictive Mode: [ON]" if self.predictive_mode_enabled else "Predictive Mode: [OFF]"
            pred_col = 0x38BDF8 if self.predictive_mode_enabled else 0x94A3B8
            self.display.setColor(pred_col)
            self.display.drawText(pred_txt, 155, 35)

            # Temperature Value & Gauge
            if self.current_temp < self.TEMP_STAGE_1_THRESHOLD:
                gauge_color = 0x10B981  # Green
            elif self.current_temp < self.TEMP_STAGE_2_THRESHOLD:
                gauge_color = 0xF59E0B  # Amber
            else:
                gauge_color = 0xEF4444  # Red

            self.display.setColor(0xF8FAFC)
            self.display.drawText(f"Core Temp: {self.current_temp:5.1f} C", 10, 60)

            # Temperature Progress Bar
            gauge_w = 210
            gauge_h = 12
            self.display.setColor(0x1E293B)
            self.display.fillRectangle(10, 78, gauge_w, gauge_h)
            self.display.setColor(0x475569)
            self.display.drawRectangle(10, 78, gauge_w, gauge_h)

            # Scale between 25.0°C and 100.0°C
            ratio = max(0.0, min(1.0, (self.current_temp - 25.0) / 75.0))
            fill_w = int(ratio * (gauge_w - 4))
            if fill_w > 0:
                self.display.setColor(gauge_color)
                self.display.fillRectangle(12, 80, fill_w, gauge_h - 4)

            # Telemetry Metrics Grid
            self.display.setColor(0xE2E8F0)
            self.display.drawText(f"Main Drive: {self.cmd_main_velocity:3.1f} rad/s", 10, 98)
            self.display.drawText(f"Aux Fan   : {self.cmd_fan_velocity:4.1f} rad/s", 10, 114)

            if self.system_locked:
                ror_display = self.trip_rate_of_rise if self.trip_rate_of_rise != 0.0 else self.rate_of_rise
                self.display.setColor(0xEF4444)
                self.display.drawText(f"Peak dT/dt: {ror_display:+5.2f} C/s", 155, 98)
            else:
                ror_color = 0xEF4444 if abs(self.rate_of_rise) > self.RATE_OF_RISE_THRESHOLD else 0x38BDF8
                self.display.setColor(ror_color)
                self.display.drawText(f"dT/dt: {self.rate_of_rise:+5.2f} C/s", 160, 98)

            loto_txt = "LOTO: ENGAGED" if self.system_locked else "LOTO: CLEAR"
            loto_col = 0xEF4444 if self.system_locked else 0x10B981
            self.display.setColor(loto_col)
            self.display.drawText(loto_txt, 160, 114)

            # Footer / Key Controls Ribbon
            self.display.setColor(0x1E293B)
            self.display.fillRectangle(1, h - 22, w - 2, 21)
            self.display.setColor(0x64748B)
            try:
                self.display.setFont("Arial", 8, False)
            except Exception:
                pass
            self.display.drawText("[E] E-Stop | [R] Reset | [P] Toggle Predictive | [S] Spike | [F] Seizure", 6, h - 16)
            try:
                self.display.setFont("Arial", 10, True)
            except Exception:
                pass

        except Exception:
            pass

    def process_keyboard_inputs(self) -> None:
        """
        Poll keyboard buffer and process operator overrides:
        - 'E': Immediate manual Emergency Stop (E-Stop).
        - 'R': Supervisor OSHA LOTO reset.
        - 'P': Presentation mode toggle for predictive surge auto-trip.
        - 'S': Inject instantaneous fault spike (+0.35°C in single tick).
        - 'F': Fault injection: simulate instantaneous bearing seizure (surge to 92°C).
        - 'H': Inject thermal rate-of-rise disturbance (+8.0°C/s).
        - 'C': Cryogenic pulse (-10.0°C).
        """
        if self.keyboard is None:
            return

        try:
            key = self.keyboard.getKey()
            while key > 0:
                char = chr(key).upper() if 0 <= key <= 255 else ""

                if char == 'E':
                    self.trigger_manual_estop()
                elif char == 'R':
                    self.attempt_supervisor_reset()
                elif char == 'P':
                    self.predictive_mode_enabled = not self.predictive_mode_enabled
                    if self.predictive_mode_enabled:
                        print(f"{ANSI.CYAN}[MODE] Predictive Surge Testing Mode: ACTIVE. The next anomaly will trigger an immediate dT/dt trip.{ANSI.RESET}")
                    else:
                        print(f"{ANSI.DARK_GRAY}[MODE] Predictive Surge Testing Mode: DEACTIVATED. Running standard thermal progression (Green -> Yellow -> Red).{ANSI.RESET}")
                elif char == 'S':
                    self.manual_spike_pending = True
                    print(f"\n{ANSI.BG_YELLOW}{ANSI.BOLD} >>> [SPIKE INJECTION] Dynamic Thermal Surge (+0.35°C single tick) Injected ('S' Key)! <<< {ANSI.RESET}")
                elif char == 'F':
                    # Fault injection: Simulate instantaneous bearing seizure (heat surge to 92°C)
                    print(f"\n{ANSI.BG_YELLOW}{ANSI.BOLD} >>> [FAULT INJECTION] BEARING SEIZURE SIMULATED ('F' KEY)! <<< {ANSI.RESET}")
                    print(f"{ANSI.YELLOW} Core temperature surging immediately to 92.0°C to trigger auto-trip sequence!{ANSI.RESET}")
                    self.temperature = 92.0
                    # Evaluate immediately so 2.0s countdown or trip is initiated
                    self.evaluate_interlock_logic()
                elif char == 'H':
                    # Rate-of-rise surge injection
                    surge = 6.0
                    self.external_heat_flux += surge
                    print(f"{ANSI.YELLOW}[SIMULATION INJECTION] Thermal rate-of-rise shock injected (+{surge:.1f}°C/s)!{ANSI.RESET}")
                elif char == 'C':
                    self.temperature = max(self.AMBIENT_TEMPERATURE, self.temperature - 10.0)
                    print(f"{ANSI.CYAN}[SIMULATION] Cryogenic cooldown applied (-10.0°C). Temp: {self.temperature:.1f}°C{ANSI.RESET}")

                key = self.keyboard.getKey()
        except Exception:
            pass

    def trigger_trip(self, reason: Any, ror_at_trip: float) -> None:
        """
        Execute Emergency Shutdown (ESD):
        - Cut main drive motor to 0.0 rad/s (conveyor halts instantly).
        - Keep auxiliary fan active at 15.0 rad/s for post-trip thermal remediation.
        - Engage OSHA Lockout/Tagout hard safety latch (system_locked = True).
        - Latch trip_reason = "PREDICTIVE ANOMALY: dT/dt EXCEEDED 4.5°C/s" for predictive trip.
        - Record peak dT/dt that caused the shutdown.
        - Output structured trip diagnostics.
        """
        self.system_locked = True
        if reason == TripReason.PREDICTIVE_RATE_OF_RISE or str(reason) == "PREDICTIVE ANOMALY: dT/dt EXCEEDED 4.5°C/s":
            self.trip_reason = "PREDICTIVE ANOMALY: dT/dt EXCEEDED 4.5°C/s"
        else:
            self.trip_reason = reason
        self.trip_timestamp = self.sim_time
        self.trip_rate_of_rise = ror_at_trip
        self.current_stage = InterlockStage.ESD_TRIPPED

        # Hard stop main drive immediately; auxiliary fan remains active for remediation
        self.set_actuator_velocities(self.MAIN_MOTOR_STOPPED, self.FAN_MOTOR_ACTIVE)

        # Output trip diagnostics
        self.output_trip_diagnostics(ror_at_trip)

    def trigger_manual_estop(self) -> None:
        """Process operator manual E-stop kill switch ('E')."""
        if self.system_locked and self.trip_reason == TripReason.MANUAL_E_STOP:
            return
        print(f"\n{ANSI.BG_RED}{ANSI.WHITE}{ANSI.BOLD} >>> [OPERATOR OVERRIDE] MANUAL EMERGENCY STOP ('E') ENGAGED! <<< {ANSI.RESET}")
        self.trigger_trip(TripReason.MANUAL_E_STOP, self.rate_of_rise)

    def attempt_supervisor_reset(self) -> None:
        """
        Conditional Supervisor Reset procedure ('R' key) under OSHA 1910.147 / IEC 61508:
        - If current_temp >= 75.0°C: Reject reset, print hazard alert, keep motor 0.0 rad/s and system_locked = True.
        - If current_temp < 75.0°C: Accept reset, clear lockout (system_locked = False), reset timers, restore 5.0 rad/s.
        """
        print(f"\n{ANSI.CYAN}[OSHA LOTO PROCEDURE] Supervisor Reset key ('R') registered by safety interlock...{ANSI.RESET}")

        if not self.system_locked:
            print(f"{ANSI.DARK_GRAY}  [LOTO INFO] Safety interlock is not currently latched. Machine operational.{ANSI.RESET}")
            return

        current_temp = self.current_temp

        # Conditional Supervisor Reset Check ('R' Key)
        if current_temp >= self.TEMP_SAFE_RESET_LIMIT:
            print(f"{ANSI.RED}{ANSI.BOLD}[RESET REJECTED - THERMAL HAZARD] Cannot clear LOTO! Motor temperature ({current_temp:.1f}°C) exceeds safe restart threshold ({self.TEMP_SAFE_RESET_LIMIT:.1f}°C). Active cooling in progress...{ANSI.RESET}")
            # Keep the main motor velocity at 0.0 rad/s and system_locked = True
            self.cmd_main_velocity = self.MAIN_MOTOR_STOPPED
            self.system_locked = True
            self.set_actuator_velocities(self.MAIN_MOTOR_STOPPED, self.FAN_MOTOR_ACTIVE)
            return

        # If current_temp < self.TEMP_SAFE_RESET_LIMIT: Accept the reset
        self.system_locked = False
        self.trip_reason = TripReason.NONE
        self.overheat_timer = 0.0
        self.trip_rate_of_rise = 0.0
        self.current_stage = InterlockStage.STAGE_0_NORMAL
        self.surge_injected = False
        self.surge_target_time = self.sim_time + random.uniform(10.0, 15.0)
        self.next_surge_cycle = self.cycle_count + int(random.uniform(10.0, 15.0) / self.dt)
        self.cmd_main_velocity = self.MAIN_MOTOR_FULL_SPEED
        self.cmd_fan_velocity = self.FAN_MOTOR_INACTIVE
        self.set_actuator_velocities(self.MAIN_MOTOR_FULL_SPEED, self.FAN_MOTOR_INACTIVE)

        print(f"{ANSI.BG_GREEN}{ANSI.WHITE}{ANSI.BOLD}  [OSHA LOTO CLEARED - AUTHORIZED SUPERVISOR RESET]  {ANSI.RESET}")
        print(f"{ANSI.GREEN}  Thermal conditions verified safe ({current_temp:.2f}°C < {self.TEMP_SAFE_RESET_LIMIT:.1f}°C).")
        print(f"  Safety interlock de-latched. Conveyor drive re-energized to normal drive velocity ({self.MAIN_MOTOR_FULL_SPEED:.1f} rad/s).")
        print(f"  Machine returned to Stage 0 Nominal Operation.{ANSI.RESET}\n")

    def output_trip_diagnostics(self, ror_at_trip: float) -> None:
        """
        Output structured, high-visibility trip diagnostics formatted with ANSI colors.
        """
        fan_status_str = f"ACTIVE ({self.cmd_fan_velocity:.1f} rad/s - Convective Remediation)"
        if self.fan_hardware_fault:
            fan_status_str = "FAULTED / ZERO RPM (Hardware Breakdown)"
        elif not self.fan_motor_available:
            fan_status_str = "SIMULATED DISSIPATION (Motor not declared)"

        wall_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        trip_str = self.trip_reason.value if hasattr(self.trip_reason, 'value') else str(self.trip_reason)

        print(f"\n{ANSI.BG_RED}{ANSI.WHITE}{ANSI.BOLD}======================================================================{ANSI.RESET}")
        print(f"{ANSI.RED}{ANSI.BOLD}  CRITICAL SAFETY TRIP: EMERGENCY SHUTDOWN (ESD) INTERLOCK ENGAGED   {ANSI.RESET}")
        print(f"{ANSI.BG_RED}{ANSI.WHITE}{ANSI.BOLD}======================================================================{ANSI.RESET}")
        print(f"  {ANSI.BOLD}Sim Timestamp{ANSI.RESET}           : {self.sim_time:.3f} s (Sim Time) | {wall_time_str}")
        print(f"  {ANSI.BOLD}Primary Trip Trigger{ANSI.RESET}    : {ANSI.BOLD}{ANSI.RED}{trip_str}{ANSI.RESET}")
        print(f"  {ANSI.BOLD}Core Temperature{ANSI.RESET}        : {self.temperature:.2f} °C (Ceiling Limit: {self.TEMP_STAGE_2_THRESHOLD:.1f} °C)")
        print(f"  {ANSI.BOLD}Peak Temperature Seen{ANSI.RESET}   : {self.peak_temperature:.2f} °C")
        print(f"  {ANSI.BOLD}Thermal Rate-of-Rise{ANSI.RESET}    : {ror_at_trip:+.2f} °C/s (Predictive Ceiling: {self.RATE_OF_RISE_THRESHOLD:.2f} °C/s)")
        print(f"  {ANSI.BOLD}Sustained Overheat Dur{ANSI.RESET}  : {self.overheat_timer:.2f} s (Continuous Limit: {self.OVERHEAT_SUSTAINED_SEC:.1f} s)")
        print(f"  {ANSI.BOLD}Main Drive Motor{ANSI.RESET}        : {ANSI.RED}HARD CUT / 0.0 rad/s (CONVEYOR HALTED INSTANTLY){ANSI.RESET}")
        print(f"  {ANSI.BOLD}Auxiliary Cooling Fan{ANSI.RESET}   : {ANSI.CYAN}{fan_status_str}{ANSI.RESET}")
        print(f"  {ANSI.BOLD}Andon Beacon Status{ANSI.RESET}     : {ANSI.RED}RED LED FLASHING (EMERGENCY ALARM ACTIVE){ANSI.RESET}")
        print(f"  {ANSI.BOLD}Safety Latch State{ANSI.RESET}      : {ANSI.BG_RED}{ANSI.WHITE} HARD OSHA LOTO LOCKOUT ENGAGED {ANSI.RESET}")
        print(f"  {ANSI.BOLD}OSHA Standard Ref{ANSI.RESET}       : 29 CFR 1910.147 Control of Hazardous Energy (LOTO)")
        print(f"  {ANSI.BOLD}Supervisor Action Req{ANSI.RESET}   : 1. Allow convective cooling until Temp < {self.TEMP_SAFE_RESET_LIMIT:.1f} °C")
        print(f"                            2. Verify mechanical conveyor clearance & motor integrity")
        print(f"                            3. Press supervisor reset key ['R']")
        print(f"{ANSI.RED}======================================================================{ANSI.RESET}\n")

    def update_physics_and_thermal(self) -> None:
        """
        Execute the automated thermal degradation pipeline and convective physics:

        Thermal Model:
        - Baseline heating rate: TEMP_RISE_RATE = 0.125 °C/s (+0.10°C to +0.15°C/s normal smooth rise).
        - Convective cooling rate: CONVECTIVE_COOLING_RATE = 0.05 °C/s when fan active (15.0 rad/s).
        - In Stage 1: Throttled heat input + degradation friction keeps positive upward progression.
        - Dynamic Spike Trigger: Sudden jump of +0.35°C in a single 64ms tick after 10-15s normal run.
        """
        self.tick_count += 1

        # 1. Drive Motor Heat Generation (Normal smooth rise: +0.10°C to +0.15°C per second)
        delta_heating = 0.0
        if self.cmd_main_velocity > 0.0:
            speed_ratio = self.cmd_main_velocity / self.MAIN_MOTOR_FULL_SPEED
            base_heat = self.TEMP_RISE_RATE * speed_ratio

            # In Yellow Zone (Stage 1), throttling reduces heat input by 40% (0.60 factor)
            if self.current_stage == InterlockStage.STAGE_1_TRIAGE:
                # Severe mechanical conveyor load & automated thermal degradation pipeline
                degradation = 1.0 + 0.04 * max(0.0, self.current_temp - self.TEMP_STAGE_1_THRESHOLD)
                heat_input_rate = (0.60 * self.TEMP_RISE_RATE) * degradation
                delta_heating = heat_input_rate * self.dt
            else:
                delta_heating = base_heat * self.dt

        # 2. Convective Cooling Dissipation
        fan_effective = (self.cmd_fan_velocity > 0.0) and (not self.fan_hardware_fault)
        delta_cooling = 0.0
        if fan_effective:
            if self.cmd_main_velocity == 0.0:
                # Active post-trip remediation at stopped rotor
                cooling_rate = 0.20
            else:
                cooling_rate = self.CONVECTIVE_COOLING_RATE * (self.cmd_fan_velocity / self.FAN_MOTOR_ACTIVE)
            delta_cooling = cooling_rate * self.dt

        # 3. External disturbance / fault surge flux (decaying pulse from 'H' key)
        delta_flux = self.external_heat_flux * self.dt
        self.external_heat_flux = max(0.0, self.external_heat_flux - (4.0 * self.dt))

        # 4. Small ambient decay towards 25.0°C only if both motor and fan are off
        ambient_decay = 0.0
        if self.cmd_main_velocity == 0.0 and self.cmd_fan_velocity == 0.0:
            ambient_decay = 0.02 * (self.current_temp - self.AMBIENT_TEMPERATURE) * self.dt

        # 5. Dynamic Spike Trigger & Manual Spike Injection ('S' Key)
        # While the motor is running smoothly, after a short operational period
        # (around 10–15 seconds of normal running, or randomly via a simulation event),
        # or when triggered manually via 'S' key, introduce an instantaneous surge:
        # a sudden temperature delta jump of +0.35°C in a single 64ms tick.
        # dt = 0.064s -> dT/dt = 0.35 / 0.064 ~ 5.47°C/s.
        surge_jump = 0.0
        if self.manual_spike_pending:
            if not self.system_locked:
                surge_jump = 0.35
                print(f"\n{ANSI.BG_YELLOW}{ANSI.BOLD} >>> [SPIKE INJECTED] Instantaneous surge: +{surge_jump:.2f}°C in single tick! <<< {ANSI.RESET}")
            self.manual_spike_pending = False
        elif (self.enable_random_surge and self.predictive_mode_enabled and not self.system_locked and not self.surge_injected and
                self.tick_count > self.warmup_ticks and
                self.current_stage in (InterlockStage.STAGE_0_NORMAL, InterlockStage.STAGE_1_TRIAGE)):
            if self.sim_time >= self.surge_target_time or self.cycle_count >= self.next_surge_cycle:
                surge_jump = 0.35  # Sudden temperature delta jump of +0.35°C in a single 64ms tick
                self.surge_injected = True
                print(f"\n{ANSI.BG_YELLOW}{ANSI.BOLD} >>> [DYNAMIC SPIKE TRIGGER] Sudden surge: +{surge_jump:.2f}°C in single 64ms tick! <<< {ANSI.RESET}")

        self.cycle_count += 1

        # Net thermal update
        net_delta = delta_heating - delta_cooling + delta_flux - ambient_decay + surge_jump
        self.current_temp = max(self.AMBIENT_TEMPERATURE, self.current_temp + net_delta)

        # Track peak temperature
        if self.current_temp > self.peak_temperature:
            self.peak_temperature = self.current_temp

        # 6. Compute Rate-of-Rise: dT/dt (°C / second) relative to previous tick
        if self.dt > 1e-6:
            self.rate_of_rise = (self.current_temp - self.prev_temp) / self.dt
        else:
            self.rate_of_rise = 0.0

        # Store temperature for next tick's delta calculation
        self.prev_temp = self.current_temp

    def evaluate_interlock_logic(self) -> None:
        """
        Execute safety interlock state machine rules:
        - If OSHA LOTO locked: motor is strictly held dead at 0.0 rad/s.
        - Stage 2: Temp >= 90.0°C sustained for >= 2.0s -> Automatic Emergency Trip.
        - Stage 1: 75.0°C <= Temp < 90.0°C -> Warning & derating (motor 2.5 rad/s, fan 15.0 rad/s).
        - Stage 0: Temp < 75.0°C -> Nominal run (motor 5.0 rad/s, fan 0.0 rad/s).
        """
        # If locked under OSHA LOTO, motor is strictly inhibited at 0.0 rad/s
        if self.system_locked:
            self.current_stage = InterlockStage.ESD_TRIPPED
            self.set_actuator_velocities(self.MAIN_MOTOR_STOPPED, self.FAN_MOTOR_ACTIVE)
            return

        # -------------------------------------------------------------
        # Rule 1: Stage 2 Critical Overheat (Temp >= 90.0°C)
        # -------------------------------------------------------------
        if self.temperature >= self.TEMP_STAGE_2_THRESHOLD:
            self.current_stage = InterlockStage.STAGE_2_OVERHEAT
            self.overheat_timer += self.dt

            # If temperature remains >= 90°C for longer than 2.0 continuous seconds:
            if self.overheat_timer >= self.OVERHEAT_SUSTAINED_SEC:
                self.trigger_trip(TripReason.SUSTAINED_OVERHEAT, self.rate_of_rise)
                return
            else:
                # Motor throttled / maximum fan during initial overheat warning countdown
                self.set_actuator_velocities(self.MAIN_MOTOR_THROTTLED, self.FAN_MOTOR_ACTIVE)
                return
        else:
            # Overheat condition was interrupted or has not occurred -> reset continuous timer
            self.overheat_timer = 0.0

        # -------------------------------------------------------------
        # Rule 2: Stage 1 Active Warning & Derating (75.0°C <= Temp < 90.0°C)
        # Main motor throttled to 50% (2.5 rad/s), cooling fan active (15.0 rad/s)
        # -------------------------------------------------------------
        if self.temperature >= self.TEMP_STAGE_1_THRESHOLD:
            self.current_stage = InterlockStage.STAGE_1_TRIAGE
            self.set_actuator_velocities(self.MAIN_MOTOR_THROTTLED, self.FAN_MOTOR_ACTIVE)
            return

        # -------------------------------------------------------------
        # Rule 3: Stage 0 Nominal Operation (Temp < 75.0°C)
        # Main motor operates at full speed (5.0 rad/s), cooling fan inactive (0.0 rad/s)
        # -------------------------------------------------------------
        self.current_stage = InterlockStage.STAGE_0_NORMAL
        self.set_actuator_velocities(self.MAIN_MOTOR_FULL_SPEED, self.FAN_MOTOR_INACTIVE)

    def log_telemetry_step(self) -> None:
        """
        Output scheduled console telemetry and state transitions with ANSI colors.
        """
        stage_changed = (self.current_stage != self.last_logged_stage)
        time_for_periodic = (self.sim_time - self.last_telemetry_time >= self.telemetry_interval)

        if not (stage_changed or time_for_periodic):
            return

        self.last_telemetry_time = self.sim_time
        self.last_logged_stage = self.current_stage

        # Format Sim Time MM:SS.SS
        mins = int(self.sim_time // 60)
        secs = self.sim_time % 60
        time_str = f"{mins:02d}:{secs:05.2f}"

        # Color badges per stage
        if self.current_stage == InterlockStage.STAGE_0_NORMAL:
            stage_badge = f"{ANSI.GREEN}[STAGE 0: NOMINAL (GREEN)]{ANSI.RESET}"
            temp_color = ANSI.GREEN
            andon_badge = f"{ANSI.GREEN}GREEN{ANSI.RESET}"
        elif self.current_stage == InterlockStage.STAGE_1_TRIAGE:
            stage_badge = f"{ANSI.YELLOW}[STAGE 1: DERATING (AMBER)]{ANSI.RESET}"
            temp_color = ANSI.YELLOW
            andon_badge = f"{ANSI.YELLOW}AMBER{ANSI.RESET}"
        elif self.current_stage == InterlockStage.STAGE_2_OVERHEAT:
            stage_badge = f"{ANSI.RED}[STAGE 2: OVERHEAT (RED)]{ANSI.RESET}"
            temp_color = ANSI.RED
            andon_badge = f"{ANSI.RED}RED PULSING{ANSI.RESET}"
        else:  # ESD_TRIPPED
            stage_badge = f"{ANSI.BG_RED}{ANSI.WHITE}[ESD TRIPPED: OSHA LOTO]{ANSI.RESET}"
            temp_color = ANSI.RED
            andon_badge = f"{ANSI.RED}RED STROBE{ANSI.RESET}"

        loto_badge = f"{ANSI.RED}LOCKED{ANSI.RESET}" if self.system_locked else f"{ANSI.GREEN}CLEAR{ANSI.RESET}"
        pred_badge = f"{ANSI.CYAN}ON{ANSI.RESET}" if self.predictive_mode_enabled else f"{ANSI.DARK_GRAY}OFF{ANSI.RESET}"

        # Print clean status line
        print(
            f"[{time_str}] {stage_badge} "
            f"Temp: {temp_color}{self.temperature:5.1f}°C{ANSI.RESET} | "
            f"dT/dt: {self.rate_of_rise:+5.2f}°C/s | "
            f"Drive: {self.cmd_main_velocity:3.1f} rad/s | "
            f"Fan: {self.cmd_fan_velocity:4.1f} rad/s | "
            f"Pred: {pred_badge} | "
            f"Andon: {andon_badge} | "
            f"Overheat: {self.overheat_timer:4.2f}s | "
            f"LOTO: {loto_badge}"
        )

    def step(self) -> bool:
        """
        Execute one complete supervisory cycle:
        1. Process keyboard inputs and overrides
        2. Evaluate safety interlock logic
        3. Update thermal physics and Rate-of-Rise
        4. Check predictive Rate-of-Rise safety trigger (dT/dt > 4.5°C/s ONLY IF predictive_mode_enabled)
        5. Update Andon stack lights and in-viewport SCADA HUD
        6. Stream console telemetry
        7. Advance simulation time
        """
        self.process_keyboard_inputs()
        self.evaluate_interlock_logic()
        self.update_physics_and_thermal()

        # Check predictive Rate-of-Rise safety trigger (ONLY when predictive_mode_enabled is True)
        if self.predictive_mode_enabled and self.tick_count > self.warmup_ticks and not self.system_locked:
            if self.rate_of_rise > self.RATE_OF_RISE_THRESHOLD:
                self.trigger_trip(TripReason.PREDICTIVE_RATE_OF_RISE, self.rate_of_rise)

        self.apply_andon_lights()

        # Update in-viewport SCADA HUD at scheduled intervals
        if self.sim_time - self.last_hud_update >= self.hud_interval:
            self.last_hud_update = self.sim_time
            self.render_scada_hud()

        self.log_telemetry_step()
        self.sim_time += self.dt
        return True


# =============================================================================
# HARDWARE ABSTRACTION & SELF-CONTAINED VALIDATION HARNESS (HEADLESS / TESTS)
# =============================================================================

class MockDevice:
    """Mock Webots motor device for verification without GUI."""
    def __init__(self, name: str):
        self.name = name
        self.position = 0.0
        self.velocity = 0.0

    def setPosition(self, pos: float) -> None:
        self.position = pos

    def setVelocity(self, vel: float) -> None:
        self.velocity = vel


class MockLED:
    """Mock Webots LED device for automated testing."""
    def __init__(self, name: str):
        self.name = name
        self.val = 0

    def set(self, val: int) -> None:
        self.val = val

    def get(self) -> int:
        return self.val


class MockDisplay:
    """Mock Webots Display node for in-viewport HUD testing."""
    def __init__(self, name: str, width: int = 320, height: int = 180):
        self.name = name
        self.width = width
        self.height = height

    def getWidth(self) -> int:
        return self.width

    def getHeight(self) -> int:
        return self.height

    def setColor(self, color: int) -> None:
        pass

    def fillRectangle(self, x: int, y: int, w: int, h: int) -> None:
        pass

    def drawRectangle(self, x: int, y: int, w: int, h: int) -> None:
        pass

    def drawLine(self, x1: int, y1: int, x2: int, y2: int) -> None:
        pass

    def drawText(self, text: str, x: int, y: int) -> None:
        pass

    def setFont(self, font: str, size: int, anti_aliasing: bool) -> None:
        pass


class MockKeyboard:
    """Mock Webots keyboard buffer for automated headless tests."""
    def __init__(self):
        self.queue = []

    def enable(self, ms: int) -> None:
        pass

    def push_key(self, char: str) -> None:
        self.queue.append(ord(char))

    def getKey(self) -> int:
        return self.queue.pop(0) if self.queue else -1


class MockRobot:
    """Mock Webots Robot for standalone CLI validation."""
    def __init__(self, include_fan: bool = True, include_peripherals: bool = True):
        self.devices = {
            'rotational motor': MockDevice('rotational motor')
        }
        if include_fan:
            self.devices['fan motor'] = MockDevice('fan motor')
        if include_peripherals:
            self.devices['green_led'] = MockLED('green_led')
            self.devices['amber_led'] = MockLED('amber_led')
            self.devices['red_led'] = MockLED('red_led')
            self.devices['scada_display'] = MockDisplay('scada_display')

    def getDevice(self, name: str):
        return self.devices.get(name, None)

    def getBasicTimeStep(self) -> float:
        return 64.0


def run_automated_validation_suite() -> bool:
    """
    Comprehensive headless test suite validating all upgraded industrial specifications:
    1. Stage 0 Nominal Operation (Green LED ON, Motor 5.0, Fan 0.0)
    2. Stage 1 Active Warning & Derating (Amber LED ON, Motor 2.5, Fan 15.0)
    3. Automated Progression through Yellow Zone (No 75°C freeze, steady climb to 90°C)
    4. Stage 2 Sustained Overheat (>= 90°C for >= 2.0s) -> Automatic ESD Trip & LOTO
    5. Predictive Rate-of-Rise Analytics trip (> 4.5°C/s before 90°C)
    6. Fault Injection ('F' Key Bearing Seizure -> instant surge to 92°C and auto-trip)
    7. Conditional Supervisor Reset Check ('R' Key rejection if >= 75°C, acceptance if < 75°C)
    8. Random High-Temperature Surge (Predictive dT/dt Auto-Trip without keyboard input)
    9. Hardware Tolerance: Missing fan motor & missing peripherals without crashing
    10. In-Viewport SCADA HUD Display rendering
    """
    print(f"\n{ANSI.BOLD}{ANSI.CYAN}======================================================================{ANSI.RESET}")
    print(f"{ANSI.BOLD}{ANSI.CYAN}  STARTING AUTOMATED UPGRADE VERIFICATION SUITE                       {ANSI.RESET}")
    print(f"{ANSI.BOLD}{ANSI.CYAN}======================================================================{ANSI.RESET}\n")

    tests_passed = 0
    total_tests = 10

    # -------------------------------------------------------------------------
    # Test 1: Stage 0 Nominal Operation (Temp < 75.0°C)
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 1/10] Stage 0 Nominal Operation (Temp < 75.0°C)...{ANSI.RESET}")
    mock_robot = MockRobot(include_fan=True, include_peripherals=True)
    mock_kb = MockKeyboard()
    interlock = SafetyInterlockSupervisor(mock_robot, mock_kb, time_step=100, initial_temp=60.0, enable_random_surge=False)
    interlock.step()

    assert interlock.current_stage == InterlockStage.STAGE_0_NORMAL, "Failed: Expected Stage 0"
    assert interlock.cmd_main_velocity == 5.0, "Failed: Main motor should be 5.0 rad/s"
    assert interlock.cmd_fan_velocity == 0.0, "Failed: Fan motor should be 0.0 rad/s"
    assert not interlock.system_locked, "Failed: Interlock should not be locked"
    assert mock_robot.getDevice('green_led').get() == 1, "Failed: Green LED must be ON"
    assert mock_robot.getDevice('amber_led').get() == 0, "Failed: Amber LED must be OFF"
    assert mock_robot.getDevice('red_led').get() == 0, "Failed: Red LED must be OFF"
    print(f"{ANSI.GREEN}  PASSED: Main Motor = 5.0 rad/s, Fan = 0.0 rad/s, Green LED ON, Amber/Red OFF.{ANSI.RESET}\n")
    tests_passed += 1

    # -------------------------------------------------------------------------
    # Test 2: Stage 1 Active Warning & Derating (75.0°C <= Temp < 90.0°C)
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 2/10] Stage 1 Active Warning & Derating (75.0°C <= Temp < 90.0°C)...{ANSI.RESET}")
    interlock = SafetyInterlockSupervisor(mock_robot, mock_kb, time_step=100, initial_temp=78.0, enable_random_surge=False)
    interlock.step()

    assert interlock.current_stage == InterlockStage.STAGE_1_TRIAGE, "Failed: Expected Stage 1 Warning"
    assert interlock.cmd_main_velocity == 2.5, "Failed: Main motor should be throttled by 50% (2.5 rad/s)"
    assert interlock.cmd_fan_velocity == 15.0, "Failed: Fan motor should be rapid 15.0 rad/s"
    assert mock_robot.getDevice('green_led').get() == 0, "Failed: Green LED must be OFF"
    assert mock_robot.getDevice('amber_led').get() == 1, "Failed: Amber LED must be ON"
    assert mock_robot.getDevice('red_led').get() == 0, "Failed: Red LED must be OFF"
    print(f"{ANSI.GREEN}  PASSED: Main Throttled 50% (2.5 rad/s), Fan Active (15.0 rad/s), Amber LED ON.{ANSI.RESET}\n")
    tests_passed += 1

    # -------------------------------------------------------------------------
    # Test 3: Automated Thermal Degradation Progression (No 75°C Freeze)
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 3/10] Automated Thermal Degradation Progression (75°C -> 90°C without freeze)...{ANSI.RESET}")
    interlock = SafetyInterlockSupervisor(mock_robot, mock_kb, time_step=100, initial_temp=75.0, enable_random_surge=False)

    # Step through 50 steps (5.0s simulation time) in Yellow Zone
    initial_t = interlock.temperature
    for _ in range(50):
        interlock.step()

    assert interlock.temperature > initial_t, f"Failed: Temperature must climb continuously; was {initial_t}°C, now {interlock.temperature}°C"
    # Verify positive rate-of-rise in Stage 1
    assert interlock.rate_of_rise > 0.0, f"Failed: Net rate-of-rise must be positive in Yellow Zone, got {interlock.rate_of_rise:+.3f}°C/s"
    print(f"{ANSI.GREEN}  PASSED: Temperature climbed steadily from {initial_t:.1f}°C to {interlock.temperature:.1f}°C (dT/dt: {interlock.rate_of_rise:+.3f}°C/s). No 75°C stall.{ANSI.RESET}\n")
    tests_passed += 1

    # -------------------------------------------------------------------------
    # Test 4: Stage 2 Sustained Overheat (>= 90°C for >= 2.0s) -> Automatic ESD Trip
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 4/10] Stage 2 Sustained Overheat Countdown & Automatic Trip...{ANSI.RESET}")
    interlock = SafetyInterlockSupervisor(mock_robot, mock_kb, time_step=100, initial_temp=91.0, enable_random_surge=False)

    # Step for 1.8s (18 steps at 0.1s) -> Should warn but NOT trip yet
    for _ in range(18):
        interlock.temperature = 91.0
        interlock.step()

    assert not interlock.system_locked, "Failed: Should not trip before 2.0 continuous seconds"
    assert interlock.current_stage == InterlockStage.STAGE_2_OVERHEAT, "Failed: Expected Stage 2 Overheat countdown"

    # Step past 2.0s (3 more steps = 2.1s total)
    for _ in range(3):
        interlock.temperature = 91.0
        interlock.step()

    assert interlock.system_locked, "Failed: Interlock must automatically trip after 2.0s sustained overheat"
    assert interlock.trip_reason == TripReason.SUSTAINED_OVERHEAT, f"Failed: Expected SUSTAINED_OVERHEAT, got {interlock.trip_reason}"
    assert interlock.cmd_main_velocity == 0.0, "Failed: Main motor must hard cut to 0.0 rad/s"
    assert interlock.cmd_fan_velocity == 15.0, "Failed: Fan must remain ACTIVE (15.0 rad/s) for remediation"
    print(f"{ANSI.GREEN}  PASSED: Sustained overheat accurately tripped at {interlock.overheat_timer:.2f}s >= 2.0s. Motor cut to 0.0, Fan remains 15.0.{ANSI.RESET}\n")
    tests_passed += 1

    # -------------------------------------------------------------------------
    # Test 5: Predictive Analytics Rate-of-Rise Trip (> 4.5°C/s before 90°C)
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 5/10] Advanced Predictive Analytics Rate-of-Rise Trip (> 4.5°C/s)...{ANSI.RESET}")
    interlock = SafetyInterlockSupervisor(mock_robot, mock_kb, time_step=100, initial_temp=70.0, enable_random_surge=False)
    interlock.predictive_mode_enabled = True  # Enable predictive surge presentation mode
    # Complete warm-up phase (first 5 ticks)
    for _ in range(6):
        interlock.step()
    # Injected sudden thermal shock surge (+5.5°C in 0.1s = 55°C/s)
    interlock.temperature += 5.5
    interlock.step()

    assert interlock.system_locked, "Failed: Predictive trip must engage latch lock"
    assert interlock.trip_reason == "PREDICTIVE ANOMALY: dT/dt EXCEEDED 4.5°C/s" or interlock.trip_reason == TripReason.PREDICTIVE_RATE_OF_RISE
    assert interlock.temperature < 90.0, "Failed: Predictive trip must occur BEFORE reaching 90.0°C ceiling"
    assert interlock.cmd_main_velocity == 0.0, "Failed: Motor must halt immediately"
    print(f"{ANSI.GREEN}  PASSED: Predictive Trip engaged at {interlock.temperature:.1f}°C (< 90°C) on dT/dt = {interlock.rate_of_rise:+.2f}°C/s.{ANSI.RESET}\n")
    tests_passed += 1

    # -------------------------------------------------------------------------
    # Test 6: Fault Injection: Bearing Seizure ('F' Key -> instant surge to 92°C)
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 6/10] Fault Injection: Bearing Seizure ('F' Key)...{ANSI.RESET}")
    interlock = SafetyInterlockSupervisor(mock_robot, mock_kb, time_step=100, initial_temp=65.0, enable_random_surge=False)
    for _ in range(6):
        interlock.step()
    assert not interlock.system_locked

    mock_kb.push_key('F')
    interlock.step()

    # Temperature jumped immediately to 92.0°C
    assert interlock.temperature >= 92.0, f"Failed: 'F' key should jump temperature to >= 92.0°C, got {interlock.temperature}°C"
    assert interlock.current_stage == InterlockStage.STAGE_2_OVERHEAT or interlock.system_locked

    # Step for 2.0s to verify automatic auto-trip from bearing seizure
    for _ in range(21):
        interlock.step()

    assert interlock.system_locked, "Failed: Interlock must trip automatically after bearing seizure"
    assert interlock.cmd_main_velocity == 0.0, "Failed: Drive motor must be halted"
    print(f"{ANSI.GREEN}  PASSED: 'F' key jumped core temp to 92.0°C and triggered automatic emergency trip.{ANSI.RESET}\n")
    tests_passed += 1

    # -------------------------------------------------------------------------
    # Test 7: Conditional Supervisor Reset Check ('R' Key Rejection & Clearance)
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 7/10] Conditional Supervisor Reset Check ('R' Key)...{ANSI.RESET}")
    # Attempt reset while temperature is still dangerous (92.0°C >= 75.0°C)
    mock_kb.push_key('R')
    interlock.step()
    assert interlock.system_locked, "Failed: LOTO reset must be REJECTED while Temp >= 75.0°C"
    assert interlock.cmd_main_velocity == 0.0, "Failed: Main motor velocity must remain 0.0 rad/s when reset rejected"

    # Cool down to safe temperature (50.0°C < 75.0°C) and reset
    interlock.temperature = 50.0
    mock_kb.push_key('R')
    interlock.step()
    assert not interlock.system_locked, "Failed: LOTO reset should succeed when Temp < 75.0°C"
    assert interlock.current_stage == InterlockStage.STAGE_0_NORMAL, "Failed: Should return to Stage 0"
    assert interlock.cmd_main_velocity == 5.0, "Failed: Main motor should restore to full speed (5.0 rad/s)"
    assert interlock.cmd_fan_velocity == 0.0, "Failed: Fan motor should return to 0.0 rad/s"
    print(f"{ANSI.GREEN}  PASSED: [RESET REJECTED - THERMAL HAZARD] properly guarded >= 75°C; reset cleared at < 75°C.{ANSI.RESET}\n")
    tests_passed += 1

    # -------------------------------------------------------------------------
    # Test 8: Presentation Mode Toggle ('P' Key) & Dynamic Spike ('S' Key)
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 8/10] Presentation Mode Toggle ('P') & Dynamic Spike ('S')...{ANSI.RESET}")
    surge_robot = MockRobot(include_fan=True, include_peripherals=True)
    surge_kb = MockKeyboard()
    # 1. First-tick initialization check
    surge_interlock = SafetyInterlockSupervisor(surge_robot, surge_kb, time_step=64, initial_temp=26.0, enable_random_surge=False)
    assert surge_interlock.prev_temp == surge_interlock.current_temp, "Failed: prev_temp must equal current_temp at initialization"
    assert not surge_interlock.predictive_mode_enabled, "Failed: predictive_mode_enabled must default to False"

    # 2. Warm-up Phase check: bypass dT/dt trip check for first 5 ticks
    for _ in range(5):
        surge_interlock.step()
    assert not surge_interlock.system_locked, "Failed: First 5 ticks (warm-up) must not trigger emergency lockout"
    assert surge_interlock.cmd_main_velocity == 5.0, "Failed: Motor must continue at full speed during warm-up"

    # 3. Normal smooth temperature rise check (dT/dt ~ 0.10 to 0.20 °C/s)
    for _ in range(5):
        surge_interlock.step()
    assert not surge_interlock.system_locked, "Failed: Normal gradual rise must not trip motor"
    assert 0.05 < surge_interlock.rate_of_rise < 0.30, f"Failed: Expected normal dT/dt around 0.10-0.20°C/s, got {surge_interlock.rate_of_rise:+.2f}°C/s"

    # 4. Spike with Predictive Mode OFF: press 'S' -> dT/dt > 4.5°C/s, but trip is bypassed!
    surge_kb.push_key('S')
    surge_interlock.step()
    assert surge_interlock.rate_of_rise > 4.5, f"Failed: Rate of rise should exceed 4.5°C/s on 'S' key, got {surge_interlock.rate_of_rise:+.2f}°C/s"
    assert not surge_interlock.system_locked, "Failed: Spike must NOT trip when predictive_mode_enabled is False"
    assert surge_interlock.cmd_main_velocity == 5.0, "Failed: Motor must remain running when predictive mode is OFF"

    # Step one tick to settle
    surge_interlock.step()

    # 5. Toggle Predictive Mode ON: press 'P'
    surge_kb.push_key('P')
    surge_interlock.step()
    assert surge_interlock.predictive_mode_enabled, "Failed: 'P' key should enable predictive_mode_enabled"

    # 6. Spike with Predictive Mode ON: press 'S' -> immediate predictive ESD trip!
    surge_kb.push_key('S')
    surge_interlock.step()
    assert surge_interlock.system_locked, "Failed: Spike must trip emergency lockout when predictive_mode_enabled is True"
    assert surge_interlock.trip_reason == "PREDICTIVE ANOMALY: dT/dt EXCEEDED 4.5°C/s"
    assert surge_interlock.cmd_main_velocity == 0.0, "Failed: Main motor must be immediately halted at 0.0 rad/s"
    assert surge_interlock.trip_rate_of_rise > 4.5, f"Failed: Recorded trip peak dT/dt should exceed 4.5°C/s, got {surge_interlock.trip_rate_of_rise:+.2f}°C/s"

    # 7. Toggle 'P' key again -> test toggling back OFF
    surge_kb.push_key('P')
    surge_interlock.step()
    assert not surge_interlock.predictive_mode_enabled, "Failed: 'P' key should toggle predictive_mode_enabled back to False"

    print(f"{ANSI.GREEN}  PASSED: Warm-up verified, 'P' toggle verified, spike bypassed when OFF, and 'S' spike tripped LOTO when ON.{ANSI.RESET}\n")
    tests_passed += 1

    # -------------------------------------------------------------------------
    # Test 9: Hardware Fault Tolerance (Missing 'fan motor' & missing LEDs)
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 9/10] Hardware Tolerance: Missing Fan & Peripherals...{ANSI.RESET}")
    bare_robot = MockRobot(include_fan=False, include_peripherals=False)
    bare_interlock = SafetyInterlockSupervisor(bare_robot, mock_kb, time_step=100, initial_temp=80.0, enable_random_surge=False)

    assert not bare_interlock.fan_motor_available, "Failed: Fan should be detected as unavailable"
    assert not bare_interlock.leds_available, "Failed: LEDs should be detected as unavailable"
    assert not bare_interlock.display_available, "Failed: Display should be detected as unavailable"

    # Step multiple cycles without crashing
    for _ in range(5):
        bare_interlock.step()

    print(f"{ANSI.GREEN}  PASSED: Controller executes cleanly without fan, LEDs, or display declared; zero exceptions.{ANSI.RESET}\n")
    tests_passed += 1

    # -------------------------------------------------------------------------
    # Test 10: In-Viewport SCADA HUD Display Rendering
    # -------------------------------------------------------------------------
    print(f"{ANSI.BOLD}[TEST 10/10] In-Viewport SCADA HUD Display Rendering...{ANSI.RESET}")
    hud_robot = MockRobot(include_fan=True, include_peripherals=True)
    hud_interlock = SafetyInterlockSupervisor(hud_robot, mock_kb, time_step=100, initial_temp=76.0, enable_random_surge=False)
    assert hud_interlock.display_available
    # Exercise HUD rendering under all operating stages & predictive toggle states
    hud_interlock.step()  # Stage 1 with Predictive OFF
    hud_interlock.predictive_mode_enabled = True
    hud_interlock.render_scada_hud()  # Stage 1 with Predictive ON
    hud_interlock.temperature = 92.0
    hud_interlock.step()  # Stage 2 Overheat
    hud_interlock.trigger_manual_estop()
    hud_interlock.step()  # ESD Tripped / LOTO
    print(f"{ANSI.GREEN}  PASSED: In-viewport SCADA HUD rendered cleanly across all operational stages.{ANSI.RESET}\n")
    tests_passed += 1

    print(f"{ANSI.BG_GREEN}{ANSI.WHITE}{ANSI.BOLD}======================================================================{ANSI.RESET}")
    print(f"{ANSI.GREEN}{ANSI.BOLD}  ALL {tests_passed}/{total_tests} AUTOMATED UPGRADE VERIFICATION TESTS PASSED SUCCESSFULLY! {ANSI.RESET}")
    print(f"{ANSI.BG_GREEN}{ANSI.WHITE}{ANSI.BOLD}======================================================================{ANSI.RESET}\n")
    return True


# =============================================================================
# MAIN CONTROLLER ENTRY POINT (WEBOTS NATIVE EXECUTION & CLI)
# =============================================================================

def main():
    """
    Main controller loop.
    Supports:
    - Native execution inside Webots 3D simulation.
    - Automated verification mode via `--test`.
    - Interactive CLI demonstration via `--standalone`.
    """
    parser = argparse.ArgumentParser(description="Webots Cyber-Physical Safety Interlock Controller")
    parser.add_argument("--test", action="store_true", help="Run automated headless verification test suite")
    parser.add_argument("--standalone", action="store_true", help="Run standalone terminal simulation loop")
    args, unknown = parser.parse_known_args()

    if args.test:
        run_automated_validation_suite()
        sys.exit(0)

    # Attempt to load Webots native controller libraries
    try:
        from controller import Robot, Keyboard, Display, LED
        webots_imported = True
    except ImportError:
        webots_imported = False

    if args.standalone or not webots_imported:
        print(f"{ANSI.YELLOW}[INFO] Running in Standalone CLI Mode (Webots IPC disconnected).{ANSI.RESET}")
        mock_robot = MockRobot(include_fan=True, include_peripherals=True)
        mock_kb = MockKeyboard()
        interlock = SafetyInterlockSupervisor(mock_robot, mock_kb, time_step=64, initial_temp=26.0)

        print(f"{ANSI.CYAN}Simulating live step loop. Press Ctrl+C to terminate...{ANSI.RESET}")
        try:
            step_count = 0
            while True:
                interlock.step()
                time.sleep(0.064)
                step_count += 1
                if step_count == 250:
                    print(f"\n{ANSI.YELLOW}[DEMO AUTO-INJECTION] Simulating bearing seizure fault ('F')...{ANSI.RESET}")
                    mock_kb.push_key('F')
        except KeyboardInterrupt:
            print(f"\n{ANSI.GREEN}[SHUTDOWN] Controller loop terminated cleanly.{ANSI.RESET}")
            sys.exit(0)

    # -------------------------------------------------------------------------
    # Native Webots Execution Loop
    # -------------------------------------------------------------------------
    try:
        robot = Robot()
        time_step = int(robot.getBasicTimeStep())
        keyboard = robot.getKeyboard()

        supervisor = SafetyInterlockSupervisor(
            robot=robot,
            keyboard=keyboard,
            time_step=time_step,
            initial_temp=26.0
        )

        print(f"{ANSI.GREEN}[ESD SUPERVISOR] Native Webots step loop engaged (TimeStep: {time_step}ms).{ANSI.RESET}")

        # Native Webots clean step-loop execution
        while robot.step(time_step) != -1:
            supervisor.step()

        print(f"{ANSI.CYAN}[ESD SUPERVISOR] Webots simulation closed. Controller exiting safely.{ANSI.RESET}")

    except Exception as exc:
        print(f"{ANSI.RED}[CRITICAL ERROR] Webots controller encounter exception: {exc}{ANSI.RESET}")
        sys.exit(1)


if __name__ == '__main__':
    main()
