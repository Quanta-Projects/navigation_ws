#!/usr/bin/env python3
"""
Mock Arduino for testing smrr_base_controller docking/charging logic.

Creates a virtual serial port pair using a pty (pseudo-terminal), then runs
through a set of docking scenarios while printing both what is sent to the
controller and what the controller writes back (including the charge_cmd field).

Serial protocol
---------------
  Controller → Arduino  :  "left_scaled,right_scaled,charge_cmd\n"
  Arduino    → Controller:  "r_enc,l_enc,v2,v3,is_connected,is_charging,battery_pct\n"

Usage
-----
  # Step 1 — start the mock Arduino
  python3 src/smrr_base_controller/scripts/mock_arduino.py

  # The script prints a line like:
  #   Virtual serial port : /dev/pts/4
  # Copy that EXACT path (the number differs every run).

  # Step 2 — in a NEW terminal, launch the hardware interface with that path:
  ros2 launch smrr_base_controller hardware_interface.launch.py port:=/dev/pts/4

  # Step 3 (optional) — watch the battery topic in a third terminal:
  ros2 topic echo /battery_state

  The script waits until the controller sends its first command before
  running the scenarios, so there is no timing constraint on step 2.
"""

import os
import pty
import tty
import termios
import threading
import time
import select
import sys


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _set_raw(fd: int) -> None:
    """Put a pty master fd into raw mode so bytes pass through unchanged."""
    attrs = termios.tcgetattr(fd)
    tty.setraw(fd)
    # Disable software flow-control and output post-processing
    attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.ICRNL)
    attrs[1] &= ~termios.OPOST
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


# ---------------------------------------------------------------------------
# MockArduino
# ---------------------------------------------------------------------------

class MockArduino:
    """Simulates the low-level Arduino controller over a virtual serial port."""

    FEEDBACK_INTERVAL = 0.1   # seconds between feedback lines (matches 100 ms firmware interval)

    def __init__(self):
        self.master_fd, self.slave_fd = pty.openpty()
        _set_raw(self.master_fd)
        self.slave_path: str = os.ttyname(self.slave_fd)

        # Encoder counters (16-bit rollover)
        self.right_encoder: int = 0
        self.left_encoder:  int = 0

        # Docking / charging state exposed to the controller
        self.is_connected:  int = 0
        self.is_charging:   int = 0   # driven by charge_cmd from controller
        self.battery_pct:   int = 50

        # Last parsed command from the controller
        self.last_left:       float = 0.0
        self.last_right:      float = 0.0
        self.last_charge_cmd: int   = 0

        # Statistics
        self.cmd_count:           int = 0
        self.charge_transitions:  list = []   # (timestamp, new_charge_cmd)

        self._running:   bool = False
        self._recv_buf:  str  = ""
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._running = True
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    def stop(self) -> None:
        self._running = False
        try:
            os.close(self.master_fd)
        except OSError:
            pass
        try:
            os.close(self.slave_fd)
        except OSError:
            pass

    # ------------------------------------------------------------------
    # Reader thread — parses commands sent by the controller
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        while self._running:
            try:
                r, _, _ = select.select([self.master_fd], [], [], 0.05)
                if r:
                    data = os.read(self.master_fd, 512).decode("utf-8", errors="replace")
                    self._recv_buf += data
                    while "\n" in self._recv_buf:
                        line, self._recv_buf = self._recv_buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            self._parse_command(line)
            except OSError:
                break

    def _parse_command(self, line: str) -> None:
        """Parse  left_scaled,right_scaled,charge_cmd  from the controller."""
        parts = line.split(",")
        if len(parts) < 3:
            return
        try:
            left   = float(parts[0])
            right  = float(parts[1])
            charge = int(parts[2])
        except ValueError:
            return

        with self._lock:
            prev   = self.last_charge_cmd
            self.last_left       = left
            self.last_right      = right
            self.last_charge_cmd = charge
            self.cmd_count      += 1

            # Simulate charging hardware: start/stop based on controller command
            if charge == 1 and self.is_connected:
                self.is_charging = 1
            else:
                self.is_charging = 0

            if charge != prev:
                self.charge_transitions.append((time.time(), charge))

        print(
            f"  [RX] left={left:+7.2f}  right={right:+7.2f}  charge_cmd={charge}"
            f"  → is_charging={self.is_charging}"
        )

    # ------------------------------------------------------------------
    # Sender — emits 7-value feedback lines
    # ------------------------------------------------------------------

    def _send_feedback(self) -> None:
        with self._lock:
            line = (
                f"{self.right_encoder},{self.left_encoder},"
                f"0,0,"                               # v2, v3 (unused)
                f"{self.is_charging},"
                f"{self.is_connected},"
                f"{self.battery_pct}\n"
            )
        try:
            os.write(self.master_fd, line.encode("utf-8"))
        except OSError:
            pass

    # ------------------------------------------------------------------
    # Scenario runner
    # ------------------------------------------------------------------

    def run_scenario(
        self,
        label: str,
        is_connected: int,
        battery_pct: int,
        duration_s: float,
        encoder_delta: int = 8,
    ) -> None:
        """Run a scenario for *duration_s* seconds, sending feedback every 100 ms."""
        print(f"\n{'─' * 62}")
        print(f"  SCENARIO : {label}")
        print(f"  connected={is_connected}  battery={battery_pct}%  duration={duration_s}s")
        print(f"{'─' * 62}")

        with self._lock:
            self.is_connected = is_connected
            self.battery_pct  = battery_pct
            if not is_connected:
                # Physical disconnection overrides charging regardless of cmd
                self.is_charging = 0

        deadline = time.time() + duration_s
        while time.time() < deadline:
            with self._lock:
                self.right_encoder = (self.right_encoder + encoder_delta) & 0xFFFF
                self.left_encoder  = (self.left_encoder  + encoder_delta) & 0xFFFF
            self._send_feedback()
            time.sleep(self.FEEDBACK_INTERVAL)

        with self._lock:
            print(
                f"\n  → Scenario end  |  "
                f"last charge_cmd from controller = {self.last_charge_cmd}  "
                f"is_charging = {self.is_charging}"
            )


# ---------------------------------------------------------------------------
# Summary helper
# ---------------------------------------------------------------------------

def _print_summary(arduino: MockArduino) -> None:
    print("\n" + "=" * 62)
    print("  TEST SUMMARY")
    print("=" * 62)
    print(f"  Total controller commands received : {arduino.cmd_count}")

    if arduino.charge_transitions:
        print(f"  charge_cmd transitions ({len(arduino.charge_transitions)}):")
        t0 = arduino.charge_transitions[0][0]
        for ts, val in arduino.charge_transitions:
            print(f"    t+{ts - t0:5.1f}s  charge_cmd \u2192 {val}")
    else:
        print("  No charge_cmd transitions observed \u2014 is the controller running?")

    cmds = [v for _, v in arduino.charge_transitions]
    expected = [1, 0, 1]
    if cmds == expected:
        print("\n  RESULT : PASS \u2713  (transitions matched 0\u21921, 1\u21920, 0\u21921)")
    elif cmds:
        print(f"\n  RESULT : transitions were {cmds}, expected {expected}")
        print("  Check controller logs for details.")
    print("=" * 62)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    arduino = MockArduino()
    arduino.start()

    print("=" * 62)
    print("  smrr_base_controller — Mock Arduino (docking/charging test)")
    print("=" * 62)
    print()
    print(f"  Virtual serial port : \033[1;32m{arduino.slave_path}\033[0m")
    print()
    print("  Run this command in a NEW terminal (copy the path exactly):")
    print(f"  \033[1;33mros2 launch smrr_base_controller hardware_interface.launch.py port:={arduino.slave_path}\033[0m")
    print()
    print("  Optional — watch /battery_state in a third terminal:")
    print("    ros2 topic echo /battery_state")
    print()
    print("  Waiting for the controller to send its first command...")
    print("  (Press Ctrl+C to abort at any time)\n")

    try:
        # Wait until the controller sends at least one command (no fixed timeout)
        while arduino.cmd_count == 0:
            time.sleep(0.1)

        print("  Controller connected! Starting scenarios...\n")
        # Allow the Arduino DTR reset cycle to complete (~3 s)
        time.sleep(3)

        # ------------------------------------------------------------------
        # Scenario 1: Normal driving, robot is NOT at the dock
        # ------------------------------------------------------------------
        arduino.run_scenario(
            label="Normal driving — no dock",
            is_connected=0,
            battery_pct=72,
            duration_s=5,
        )

        # ------------------------------------------------------------------
        # Scenario 2: Robot backed into dock — dock physically connected
        #             Controller should detect is_connected=1 and enable charging
        # ------------------------------------------------------------------
        arduino.run_scenario(
            label="Dock CONNECTED — expect controller to send charge_cmd=1",
            is_connected=1,
            battery_pct=72,
            duration_s=6,
        )

        # ------------------------------------------------------------------
        # Scenario 3: Charging in progress
        # ------------------------------------------------------------------
        arduino.run_scenario(
            label="Charging in progress — battery rising",
            is_connected=1,
            battery_pct=78,
            duration_s=5,
        )

        # ------------------------------------------------------------------
        # Scenario 4: Undocking — connection lost
        #             Controller should clear charge_cmd and send 0
        # ------------------------------------------------------------------
        arduino.run_scenario(
            label="UNDOCKING — is_connected=0, expect charge_cmd=0",
            is_connected=0,
            battery_pct=80,
            duration_s=5,
        )

        # ------------------------------------------------------------------
        # Scenario 5: Re-dock to verify the state machine resets correctly
        # ------------------------------------------------------------------
        arduino.run_scenario(
            label="Re-docking — charge should restart automatically",
            is_connected=1,
            battery_pct=80,
            duration_s=6,
        )

        # Print summary now (before the hold loop so results are visible)
        _print_summary(arduino)

        # Keep the pty alive so the controller doesn't crash on a closed port
        print("\n  All scenarios done. Holding port open — press Ctrl+C to exit.\n")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n  Interrupted by user.")
        _print_summary(arduino)
    finally:
        arduino.stop()


if __name__ == "__main__":
    main()
