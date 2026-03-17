"""
spot_status.py — quick Spot health check
Usage: python spot_status.py
"""

import os, sys
from dotenv import load_dotenv

load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))

def _require(name):
    v = os.environ.get(name)
    if not v:
        raise SystemExit(f'Missing env var: {name}')
    return v

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient

robot_ip = _require('SPOT_IP')
username = _require('SPOT_USER')
password = _require('SPOT_PASSWORD')

bosdyn.client.util.setup_logging(verbose=False)
sdk   = bosdyn.client.create_standard_sdk('SpotStatus')
robot = sdk.create_robot(robot_ip)
robot.authenticate(username, password)

state_client = robot.ensure_client(RobotStateClient.default_service_name)
state        = state_client.get_robot_state()

# ── Battery ───────────────────────────────────────────────────────────────
for b in state.battery_states:
    pct   = b.charge_percentage.value
    volts = b.voltage.value
    amps  = b.current.value
    temps = [t if isinstance(t, float) else t.value for t in b.temperatures]
    avg_t = sum(temps) / len(temps) if temps else 0

    bar_len = 20
    filled  = int(pct / 100 * bar_len)
    bar     = '█' * filled + '░' * (bar_len - filled)

    status_str = {1: 'discharging', 2: 'charging', 3: 'full'}.get(b.status, '?')

    print(f'Battery  [{bar}] {pct:.0f}%  {volts:.1f} V  {amps:.1f} A  '
          f'{avg_t:.0f}°C  ({status_str})')

# ── Power / estop ─────────────────────────────────────────────────────────
ps  = state.power_state
estopped = robot.is_estopped()
powered  = robot.is_powered_on()
print(f'Motors   {"ON " if powered else "off"}   '
      f'Estop {"TRIGGERED" if estopped else "clear"}')

# ── Comms / time sync ─────────────────────────────────────────────────────
print(f'Time sync  {"ok" if robot.time_sync.has_established_time_sync else "not synced"}')
