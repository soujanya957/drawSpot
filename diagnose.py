"""
Network + SDK diagnostics for Spot and Vicon.
Run: python diagnose.py
"""
import os, sys, socket, subprocess, time
sys.path.insert(0, os.path.dirname(__file__))

from dotenv import load_dotenv
load_dotenv()

SPOT_IP   = os.environ.get("SPOT_IP",   "128.148.140.23")
VICON_IP  = "169.254.217.218"
VICON_PORT = 801

SEP = "─" * 50

def ping(ip):
    r = subprocess.run(["ping", "-c", "2", "-W", "2000", ip],
                       capture_output=True, text=True)
    lost = "100%" in r.stdout or "100.0%" in r.stdout
    lines = [l for l in r.stdout.splitlines() if "round-trip" in l or "rtt" in l]
    rtt = lines[0] if lines else "(no rtt)"
    return not lost, rtt

def tcp(ip, port, timeout=10):
    try:
        s = socket.create_connection((ip, port), timeout=timeout)
        s.close()
        return True
    except Exception as e:
        return False

def interface_for(ip):
    r = subprocess.run(["route", "get", ip], capture_output=True, text=True)
    for line in r.stdout.splitlines():
        if "interface" in line:
            return line.strip()
    return "unknown"

# ── Spot ──────────────────────────────────────────────────────────────────
print(SEP)
print(f"  Spot  {SPOT_IP}")
print(SEP)

ok, rtt = ping(SPOT_IP)
print(f"  ping 443     {'OK  ' + rtt if ok else 'FAIL — host unreachable'}")

ok443 = tcp(SPOT_IP, 443)
print(f"  tcp :443     {'OK' if ok443 else 'FAIL — port closed or filtered'}")

print(f"  route        {interface_for(SPOT_IP)}")

if ok443:
    print("  gRPC auth    testing…")
    try:
        import bosdyn.client, bosdyn.client.util
        bosdyn.client.util.setup_logging(verbose=False)
        sdk   = bosdyn.client.create_standard_sdk("diag")
        robot = sdk.create_robot(SPOT_IP)
        robot_id = robot.get_id()
        print(f"  gRPC auth    OK — robot serial {robot_id.serial_number}")
        username = os.environ.get("SPOT_USER", "")
        password = os.environ.get("SPOT_PASSWORD", "")
        robot.authenticate(username, password)
        print(f"  authenticate OK")
    except Exception as e:
        print(f"  gRPC auth    FAIL — {e}")
else:
    print("  gRPC auth    SKIP (tcp failed)")

# ── Vicon ──────────────────────────────────────────────────────────────────
print()
print(SEP)
print(f"  Vicon  {VICON_IP}:{VICON_PORT}")
print(SEP)

ok, rtt = ping(VICON_IP)
print(f"  ping         {'OK  ' + rtt if ok else 'FAIL — host unreachable'}")

ok801 = tcp(VICON_IP, VICON_PORT)
print(f"  tcp :801     {'OK' if ok801 else 'FAIL — port closed or filtered'}")

print(f"  route        {interface_for(VICON_IP)}")

if ok801:
    print("  SDK connect  testing…")
    try:
        from vicon.sdk import ViconSDKClient, SUCCESS
        c = ViconSDKClient()
        r = c.connect(f"{VICON_IP}:{VICON_PORT}")
        if r == SUCCESS:
            c.enable_segment_data()
            c.enable_marker_data()
            for _ in range(5): c.get_frame(); time.sleep(0.05)
            nsub = c.get_subject_count()
            print(f"  SDK connect  OK — {nsub} subjects")
            for i in range(nsub):
                name = c.get_subject_name(i)
                print(f"               [{i}] {name}")
            c.disconnect()
        else:
            print(f"  SDK connect  FAIL (result={r}) — Tracker DataStream protocol rejected")
            print(f"               Port is open but Vicon handshake failed.")
            print(f"               → Restart Vicon Tracker on Windows PC")
            print(f"               → Or SDK dylib version mismatch with Tracker")
        c.destroy()
    except Exception as e:
        print(f"  SDK connect  ERROR — {e}")
else:
    print("  SDK connect  SKIP (tcp failed)")

print()
print(SEP)
