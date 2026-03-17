"""
Raw Vicon dump — prints everything Vicon Tracker is currently streaming.

Usage:
    python tests/raw_vicon.py 169.254.217.218
"""

import sys
import time

from pyvicon_datastream import PyViconDatastream, Result, StreamMode

host = sys.argv[1] if len(sys.argv) > 1 else "169.254.217.218"
if ":" not in host:
    host += ":801"

print(f"Connecting to {host} ...")
client = PyViconDatastream()
client.connect(host)
client.enable_segment_data()
client.enable_marker_data()
client.set_stream_mode(StreamMode.ClientPull)

for _ in range(50):
    if client.get_frame() == Result.Success:
        break
    time.sleep(0.1)
else:
    print("No frames received.")
    client.disconnect()
    sys.exit(1)

print("Connected. Available methods:\n")
methods = [m for m in dir(client) if not m.startswith("_")]
for m in methods:
    print(f"  {m}")

client.disconnect()
