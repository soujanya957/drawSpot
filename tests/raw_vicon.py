"""
Raw Vicon dump — prints all markers for known subjects.

Usage:
    python tests/raw_vicon.py 169.254.217.218
    python tests/raw_vicon.py 169.254.217.218 SubjectName1 SubjectName2
"""

import sys
import time

from pyvicon_datastream import PyViconDatastream, Result, StreamMode

host = sys.argv[1] if len(sys.argv) > 1 else "169.254.217.218"
if ":" not in host:
    host += ":801"

subjects = sys.argv[2:] if len(sys.argv) > 2 else ["test_canvas"]

print(f"Connecting to {host} ...")
c = PyViconDatastream()
c.connect(host)
c.enable_segment_data()
c.enable_marker_data()
c.set_stream_mode(StreamMode.ClientPull)

for _ in range(50):
    if c.get_frame() == Result.Success:
        break
    time.sleep(0.05)
else:
    print("No frames received.")
    c.disconnect()
    sys.exit(1)

print(f"Frame #{c.get_frame_number()}  |  {c.get_subject_count()} subject(s)\n")

for subject in subjects:
    print(f"Subject: '{subject}'")
    root = c.get_subject_root_segment_name(subject)
    print(f"  root segment : {root!r}")

    trans = c.get_segment_global_translation(subject, root or subject)
    print(f"  translation  : {trans}")

    n = c.get_marker_count(subject)
    print(f"  markers      : {n}")
    for i in range(n):
        mname = c.get_marker_name(subject, i)
        mpos  = c.get_marker_global_translation(subject, mname)
        print(f"    [{i}] '{mname}'  ->  {mpos}")
    print()

c.disconnect()
