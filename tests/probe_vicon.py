"""
Probe Vicon using the official C SDK via ctypes (vicon/sdk.py).

Usage:
    python tests/probe_vicon.py 169.254.217.218 test_canvas
"""
import sys, time, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vicon.sdk import ViconSDKClient, SUCCESS, CLIENT_PULL

host    = sys.argv[1] if len(sys.argv) > 1 else "169.254.217.218"
subject = sys.argv[2] if len(sys.argv) > 2 else "test_canvas"
if ":" not in host:
    host += ":801"

with ViconSDKClient() as c:
    r = c.connect(host)
    print(f"connect: {'OK' if r == SUCCESS else 'FAIL'} ({r})")
    c.enable_segment_data()
    c.enable_marker_data()
    c.set_stream_mode(CLIENT_PULL)

    for _ in range(10):
        c.get_frame()
        time.sleep(0.01)

    print(f"frame    : {c.get_frame_number()}")
    print(f"subjects : {c.get_subject_count()}")
    print(f"\nSubject '{subject}'")

    root = c.get_subject_root_segment_name(subject)
    print(f"  root segment : {root!r}")

    trans, occ = c.get_segment_global_translation(subject, root or subject)
    print(f"  translation  : {[round(x,2) for x in trans]}  occluded={occ}")

    rot, rocc = c.get_segment_global_rotation_quaternion(subject, root or subject)
    print(f"  quaternion   : {[round(x,4) for x in rot]}  occluded={rocc}")

    n = c.get_marker_count(subject)
    print(f"  markers ({n}):")
    for i in range(n):
        mname = c.get_marker_name(subject, i)
        pos, mocc = c.get_marker_global_translation(subject, mname)
        print(f"    [{i}] '{mname}'  {[round(x,2) for x in pos]}  occ={mocc}")
