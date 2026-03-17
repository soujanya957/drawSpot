"""
Probe Vicon using the official C SDK via ctypes (vicon/sdk.py).

Usage:
    python -m tests.probe_vicon
    python -m tests.probe_vicon --host 169.254.217.218:801
    python -m tests.probe_vicon --host 169.254.217.218:801 --subject spot_base
"""
import argparse, sys, time, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vicon.sdk import ViconSDKClient, SUCCESS, CLIENT_PULL

ap = argparse.ArgumentParser()
ap.add_argument("--host",    default="169.254.217.218:801")
ap.add_argument("--subject", default="test_canvas")
args = ap.parse_args()

host    = args.host if ":" in args.host else args.host + ":801"
subject = args.subject

with ViconSDKClient() as c:
    r = c.connect(host)
    print(f"connect: {'OK' if r == SUCCESS else 'FAIL'} ({r})")
    if r != SUCCESS:
        print("Cannot connect — check that Vicon Tracker is open and DataStream is enabled.")
        sys.exit(1)

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
    if trans is not None:
        print(f"  translation  : {[round(x,2) for x in trans]}  occluded={occ}")
    else:
        print(f"  translation  : NO DATA")

    rot, rocc = c.get_segment_global_rotation_quaternion(subject, root or subject)
    if rot is not None:
        print(f"  quaternion   : {[round(x,4) for x in rot]}  occluded={rocc}")
    else:
        print(f"  quaternion   : NO DATA")

    n = c.get_marker_count(subject)
    print(f"  markers ({n}):")
    for i in range(n):
        mname = c.get_marker_name(subject, i)
        pos, mocc = c.get_marker_global_translation(subject, mname)
        if pos is not None:
            print(f"    [{i}] '{mname}'  {[round(x,2) for x in pos]}  occ={mocc}")
        else:
            print(f"    [{i}] '{mname}'  NO DATA")
