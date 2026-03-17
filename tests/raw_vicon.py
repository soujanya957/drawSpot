"""
Raw Vicon dump — prints all subjects and markers using the C SDK ctypes wrapper.

Usage:
    python tests/raw_vicon.py 169.254.217.218
    python tests/raw_vicon.py 169.254.217.218 SubjectName1 SubjectName2
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vicon.sdk import ViconSDKClient, SUCCESS, CLIENT_PULL  # noqa: E402

host = sys.argv[1] if len(sys.argv) > 1 else "169.254.217.218"
if ":" not in host:
    host += ":801"

subjects = sys.argv[2:] if len(sys.argv) > 2 else []

print(f"Connecting to {host} ...")
with ViconSDKClient() as c:
    r = c.connect(host)
    if r != SUCCESS:
        sys.exit(f"connect FAILED (result={r})")
    print(f"connect: OK ({r})")

    c.enable_segment_data()
    c.enable_marker_data()
    c.set_stream_mode(CLIENT_PULL)

    for _ in range(50):
        if c.get_frame() == SUCCESS:
            break
        time.sleep(0.05)
    else:
        sys.exit("No frames received.")

    n_subjects = c.get_subject_count()
    print(f"Frame #{c.get_frame_number()}  |  {n_subjects} subject(s)\n")

    # If no subjects specified, enumerate them
    if not subjects:
        for i in range(n_subjects):
            name = c.get_subject_name(i)
            if name:
                subjects.append(name)

    for subject in subjects:
        print(f"Subject: '{subject}'")
        root = c.get_subject_root_segment_name(subject)
        print(f"  root segment : {root!r}")

        trans, occ = c.get_segment_global_translation(subject, root or subject)
        if trans:
            print(f"  translation  : {[round(x, 2) for x in trans]}  occluded={occ}")
        else:
            print("  translation  : None")

        rot, rocc = c.get_segment_global_rotation_quaternion(subject, root or subject)
        if rot:
            print(f"  quaternion   : {[round(x, 4) for x in rot]}  occluded={rocc}")

        n = c.get_marker_count(subject)
        print(f"  markers ({n}):")
        for i in range(n):
            mname = c.get_marker_name(subject, i)
            pos, mocc = c.get_marker_global_translation(subject, mname)
            if pos:
                print(f"    [{i}] '{mname}'  {[round(x, 2) for x in pos]}  occ={mocc}")
            else:
                print(f"    [{i}] '{mname}'  None")
        print()
