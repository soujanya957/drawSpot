"""
ctypes wrapper around libViconDataStreamSDK_C.dylib.

Calling conventions discovered by probing:
  - Count/number functions: restype=None, last arg is OutUInt* output struct
  - Translation functions:  restype=None, last arg is OutTranslation* output struct
  - String functions:       restype=c_int (CEnum), last arg is char buffer
  - OutTranslation needs _pack_=4 (no 8-byte alignment padding after Result)

Result enum (IDataStreamClientBase.h):
  Unknown=0, NotImplemented=1, Success=2, ...

StreamMode enum:
  ClientPull=0, ClientPullPreFetch=1, ServerPush=2
"""

from __future__ import annotations

import ctypes
import os
from pathlib import Path
from typing import Optional, Tuple

# ---------------------------------------------------------------------------
# Locate the dylib
# ---------------------------------------------------------------------------

_SEARCH_PATHS = [
    Path(__file__).parent / "libViconDataStreamSDK_C.dylib",
    Path.home() / "Downloads/ViconDataStreamSDK_1.13.0+167154h"
      / "ViconDataStreamSDK_1.13.0+167154h_Mac/Mac/libViconDataStreamSDK_C.dylib",
    Path(os.environ.get("VICON_SDK_PATH", "/nonexistent")),
]

_lib: Optional[ctypes.CDLL] = None


def _load_lib() -> ctypes.CDLL:
    global _lib
    if _lib is not None:
        return _lib
    for p in _SEARCH_PATHS:
        if p.exists():
            _lib = ctypes.CDLL(str(p))
            _configure(_lib)
            return _lib
    raise RuntimeError(
        "Cannot find libViconDataStreamSDK_C.dylib.\n"
        "Set VICON_SDK_PATH env var to its directory, or copy it next to vicon/sdk.py."
    )


# ---------------------------------------------------------------------------
# Output structs
# ---------------------------------------------------------------------------

class _OutUInt(ctypes.Structure):
    """{ Result(int), Value(uint) } — 8 bytes, no padding needed."""
    _fields_ = [("Result", ctypes.c_int),
                ("Value",  ctypes.c_uint)]


class _OutTranslation(ctypes.Structure):
    """
    { Result(int), Translation[3](double), Occluded(bool) }
    _pack_=4 suppresses the 4-byte padding ctypes would insert after Result
    to align double — the C SDK uses 4-byte alignment here.
    """
    _pack_   = 4
    _fields_ = [("Result",      ctypes.c_int),
                ("Translation", ctypes.c_double * 3),
                ("Occluded",    ctypes.c_bool)]


class _OutQuaternion(ctypes.Structure):
    """{ Result(int), Rotation[4](double), Occluded(bool) }"""
    _pack_   = 4
    _fields_ = [("Result",   ctypes.c_int),
                ("Rotation", ctypes.c_double * 4),
                ("Occluded", ctypes.c_bool)]


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SUCCESS              = 2
CLIENT_PULL          = 0
CLIENT_PULL_PREFETCH = 1
SERVER_PUSH          = 2

_BUF = 256


# ---------------------------------------------------------------------------
# Configure argtypes / restypes
# ---------------------------------------------------------------------------

def _configure(lib: ctypes.CDLL) -> None:
    vp = ctypes.c_void_p
    cp = ctypes.c_char_p
    ui = ctypes.c_uint
    i  = ctypes.c_int
    pU = ctypes.POINTER(_OutUInt)
    pT = ctypes.POINTER(_OutTranslation)
    pQ = ctypes.POINTER(_OutQuaternion)

    # lifecycle — return int (CEnum)
    lib.Client_Create.restype  = vp;   lib.Client_Create.argtypes  = []
    lib.Client_Destroy.restype = None; lib.Client_Destroy.argtypes = [vp]
    lib.Client_Connect.restype    = i; lib.Client_Connect.argtypes    = [vp, cp]
    lib.Client_Disconnect.restype = i; lib.Client_Disconnect.argtypes = [vp]

    # enable / disable (return int)
    for name in ("EnableSegmentData", "EnableMarkerData", "EnableUnlabeledMarkerData",
                 "DisableSegmentData", "DisableMarkerData"):
        f = getattr(lib, f"Client_{name}")
        f.restype = i; f.argtypes = [vp]

    # stream mode / frame (return int)
    lib.Client_SetStreamMode.restype  = i; lib.Client_SetStreamMode.argtypes = [vp, i]
    lib.Client_GetFrame.restype       = i; lib.Client_GetFrame.argtypes      = [vp]

    # frame number  → f(h, OutUInt*)
    lib.Client_GetFrameNumber.restype  = None
    lib.Client_GetFrameNumber.argtypes = [vp, pU]

    # subject count  → f(h, OutUInt*)
    lib.Client_GetSubjectCount.restype  = None
    lib.Client_GetSubjectCount.argtypes = [vp, pU]

    # subject name  → f(h, index, bufLen, buf)  returns int
    lib.Client_GetSubjectName.restype  = i
    lib.Client_GetSubjectName.argtypes = [vp, ui, ui, cp]

    # root segment name  → f(h, subject, bufLen, buf)  returns int
    lib.Client_GetSubjectRootSegmentName.restype  = i
    lib.Client_GetSubjectRootSegmentName.argtypes = [vp, cp, ui, cp]

    # segment count  → f(h, subject, OutUInt*)
    lib.Client_GetSegmentCount.restype  = None
    lib.Client_GetSegmentCount.argtypes = [vp, cp, pU]

    # segment name  → f(h, subject, index, bufLen, buf)  returns int
    lib.Client_GetSegmentName.restype  = i
    lib.Client_GetSegmentName.argtypes = [vp, cp, ui, ui, cp]

    # marker count  → f(h, subject, OutUInt*)
    lib.Client_GetMarkerCount.restype  = None
    lib.Client_GetMarkerCount.argtypes = [vp, cp, pU]

    # marker name  → f(h, subject, index, bufLen, buf)  returns int
    lib.Client_GetMarkerName.restype  = i
    lib.Client_GetMarkerName.argtypes = [vp, cp, ui, ui, cp]

    # translations  → f(h, subject, segment/marker, OutTranslation*)
    lib.Client_GetSegmentGlobalTranslation.restype  = None
    lib.Client_GetSegmentGlobalTranslation.argtypes = [vp, cp, cp, pT]

    lib.Client_GetMarkerGlobalTranslation.restype  = None
    lib.Client_GetMarkerGlobalTranslation.argtypes = [vp, cp, cp, pT]

    # quaternion  → f(h, subject, segment, OutQuaternion*)
    lib.Client_GetSegmentGlobalRotationQuaternion.restype  = None
    lib.Client_GetSegmentGlobalRotationQuaternion.argtypes = [vp, cp, cp, pQ]


# ---------------------------------------------------------------------------
# High-level Python client
# ---------------------------------------------------------------------------

class ViconSDKClient:
    """Drop-in replacement for pyvicon-datastream using the official C SDK dylib."""

    def __init__(self) -> None:
        self._lib = _load_lib()
        self._h = self._lib.Client_Create()
        if not self._h:
            raise RuntimeError("Client_Create() returned null")

    def __enter__(self): return self
    def __exit__(self, *_): self.disconnect(); self.destroy()

    # ---- lifecycle --------------------------------------------------------

    def connect(self, host: str) -> int:
        return self._lib.Client_Connect(self._h, host.encode())

    def disconnect(self) -> None:
        self._lib.Client_Disconnect(self._h)

    def destroy(self) -> None:
        self._lib.Client_Destroy(self._h)
        self._h = None

    # ---- setup ------------------------------------------------------------

    def enable_segment_data(self):          self._lib.Client_EnableSegmentData(self._h)
    def enable_marker_data(self):           self._lib.Client_EnableMarkerData(self._h)
    def enable_unlabeled_marker_data(self): self._lib.Client_EnableUnlabeledMarkerData(self._h)

    def set_stream_mode(self, mode: int = CLIENT_PULL):
        self._lib.Client_SetStreamMode(self._h, mode)

    # ---- frame ------------------------------------------------------------

    def get_frame(self) -> int:
        return self._lib.Client_GetFrame(self._h)

    def get_frame_number(self) -> Optional[int]:
        out = _OutUInt()
        self._lib.Client_GetFrameNumber(self._h, ctypes.byref(out))
        return out.Value if out.Result == SUCCESS else None

    # ---- subjects ---------------------------------------------------------

    def get_subject_count(self) -> int:
        out = _OutUInt()
        self._lib.Client_GetSubjectCount(self._h, ctypes.byref(out))
        return out.Value if out.Result == SUCCESS else 0

    def get_subject_name(self, index: int) -> Optional[str]:
        buf = ctypes.create_string_buffer(_BUF)
        r = self._lib.Client_GetSubjectName(self._h, index, _BUF, buf)
        return buf.value.decode() if r == SUCCESS else None

    def get_subject_root_segment_name(self, subject: str) -> Optional[str]:
        buf = ctypes.create_string_buffer(_BUF)
        r = self._lib.Client_GetSubjectRootSegmentName(
            self._h, subject.encode(), _BUF, buf)
        return buf.value.decode() if r == SUCCESS else None

    # ---- segments ---------------------------------------------------------

    def get_segment_count(self, subject: str) -> int:
        out = _OutUInt()
        self._lib.Client_GetSegmentCount(self._h, subject.encode(), ctypes.byref(out))
        return out.Value if out.Result == SUCCESS else 0

    def get_segment_name(self, subject: str, index: int) -> Optional[str]:
        buf = ctypes.create_string_buffer(_BUF)
        r = self._lib.Client_GetSegmentName(
            self._h, subject.encode(), index, _BUF, buf)
        return buf.value.decode() if r == SUCCESS else None

    def get_segment_global_translation(
        self, subject: str, segment: str
    ) -> Tuple[Optional[list], bool]:
        out = _OutTranslation()
        self._lib.Client_GetSegmentGlobalTranslation(
            self._h, subject.encode(), segment.encode(), ctypes.byref(out))
        if out.Result != SUCCESS:
            return None, True
        return list(out.Translation), bool(out.Occluded)

    def get_segment_global_rotation_quaternion(
        self, subject: str, segment: str
    ) -> Tuple[Optional[list], bool]:
        out = _OutQuaternion()
        self._lib.Client_GetSegmentGlobalRotationQuaternion(
            self._h, subject.encode(), segment.encode(), ctypes.byref(out))
        if out.Result != SUCCESS:
            return None, True
        return list(out.Rotation), bool(out.Occluded)

    # ---- markers ----------------------------------------------------------

    def get_marker_count(self, subject: str) -> int:
        out = _OutUInt()
        self._lib.Client_GetMarkerCount(self._h, subject.encode(), ctypes.byref(out))
        return out.Value if out.Result == SUCCESS else 0

    def get_marker_name(self, subject: str, index: int) -> Optional[str]:
        buf = ctypes.create_string_buffer(_BUF)
        r = self._lib.Client_GetMarkerName(
            self._h, subject.encode(), index, _BUF, buf)
        return buf.value.decode() if r == SUCCESS else None

    def get_marker_global_translation(
        self, subject: str, marker: str
    ) -> Tuple[Optional[list], bool]:
        out = _OutTranslation()
        self._lib.Client_GetMarkerGlobalTranslation(
            self._h, subject.encode(), marker.encode(), ctypes.byref(out))
        if out.Result != SUCCESS:
            return None, True
        return list(out.Translation), bool(out.Occluded)
