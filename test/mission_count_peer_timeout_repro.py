#!/usr/bin/env python3
"""Disarmed PX4 SITL regression: foreign MISSION_COUNT must not keep an upload alive."""
import os
import time
from pymavlink import mavutil

EXPECTED = os.environ.get("EXPECT_RECOVERY", "yes") == "yes"
link = mavutil.mavlink_connection(
    "udpin:0.0.0.0:14550", source_system=245, source_component=191,
    dialect="common", autoreconnect=False,
)
heartbeat = link.wait_heartbeat(timeout=60)
assert heartbeat, "PX4 heartbeat not received on port 14550"
target_system, target_component = link.target_system, link.target_component
print(f"PX4 heartbeat: target={target_system}/{target_component}", flush=True)

def send_count(sysid, count):
    link.mav.srcSystem = sysid
    link.mav.srcComponent = 191
    link.mav.mission_count_send(target_system, target_component, count, mission_type=0)

def receive_for(sysid, seconds, types):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        msg = link.recv_match(blocking=True, timeout=min(.20, max(0.01, until-time.monotonic())))
        if msg and msg.get_type() in types and getattr(msg, "target_system", None) == sysid:
            return msg
    return None

# A starts a 2-item mission upload, then goes silent.
send_count(201, 2)
start_ack = receive_for(201, 5, {"MISSION_REQUEST_INT", "MISSION_REQUEST"})
assert start_ack is not None and start_ack.seq == 0, (
    f"A was not admitted for mission upload: {start_ack}"
)
print("A started upload and stalled before item 0", flush=True)

# B is an unrelated sender on the same MAVLink transport. Before A's
# timeout, B must be rejected. After A's timeout, B is allowed to become
# the new uploader; this admission is evidence of recovery, not a failure.
start = time.monotonic()
rejections = 0
b_admitted = False
while time.monotonic() - start < 6.5 and not b_admitted:
    send_count(202, 1)
    deadline = time.monotonic() + .20
    while time.monotonic() < deadline:
        m = link.recv_match(blocking=True, timeout=.03)
        if m is None or getattr(m, "target_system", None) != 202:
            continue
        if m.get_type() == "MISSION_ACK":
            assert m.type == mavutil.mavlink.MAV_MISSION_ERROR, (
                f"Unexpected response to B: {m}"
            )
            rejections += 1
        elif m.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            assert m.seq == 0, f"Unexpected request to B: {m}"
            b_admitted = True
            break

assert rejections >= 1, "No rejection of foreign MISSION_COUNT observed"
elapsed = time.monotonic() - start
print(f"B rejected {rejections} times; newly admitted={b_admitted} after {elapsed:.2f}s",
      flush=True)
assert b_admitted == EXPECTED, (
    f"Unexpected B admission, expected={EXPECTED}, observed={b_admitted}"
)

if b_admitted:
    # The formerly unrelated B is now the active partner. It must cancel its
    # newly accepted transfer before C can start; otherwise rejecting C is valid.
    link.mav.srcSystem = 202
    link.mav.srcComponent = 191
    link.mav.mission_ack_send(
        target_system, target_component,
        mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED, mission_type=0
    )
    time.sleep(.3)

# C must start immediately after B cancels (patched), whereas on the original
# code A still owns the transfer because rejected B packets reset its timeout.
send_count(203, 1)
response = receive_for(203, 3, {"MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"})
assert response is not None, "No response to C's mission upload"
c_admitted = response.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST") and response.seq == 0
print(f"C response={response}; admitted={c_admitted}; expected={EXPECTED}", flush=True)
assert c_admitted == EXPECTED, (
    f"Regression mismatch, expected C admission={EXPECTED} but got {response}"
)
print("PASS: baseline keeps transaction busy" if not EXPECTED else
      "PASS: patch releases timed-out transaction and permits new upload", flush=True)
