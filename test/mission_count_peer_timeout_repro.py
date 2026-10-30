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

# B is an unrelated component on the same MAVLink transport. It keeps
# submitting MISSION_COUNT, and PX4 is supposed to reject every one.
# Crucially, these rejected messages must NOT refresh A's timeout.
start = time.monotonic()
rejections = 0
while time.monotonic() - start < 6.5:
    send_count(202, 1)
    deadline = time.monotonic() + .20
    while time.monotonic() < deadline:
        m = link.recv_match(blocking=True, timeout=.03)
        if (m and m.get_type() == "MISSION_ACK"
                and getattr(m, "target_system", None) == 202):
            if m.type == mavutil.mavlink.MAV_MISSION_ERROR:
                rejections += 1
            else:
                raise AssertionError(f"B was unexpectedly accepted: {m}")
assert rejections >= 1, "No rejection of foreign MISSION_COUNT observed"
print(f"B requests rejected {rejections} times over 6.5 seconds", flush=True)

# C should be allowed to begin a new mission upload after A's 5-second
# protocol timeout; under the original code, B has kept A alive.
send_count(203, 1)
response = receive_for(203, 3, {"MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"})
assert response is not None, "No response to C's new mission upload"
recovered = response.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST") and response.seq == 0
print(f"C response={response}; recovered={recovered}; expected={EXPECTED}", flush=True)
assert recovered == EXPECTED, (
    f"Regression mismatch, expected recovery={EXPECTED} but got {response}"
)
print("PASS: baseline reproduces bug" if not EXPECTED else "PASS: patch restores timeout", flush=True)
