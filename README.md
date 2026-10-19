# Bullet message contracts

This project is a member of the [Bullet workspace](../README.md). All six
projects share `../.venv` and `../uv.lock`; run uv commands with the
workspace flags below to retain every project and optional dependency.

Shared JSON schemas and local Linux image transport for `bullet_drone`,
`bullet_render`, `mcap_bagger`, and `video_viewer`. Requires Python 3.12, uv,
and cooperating processes under the same Linux user. The native header requires C++20.

```bash
uv sync --locked --all-packages --all-extras --all-groups
uv run --locked --all-packages --all-extras --all-groups pytest -q
```

Python projects use an editable `../bullet_msgs` uv source. The renderer includes
`include/bullet_msgs/shm.hpp` directly from this sibling checkout. No generated
bindings or runtime Python bridge are involved. `schema_bytes("pose.json")`,
`schema_bytes("config.json")`, `schema_bytes("image.json")`,
`schema_bytes("command.json")`, and `schema_bytes("state.json")` expose packaged
contracts. Recorder schema references may use `bullet_msgs:pose.json`.
`schema_bytes("bbox.json")` exposes the red-target bounding-box contract.

## TCP endpoints

`load_transport(path)` validates JSON containing exactly `telemetry_endpoint` and
`image_endpoint`, and `control_endpoint`, with distinct `tcp://127.0.0.1:<port>` values (ports 1–65535).
Defaults are 5555, 5556, and 5557 respectively. Physics binds 5555; Godot and MCAP both subscribe.
Godot binds 5556; MCAP and the video viewer subscribe. No broker or registration
service is required. The pixels remain on the same host in POSIX SHM.

Telemetry retains two message parts: UTF-8 topic and JSON payload. Image signaling
uses one complete ZMQ message, allowing a viewer to use `CONFLATE=1` safely:

```
UTF-8 exact topic + NUL | uint32 LE JSON length | UTF-8 descriptor JSON
```

Topics are nonempty, wildcard-free, NUL-free and at most 4096 UTF-8 bytes. Metadata
is at most 1024 bytes, making the whole notification at most 5125 bytes. Consumers
subscribe to the topic including its NUL terminator. `decode_descriptor` also
checks exact identity and length. Old multipart socket-pixel messages are rejected.

## Pose version 2

Physics publishes `render/drone_1/pose` on telemetry TCP port 5555. Pose
`schema_version: 2` requires `linear_velocity_m_s`: three numeric world-frame
components `[vx, vy, vz]` in **m/s**, with Bullet Z up. This is ground truth from
PyBullet's base velocity, taken from the same snapshot as position/orientation;
it is not calculated from position differences or transformed into the body frame.
The existing position, orientation, identity, sequence and simulation-time fields
remain present. Velocity components must lie within +/-1e9, and runtime JSON
validation rejects non-finite values.

Update/restart physics, the recorder and the rebuilt native renderer together.
Strict consumers reject old pose v1 streams or packets missing velocity. Config,
image and control schema versions remain 1, and no endpoint changes. Existing
MCAP recordings remain readable using their embedded schemas; they are not
rewritten to add velocity.

## Version 1 image protocol

`image.json` documents descriptor fields. Cross-field checks additionally require
`bytes == step * height`, `step >= width * 3`, at most 64 MiB, and exact SHM name
`/bullet_msgs_<32 lowercase hex session characters>_s<slot>`. Slots are 0, 1, 2.
Encoding is `rgb8` or `bgr8`, including row padding. Sequence starts at 1 for each
new image publisher session; it is independent of physics pose sequence and MCAP
`frame_id`. `capture_ns` is monotonic producer-local time, never Unix time. The
native renderer records the selected request's receipt time; Python demo writers
use write-start time. It is not an exact image/pose synchronization timestamp.

The 128-byte header is explicitly little endian, Python `struct.Struct("<8s6I4Q64s")`:

| Offset | Field | Type |
| --- | --- | --- |
| 0 | Magic `BMSHM01` plus NUL | 8 bytes |
| 8 | Version, slot, width, height, encoding (1 RGB8, 2 BGR8), row stride | 6 uint32 |
| 32 | Pixel byte count, sequence, capture_ns, completion (0 incomplete, 1 complete) | 4 uint64 |
| 64 | ASCII session, NUL padded | 64 bytes |
| 128 | Pixel rows, including padding | `step * height` bytes |

Each publisher owns three separate objects with mode 0600 and this exact size.
A writer tries rotating slots with `flock(LOCK_EX | LOCK_NB)`, invalidates the old
completion flag before touching pixels, writes, commits, unlocks, then notifies.
All slots busy means skip without waiting. Native publication and demo sends are
nonblocking with zero linger. Successful PUB submission does not prove delivery.

`FrameReader(topic).read(message)` returns `(metadata, owned_pixel_bytes)` or
`None` for missing, busy, incomplete, reused or duplicate frames, incrementing
`skipped`. Malformed descriptors, object layouts or headers raise errors. Readers
validate names, owner, regular-file type, exact permissions and size, take a
nonblocking shared lock, validate the header, and copy before unlocking. Mapping
caches hold at most three slots and are cleared on session changes. `close()`
releases mappings/descriptors. Consumers must never retain an unlocked SHM view.

`FrameWriter(width, height, encoding="rgb8", step=None)` is the Python demo/test
producer. `write(topic, pixels, capture_ns=None)` returns a descriptor message or
`None` when all slots are busy. Its context manager closes and unlinks owned slots.
The C++ `bullet_msgs::FrameWriter` produces tightly packed RGB8 for the renderer.

## Failure and cleanup

Late consumers receive future notifications only. Slow consumers may miss a
frame even after receiving its notification. OS locks release on process exit;
a killed reader cannot strand a slot. Normal publisher shutdown unlinks only its
three owned objects. Already-open mappings survive unlink until readers close.
Unique sessions prevent a restarted publisher from reusing an old slot identity.

An abrupt publisher kill can leave orphan names. After confirming the publisher
has stopped, remove only its three exact `/dev/shm/bullet_msgs_<session>_sN` names,
using its logged session ID. Do not delete other runs' objects. The transport is
for trusted local processes; advisory locking requires cooperation. This still
copies GPU pixels to CPU, into SHM, and into each Python reader's private bytes.

## Simulation control (no acknowledgments)

Godot binds `control_endpoint` (loopback TCP 5557). Physics and future consumers
subscribe to exact `simulation/command` and `simulation/heartbeat` topics, using
two-part topic/JSON messages. `command.json` carries `schema_version`,
`controller_id`, monotonic `sequence`, `action` (start/stop/reset/heartbeat), and
`issued_ms` (Linux monotonic milliseconds). Payloads are bounded to 4096 bytes.
Commands are sent once; PUB/SUB can miss them. No acknowledgment, participant
barrier, retry, or automatic rollback is provided. Another click may be needed.

The native publisher emits a heartbeat every 500 ms. Physics accepts commands only
from the controller established by a fresh heartbeat, rejects messages older than
one second or from retired controller sessions, and ignores duplicate/out-of-order
sequences. Changing controller session or missing heartbeats for three seconds
ends an active run. Reconnecting does not replay Start. A fresh heartbeat can
restore the same controller after an outage, but another Start is required.

Physics publishes exact `simulation/state` on telemetry port 5555 after transitions
and every second. `state.json` carries physics/controller/episode identities,
state (running/stopped/error), reason and simulation time. This is observable
state rather than a per-command acknowledgment. It is the future recorder hook;
MCAP command handling is not implemented in this version.

`bullet_msgs.control.ControlPublisher(endpoint)` is a Python tool/test publisher.
Its caller owns the socket and schedules `heartbeat()`; `send("start")`,
`send("stop")`, and `send("reset")` submit best-effort events. `validate_command`,
`validate_state`, and `decode_json` provide shared boundary validation. Native
constants live in `include/bullet_msgs/control.hpp`.

## Red-target bounding boxes

`bullet_app` binds loopback TCP port 5558 by default and publishes two-part
`tracking/front/bbox` + JSON messages. `bbox.json` requires schema_version 1,
image_topic, session, sequence, capture_ns, and bbox. A detected bbox contains
integer x/y (top-left pixel, x right/y down), width, and height in pixels; null
means no target in that processed frame. Session, sequence, and capture_ns
identify the source image descriptor. capture_ns is monotonic nanoseconds, not
Unix time. No image means no new bbox publication.

The recorder uses `schema: bullet_msgs:bbox.json`, `timestamp_source: receive`,
and per-topic `endpoint: tcp://127.0.0.1:5558`. This endpoint is separate from
physics telemetry and does not change the existing three-key transport profile.
`validate_endpoint()` shares loopback TCP validation with application publishers.
PUB/SUB can lose results; arrival-time RawImage MCAP records do not retain source
session/sequence, so exact image association is not guaranteed.
