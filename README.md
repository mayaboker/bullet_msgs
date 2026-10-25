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

## Flight and stamped capture contracts

`bullet_msgs.flight` validates bounded RC intent, platform configuration, sensor
feedback and capture sources. Platform `attitude_model` accepts `ideal` and
`imu_complementary`; the latter identifies physics-owned IMU attitude estimation.
Feedback keeps the same body-to-world XYZW quaternion field and source stamp. JSON schemas are `flight_rc.json`,
`flight_config.json`, `flight_feedback.json` and `mission.json`. RC packets carry
18 integer channels, physics/episode/app identities, increasing sequence,
`issued_us`, `valid_until_us` and `active`. Lifetime is at most 100 ms; validators
also enforce finite values and mappings. Ids/timestamps use JSON-safe integers
through 2^53-1. Time is Linux host monotonic microseconds, not Unix time.

`flight_command_endpoint` is optional in legacy transport profiles. When present
it must be another distinct loopback TCP endpoint. The app binds it; physics
subscribes with exact topic matching. Topic constants are in `bullet_msgs.flight`.

Pose v3 adds a `source`: physics_id, episode_id, stream_id, step_index,
simulation_time_s, measurement_us. It must describe the same pose. Image
descriptor v2 carries that same source and `capture_ns=measurement_us*1000`.
The SHM binary header **remains version 1**. Legacy pose v2/descriptor v1 remain
supported. Capture time identifies the rendered source observation, not render
arrival or GPU completion. Consumers must enforce episode identity and age;
arrival-timestamped MCAP alone cannot reconstruct exact image/pose association.

Bbox v2 retains image session/sequence/capture_ns and adds the same validated
source for exact capture association in recorded JSON. Bbox v1 remains supported
for legacy detector-only streams.

## Collision and terminal scenario contracts

`bullet_msgs.scenario` exposes typed `CollisionReport` / `ScenarioEnd`, strict
validators and exact topic constants. Shared JSON schemas are `collision.json` and
`scenario_end.json`. A report on `render/drone_1/collision` carries event/controller
identity, issued monotonic microseconds, stamped source, red_truck target and the
detected drone position in Bullet world metres. The renderer uses the same event
ID on retries; IDs are unique across episodes. Physics validates freshness,
current ownership and known source history before accepting.

`scenario/ended` carries the confirmed report, success/target_hit, ended_us and a
separate final frozen source. State v2 adds required nullable `terminal_result`
with the same result; v1 is retained. Terminal results require stopped state and
consistent physics/episode/time association. Reports use 5557; results and states
use 5555. They use existing bounded JSON transport and host monotonic time.
Native JSON serialization preserves full double precision and integral stamp
fields, so a source survives a Godot round trip without losing its identity.

## Focused LOS logging contract

`bullet_msgs.los` exports `LosTelemetry`, `LOS_TOPIC` (`tracking/front/los`),
`LOS_HZ=30` and `validate_los`. `los_telemetry.json` is a strict version1 contract
for degree-based bearing/rate, TTC/confidence, state and source/generation stamps.
Use the existing bounded `flight.encode/decode` helpers. This topic does not carry
RC authority or change the library's native radian interfaces.

Every packet carries publisher sequence/app identity and a physics source stamp.
Active packets identify the originating navigation sequence and generated_us;
inactive markers use the lifecycle transition time and null navigation_sequence.
State/stage retain the last actual result until a new episode. Invalid estimates
are null with false validity flags; inactive packets clear all estimates/residual.
TTC validity requires a positive qualified estimate. Confidence is its reason and
nonnegative dimensionless relative fit residual, not a probability. The schema
and validator reject extra fields, nonfinite values and inconsistent validity.

## Stopped spawn selection

`bullet_msgs.spawn` exports `SpawnRequest`, `SpawnState`, strict validators and
exact `simulation/spawn_request` / `simulation/spawn_state` topics. Packaged
schemas are `spawn_request.json` and `spawn_state.json`; native topic/bounds
constants are in `include/bullet_msgs/spawn.hpp`. Both contracts support schema v1 and v2. V2 adds required `takeoff_height_m`
(finite 0.1–100 m relative climb); v1 requests use the 20 m default.
Requests share lifecycle 5557; status shares physics telemetry 5555.

Request fields are controller/physics/episode IDs, stable request ID, monotonic
`issued_ms`, scenario name and initial `position_m` in Bullet XYZ metres (Z up).
Names are nonblank, trimmed, at most 64 characters with no control characters.
X/Y are within ±250 m; world Z is 0.015–500 m. All numeric values must be finite.
Terrain clearance is validated by Godot against the actual landscape triangles.

Status carries current physics/controller/episode IDs, applied scenario name and
position, latest nullable request ID, result `idle|applied|rejected` and reason.
Only idle has a null request ID. Status describes configuration, not live pose;
it repeats every 100 ms even stopped. Fresh controller heartbeat and request age
<=1 s are required. Requests bind to a physics instance and stopped episode;
idempotent retries return the earlier outcome without resetting the new episode.

`flight_feedback.json` and `validate_feedback` also accept v2 feedback carrying
required `takeoff_height_m`. The external-RC runner repeats the applied value in
every feedback alongside its episode source. This lets the app latch the takeoff
goal before commanding that episode without relying on the timing of a separate
configuration advertisement. Legacy v1 feedback retains the 20 m default.

## Live wind

`bullet_msgs.wind` defines strict validators and exact topics
`simulation/wind_request` (existing lifecycle/control endpoint, default 5557) and
`simulation/wind_state` (existing telemetry endpoint, default 5555). Packaged v1
schemas are `wind_request.json` and `wind_state.json`; native constants are in
`include/bullet_msgs/wind.hpp`.

Requests contain `schema_version`, `physics_id`, `episode_id`, `controller_id`,
`request_id`, monotonic `issued_ms` and `settings`. Settings contain boolean
`enabled`, `behavior` (`constant` or `gusts`), world XYZ `velocity_world_m_s`,
nonnegative XYZ `gust_stddev_m_s`, positive `correlation_time_s` and integer `seed`.
All numbers must be finite and within shared contract bounds. Seed is bounded to
±(2^53−1) for exact Godot/JSON round trips.

States repeat identity/settings and include current world `velocity_world_m_s`,
`simulation_time_s`, nullable `request_id`, `result` (`idle`, `applied`, `rejected`)
and `reason`. The controller may be null before ownership. State is cached at
10 Hz and expires in the native consumer after 1000 ms. Reads do not sample wind.
Requests use the existing heartbeat ownership and one-second age limit, plus
bounded request-ID deduplication. Runtime physics, not the renderer, owns gusts.

## Flight analysis contracts

Packaged JSON schemas `analysis_ground_truth.json`, `analysis_sensors.json`,
`analysis_config.json`, `render_health.json`, `image_metadata.json` and
`recording_health.json` describe diagnostic-only recording data. Physics publishes
`analysis/physics/ground_truth` at 100 Hz and `analysis/physics/sensors` at its
sampling rate on telemetry; app/physics publish `analysis/<role>/config` at 1 Hz.
These snapshots retain source identities, resolved settings and Git revisions
(with `+dirty` when applicable). They do not extend sensor-only flight feedback.
Sensor values are the outputs actually exposed by the sensor implementation,
including its configured filtering. Unavailable raw values are not invented.

Godot publishes `analysis/render/health` at 1 Hz on the existing image endpoint:
source age, dropped/superseded/stale frames, sequence gaps and errors. This exact
JSON topic/schema pair is allowed beside the independent SHM descriptor input.
The recorder locally adds `<image topic>/metadata` to link original exposure
identity to MCAP image log time and recorder sequence. Final `recording_health`
is an MCAP metadata record whose map values are JSON encoded.
