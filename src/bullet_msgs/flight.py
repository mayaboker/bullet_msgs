"""Local expiring RC intent, sensor feedback and capture-source contracts."""
import json
import math

from .control import MAX_SEQUENCE, identity


RC_TOPIC = b'flight/drone_1/rc'
FEEDBACK_TOPIC = b'flight/drone_1/feedback'
CONFIG_TOPIC = b'flight/drone_1/config'
MISSION_TOPIC = b'app/drone_1/mission'
MAX_FLIGHT_BYTES = 8192
COMMAND_LIFETIME_US = 100_000
CHANNEL_COUNT = 18
SOURCE_FIELDS = {'physics_id', 'episode_id', 'stream_id', 'step_index', 'simulation_time_s', 'measurement_us'}
RC_FIELDS = {'schema_version', 'physics_id', 'episode_id', 'app_id', 'sequence', 'issued_us', 'valid_until_us', 'active', 'channels'}
CONFIG_FIELDS = {'schema_version', 'physics_id', 'stream_id', 'mass_kg', 'gravity_m_s2', 'max_thrust_n',
                 'rc_angle_scale_rad', 'rc_yaw_rate_scale_rad_s', 'aetr_channels', 'arm_channel', 'mode_channel',
                 'camera', 'attitude_model', 'physics_hz', 'feedback_hz', 'command_hz'}
FEEDBACK_FIELDS = {'schema_version', 'source', 'barometer_altitude_m', 'orientation_xyzw', 'armed', 'mode',
                   'last_applied_sequence', 'owner_id', 'state', 'reason'}


def _shape(value, fields):
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('invalid flight message fields')


def _integer(value, lower=0, upper=MAX_SEQUENCE):
    if type(value) is not int or not lower <= value <= upper:
        raise ValueError('invalid flight integer')


def _number(value):
    if type(value) not in (float, int) or abs(value) > 1e12 or not math.isfinite(value):
        raise ValueError('invalid flight number')


def _id(value):
    if not identity(value):
        raise ValueError('invalid flight identity')


def _version(value):
    _integer(value, 1, 1)


def validate_source(value):
    _shape(value, SOURCE_FIELDS)
    for name in ('physics_id', 'episode_id', 'stream_id'):
        _id(value[name])
    for name in ('step_index', 'measurement_us'):
        _integer(value[name])
    _number(value['simulation_time_s'])
    if value['simulation_time_s'] < 0:
        raise ValueError('negative simulation time')
    return value


def validate_rc(value):
    _shape(value, RC_FIELDS)
    _version(value['schema_version'])
    for name in ('physics_id', 'episode_id', 'app_id'):
        _id(value[name])
    for name in ('sequence', 'issued_us', 'valid_until_us'):
        _integer(value[name], 1 if name == 'sequence' else 0)
    if not 0 < value['valid_until_us'] - value['issued_us'] <= COMMAND_LIFETIME_US:
        raise ValueError('invalid RC lifetime')
    if type(value['active']) is not bool or not isinstance(value['channels'], (list, tuple)) or len(value['channels']) != CHANNEL_COUNT:
        raise ValueError('invalid RC active/channels')
    for channel in value['channels']:
        _integer(channel, 1000, 2000)
    return value


def validate_config(value):
    _shape(value, CONFIG_FIELDS)
    _version(value['schema_version'])
    _id(value['physics_id'])
    _id(value['stream_id'])
    for key in ('mass_kg', 'gravity_m_s2', 'max_thrust_n', 'rc_angle_scale_rad', 'rc_yaw_rate_scale_rad_s',
                'physics_hz', 'feedback_hz', 'command_hz'):
        _number(value[key])
        if value[key] <= 0:
            raise ValueError('nonpositive platform value')
    if value['max_thrust_n'] <= value['mass_kg'] * value['gravity_m_s2']:
        raise ValueError('insufficient hover thrust')
    mapping = value['aetr_channels']
    if not isinstance(mapping, (list, tuple)) or len(mapping) != 4:
        raise ValueError('invalid AETR mapping')
    for channel in (*mapping, value['arm_channel'], value['mode_channel']):
        _integer(channel, 1, CHANNEL_COUNT)
    if len(set((*mapping, value['arm_channel'], value['mode_channel']))) != 6:
        raise ValueError('overlapping RC mapping')
    if value['attitude_model'] not in ('ideal', 'imu_complementary'):
        raise ValueError('unsupported attitude model')
    camera = value['camera']
    _shape(camera, {'width_px', 'height_px', 'fov_x_deg'})
    for key in ('width_px', 'height_px'):
        _integer(camera[key], 1, 8192)
    _number(camera['fov_x_deg'])
    if not 0 < camera['fov_x_deg'] < 180:
        raise ValueError('invalid FOV')
    return value


def validate_feedback(value):
    version = value.get('schema_version') if isinstance(value, dict) else None
    _shape(value, FEEDBACK_FIELDS | {'takeoff_height_m'} if version == 2 else FEEDBACK_FIELDS)
    _integer(version, 1, 2)
    if version == 2:
        from .spawn import validate_takeoff_height
        validate_takeoff_height(value['takeoff_height_m'])
    validate_source(value['source'])
    if value['barometer_altitude_m'] is not None:
        _number(value['barometer_altitude_m'])
    q = value['orientation_xyzw']
    if not isinstance(q, (list, tuple)) or len(q) != 4:
        raise ValueError('invalid attitude')
    for component in q:
        _number(component)
    if abs(sum(v * v for v in q) - 1) > 0.002:
        raise ValueError('attitude is not a unit quaternion')
    if type(value['armed']) is not bool or value['mode'] not in ('angle', 'acro'):
        raise ValueError('invalid applied arm/mode')
    _integer(value['last_applied_sequence'])
    if value['owner_id'] is not None:
        _id(value['owner_id'])
    if value['state'] not in ('stopped', 'running', 'error') or not isinstance(value['reason'], str) or not 0 < len(value['reason']) <= 256:
        raise ValueError('invalid flight state')
    return value


def decode(payload, validator):
    if not 0 < len(payload) <= MAX_FLIGHT_BYTES:
        raise ValueError('flight payload exceeds bounds')
    def reject(value):
        raise ValueError(f'invalid JSON constant: {value}')
    try:
        value = json.loads(payload.decode('utf8'), parse_constant=reject)
    except (RecursionError, OverflowError) as exc:
        raise ValueError('flight JSON exceeds parser bounds') from exc
    return validator(value)


def encode(value, validator):
    validator(value)
    raw = json.dumps(value, separators=(',', ':'), allow_nan=False).encode()
    if len(raw) > MAX_FLIGHT_BYTES:
        raise ValueError('flight payload exceeds bounds')
    return raw
