"""Live world wind settings and physics-authoritative readback."""
from .control import identity, MAX_SEQUENCE
from .flight import _shape, _integer, _number

REQUEST_TOPIC = b'simulation/wind_request'
STATE_TOPIC = b'simulation/wind_state'
WIND_INTERVAL_S = 0.1
SETTINGS_FIELDS = {'enabled', 'behavior', 'velocity_world_m_s', 'gust_stddev_m_s', 'correlation_time_s', 'seed'}
REQUEST_FIELDS = {'schema_version', 'physics_id', 'episode_id', 'controller_id', 'request_id', 'issued_ms', 'settings'}
STATE_FIELDS = {'schema_version', 'physics_id', 'episode_id', 'controller_id', 'request_id', 'settings',
                'velocity_world_m_s', 'simulation_time_s', 'result', 'reason'}


def _vector(value, nonnegative=False):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('wind requires three components')
    for number in value:
        _number(number)
        if nonnegative and number < 0:
            raise ValueError('gust standard deviations must be nonnegative')


def validate_settings(value):
    _shape(value, SETTINGS_FIELDS)
    if type(value['enabled']) is not bool or value['behavior'] not in ('constant', 'gusts'):
        raise ValueError('invalid wind enable/behavior')
    _vector(value['velocity_world_m_s'])
    _vector(value['gust_stddev_m_s'], True)
    _number(value['correlation_time_s'])
    if value['correlation_time_s'] <= 0:
        raise ValueError('wind correlation time must be positive')
    _integer(value['seed'], -MAX_SEQUENCE, MAX_SEQUENCE)
    return value


def _header(value):
    _integer(value['schema_version'], 1, 1)
    for key in ('physics_id', 'episode_id'):
        if not identity(value[key]):
            raise ValueError('invalid wind identity')
    validate_settings(value['settings'])


def validate_request(value):
    _shape(value, REQUEST_FIELDS)
    _header(value)
    for key in ('controller_id', 'request_id'):
        if not identity(value[key]):
            raise ValueError('invalid wind request identity')
    _integer(value['issued_ms'])
    return value


def validate_state(value):
    _shape(value, STATE_FIELDS)
    _header(value)
    for key in ('controller_id', 'request_id'):
        if value[key] is not None and not identity(value[key]):
            raise ValueError('invalid wind state identity')
    _vector(value['velocity_world_m_s'])
    _number(value['simulation_time_s'])
    if value['simulation_time_s'] < 0 or value['result'] not in ('idle', 'applied', 'rejected'):
        raise ValueError('invalid wind state')
    if (value['request_id'] is None) != (value['result'] == 'idle'):
        raise ValueError('wind result requires a request identity')
    if not isinstance(value['reason'], str) or not 0 < len(value['reason']) <= 256:
        raise ValueError('invalid wind reason')
    return value
