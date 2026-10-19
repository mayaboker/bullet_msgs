"""Stopped-episode spawn selection in Bullet metres (Z up)."""
from typing import TypedDict

from .control import identity
from .flight import _shape, _integer, _number

REQUEST_TOPIC = b'simulation/spawn_request'
STATE_TOPIC = b'simulation/spawn_state'
SPAWN_INTERVAL_S = 0.1
HORIZONTAL_LIMIT_M = 250.0
MIN_ALTITUDE_M = 0.015
MAX_ALTITUDE_M = 500.0
DEFAULT_TAKEOFF_HEIGHT_M = 20.0
MIN_TAKEOFF_HEIGHT_M = 0.1
MAX_TAKEOFF_HEIGHT_M = 100.0


class SpawnRequest(TypedDict):
    schema_version: int
    controller_id: str
    physics_id: str
    episode_id: str
    request_id: str
    issued_ms: int
    scenario_name: str
    position_m: list[float]
    takeoff_height_m: float


class SpawnState(TypedDict):
    schema_version: int
    physics_id: str
    controller_id: str | None
    episode_id: str
    scenario_name: str
    position_m: list[float]
    takeoff_height_m: float
    request_id: str | None
    result: str
    reason: str


def validate_position(position):
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        raise ValueError('expected XYZ position')
    for n in position:
        _number(n)
    if any(abs(n) > HORIZONTAL_LIMIT_M for n in position[:2]) or not MIN_ALTITUDE_M <= position[2] <= MAX_ALTITUDE_M:
        raise ValueError('spawn outside landscape/altitude bounds')
    return position


def _name(name):
    if not isinstance(name, str) or not 0 < len(name) <= 64 or name != name.strip() or any(ord(c) < 32 for c in name):
        raise ValueError('invalid scenario name')


def validate_takeoff_height(value):
    _number(value)
    if not MIN_TAKEOFF_HEIGHT_M <= value <= MAX_TAKEOFF_HEIGHT_M:
        raise ValueError('takeoff height must be 0.1–100 m')
    return value


def _spawn_shape(value, fields):
    version = value.get('schema_version') if isinstance(value, dict) else None
    _shape(value, fields if version == 2 else fields - {'takeoff_height_m'})
    _integer(version, 1, 2)
    if version == 2:
        validate_takeoff_height(value['takeoff_height_m'])


def validate_request(value):
    _spawn_shape(value, set(SpawnRequest.__annotations__))
    for key in ('controller_id', 'physics_id', 'episode_id', 'request_id'):
        if not identity(value[key]):
            raise ValueError('invalid spawn identity')
    _integer(value['issued_ms'])
    _name(value['scenario_name'])
    validate_position(value['position_m'])
    return value


def validate_state(value):
    _spawn_shape(value, set(SpawnState.__annotations__))
    for key in ('physics_id', 'episode_id'):
        if not identity(value[key]):
            raise ValueError('invalid spawn identity')
    for key in ('controller_id', 'request_id'):
        if value[key] is not None and not identity(value[key]):
            raise ValueError('invalid optional spawn identity')
    _name(value['scenario_name'])
    validate_position(value['position_m'])
    if value['result'] not in ('idle', 'applied', 'rejected') or not isinstance(value['reason'], str) or not 0 < len(value['reason']) <= 256:
        raise ValueError('invalid spawn outcome')
    if (value['request_id'] is None) != (value['result'] == 'idle'):
        raise ValueError('spawn outcome requires request identity')
    return value
