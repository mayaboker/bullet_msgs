"""Renderer collision proposals and authoritative terminal episode results."""
from typing import TypedDict

from .flight import _id, _integer, _number, _shape, _version, validate_source


COLLISION_TOPIC = b'render/drone_1/collision'
END_TOPIC = b'scenario/ended'
COLLISION_MAX_AGE_S = 1.0
COLLISION_RETRY_S = 0.1
TERMINAL_DRAIN_S = 0.25
TARGET_ID = 'red_truck'


class CollisionReport(TypedDict):
    schema_version: int
    event_id: str
    controller_id: str
    issued_us: int
    source: dict
    target_id: str
    drone_position_m: list[float]


class ScenarioEnd(TypedDict):
    schema_version: int
    event_id: str
    collision: CollisionReport
    final_source: dict
    outcome: str
    reason: str
    ended_us: int


def validate_collision(value):
    _shape(value, set(CollisionReport.__annotations__))
    _version(value['schema_version'])
    _id(value['event_id'])
    _id(value['controller_id'])
    _integer(value['issued_us'])
    validate_source(value['source'])
    if value['target_id'] != TARGET_ID or value['issued_us'] < value['source']['measurement_us']:
        raise ValueError('invalid collision target/time')
    p = value['drone_position_m']
    if not isinstance(p, (list, tuple)) or len(p) != 3:
        raise ValueError('invalid collision position')
    for component in p:
        _number(component)
    return value


def validate_end(value):
    _shape(value, set(ScenarioEnd.__annotations__))
    _version(value['schema_version'])
    _id(value['event_id'])
    collision = validate_collision(value['collision'])
    final = validate_source(value['final_source'])
    _integer(value['ended_us'])
    contact = collision['source']
    if (value['event_id'] != collision['event_id'] or value['outcome'] != 'success' or value['reason'] != 'target_hit'
            or any(final[k] != contact[k] for k in ('physics_id', 'episode_id', 'stream_id'))
            or final['step_index'] < contact['step_index'] or final['simulation_time_s'] < contact['simulation_time_s']
            or not contact['measurement_us'] <= final['measurement_us'] <= value['ended_us']
            or collision['issued_us'] > value['ended_us']):
        raise ValueError('invalid terminal result association')
    return value
