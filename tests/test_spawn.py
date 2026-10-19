import json

import pytest
from jsonschema import Draft202012Validator

from bullet_msgs import schema_bytes
from bullet_msgs.spawn import validate_request, validate_state


def request():
    return dict(schema_version=1, physics_id='physics', episode_id='episode', controller_id='godot',
                request_id='apply1', issued_ms=1000, scenario_name='Hill', position_m=[12, -34, 25.125])


def state():
    return dict(schema_version=1, physics_id='physics', episode_id='episode', controller_id=None,
                request_id=None, result='idle', reason='startup', scenario_name='Default', position_m=[0, 0, .015])


def test_shared_spawn_schemas():
    for packet, validator, name in [(request(), validate_request, 'spawn_request.json'),
                                     (state(), validate_state, 'spawn_state.json')]:
        assert validator(packet) is packet
        Draft202012Validator(json.loads(schema_bytes(name))).validate(packet)
    packet = dict(state(), request_id='apply1', result='applied')
    validate_state(packet)
    Draft202012Validator(json.loads(schema_bytes('spawn_state.json'))).validate(packet)


@pytest.mark.parametrize('change', [dict(schema_version=True), dict(issued_ms=True), dict(request_id='a/b'),
    dict(scenario_name=' '), dict(scenario_name=' bad'), dict(scenario_name='a\nb'), dict(position_m=[0, 0, float('nan')]),
    dict(position_m=[251, 0, 1]), dict(position_m=[0, -251, 1]), dict(position_m=[0, 0, 0]),
    dict(position_m=[0, 0, 501]), dict(position_m=[0, True, 10]), dict(extra=1)])
def test_bad_spawn_requests(change):
    with pytest.raises(ValueError):
        validate_request(dict(request(), **change))


@pytest.mark.parametrize('change', [dict(result='applied'), dict(result='other'), dict(request_id='request'),
    dict(controller_id='bad/id'), dict(reason=''), dict(position_m=[0, 0, float('inf')])])
def test_bad_spawn_state(change):
    with pytest.raises(ValueError):
        validate_state(dict(state(), **change))


@pytest.mark.parametrize('factory,validator,name', [(request, validate_request, 'spawn_request.json'),
                                                   (state, validate_state, 'spawn_state.json')])
def test_spawn_v2_takeoff(factory, validator, name):
    packet = dict(factory(), schema_version=2, takeoff_height_m=7.5)
    validator(packet)
    Draft202012Validator(json.loads(schema_bytes(name))).validate(packet)
    for value in (0, -1, 100.1, True, float('nan'), float('inf'), '6'):
        with pytest.raises(ValueError):
            validator(dict(packet, takeoff_height_m=value))
    packet.pop('takeoff_height_m')
    with pytest.raises(ValueError):
        validator(packet)
