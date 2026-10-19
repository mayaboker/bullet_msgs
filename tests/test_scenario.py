from copy import deepcopy
import json

import pytest
from jsonschema import validate
from bullet_msgs import schema_bytes
from bullet_msgs.control import validate_state
from bullet_msgs.scenario import validate_collision, validate_end


def report():
    return dict(schema_version=1, event_id='hit', controller_id='godot', issued_us=1_010_000,
                source=dict(physics_id='physics', episode_id='episode', stream_id='stream',
                            step_index=2, simulation_time_s=.1, measurement_us=1_000_000),
                target_id='red_truck', drone_position_m=[50., 0., 5.])


def ending():
    r = report()
    return dict(schema_version=1, event_id='hit', collision=r,
                final_source=dict(r['source'], step_index=3, simulation_time_s=.12, measurement_us=1_020_000),
                outcome='success', reason='target_hit', ended_us=1_030_000)


def test_contracts_and_legacy_state():
    for value, name, check in ((report(), 'collision.json', validate_collision), (ending(), 'scenario_end.json', validate_end)):
        assert check(value) == value
        validate(value, json.loads(schema_bytes(name)))
    state = dict(schema_version=2, physics_id='physics', controller_id='godot', episode_id='episode',
                 state='stopped', reason='target_hit', simulation_time_s=.12, terminal_result=ending())
    assert validate_state(state) == state
    validate(state, json.loads(schema_bytes('state.json')))
    state.pop('terminal_result')
    state.update(schema_version=1)
    assert validate_state(state) == state


@pytest.mark.parametrize('changes', [dict(target_id='terrain'), dict(event_id='bad/id'), dict(issued_us=True),
    dict(issued_us=0), dict(drone_position_m=[0, 0]), dict(drone_position_m=[float('nan'), 0, 0])])
def test_malformed_report(changes):
    with pytest.raises(ValueError):
        validate_collision(dict(report(), **changes))


@pytest.mark.parametrize('changes', [dict(outcome='failure'), dict(event_id='other'), dict(ended_us=0),
    dict(final_source=dict(ending()['final_source'], episode_id='foreign'))])
def test_malformed_terminal_association(changes):
    with pytest.raises(ValueError):
        validate_end(dict(ending(), **changes))
