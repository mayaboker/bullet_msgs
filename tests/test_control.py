import json
import time

import pytest
from jsonschema import validate
from bullet_msgs import schema_bytes
from bullet_msgs.control import validate_command, validate_state, decode_json


def command(**changes):
    return dict(schema_version=1, controller_id='test-controller', sequence=1,
                action='start', issued_ms=int(time.monotonic()*1000), **changes)


def test_command_and_state_contracts():
    value = command()
    assert validate_command(value) == value
    validate(value, json.loads(schema_bytes('command.json')))
    state = dict(schema_version=1, physics_id='physics', controller_id=None,
                 episode_id='episode', state='stopped', reason='startup', simulation_time_s=0)
    assert validate_state(state) == state
    validate(state, json.loads(schema_bytes('state.json')))


@pytest.mark.parametrize('changes', [dict(schema_version=True), dict(sequence=0), dict(sequence=True),
    dict(action='pause'), dict(issued_ms=-1), dict(controller_id='bad/id'), dict(extra=1)])
def test_bad_command(changes):
    value = command()
    value.update(changes)
    with pytest.raises(ValueError):
        validate_command(value)


@pytest.mark.parametrize('payload', [b'', b'{}'*3000, b'\xff', b'{"value":NaN}'])
def test_bad_json(payload):
    with pytest.raises((ValueError, UnicodeError)):
        decode_json(payload)
