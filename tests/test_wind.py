import json
from copy import deepcopy

import pytest
from jsonschema import Draft202012Validator
from bullet_msgs import schema_bytes
from bullet_msgs.wind import validate_request, validate_state


def request():
    return dict(schema_version=1,physics_id='physics',episode_id='episode',controller_id='godot',
                request_id='wind',issued_ms=100,settings=dict(enabled=True,behavior='gusts',
                velocity_world_m_s=[3.,0.,0.],gust_stddev_m_s=[.3,.3,.1],correlation_time_s=1.,seed=2))


def test_contracts_and_schema_agree():
    packet=request()
    validate_request(packet)
    Draft202012Validator(json.loads(schema_bytes('wind_request.json'))).validate(packet)
    state={k:v for k,v in packet.items() if k!='issued_ms'}
    state.update(velocity_world_m_s=[3.1,.2,0.],simulation_time_s=1.,result='applied',reason='wind_applied')
    schema=Draft202012Validator(json.loads(schema_bytes('wind_state.json')))
    validate_state(state)
    schema.validate(state)
    idle=dict(state,request_id=None,result='idle')
    validate_state(idle)
    schema.validate(idle)
    with pytest.raises(ValueError):
        validate_state(dict(idle,result='applied'))
    assert not schema.is_valid(dict(idle,result='applied'))


@pytest.mark.parametrize('key,value', [('enabled',1),('behavior','dryden'),('gust_stddev_m_s',[-1,0,0]),
    ('velocity_world_m_s',[0,1]),('velocity_world_m_s',[0,1e13,0]),
    ('correlation_time_s',0),('correlation_time_s',True),('seed',True),('seed',.5)])
def test_invalid_settings(key,value):
    packet=deepcopy(request())
    packet['settings'][key]=value
    with pytest.raises(ValueError):
        validate_request(packet)
    assert not Draft202012Validator(json.loads(schema_bytes('wind_request.json'))).is_valid(packet)


def test_nonfinite_is_rejected_before_wire_serialization():
    packet=request()
    packet['settings']['velocity_world_m_s']=[0.,float('nan'),0.]
    with pytest.raises(ValueError):
        validate_request(packet)
    with pytest.raises(ValueError):
        json.dumps(packet,allow_nan=False)
