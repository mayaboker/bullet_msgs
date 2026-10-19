from copy import deepcopy
import json

import pytest
from jsonschema import validate, ValidationError
from bullet_msgs import schema_bytes
from bullet_msgs.los import validate_los
from bullet_msgs.flight import encode, decode


def packet(active=True):
    value = dict(schema_version=1, app_id='app', sequence=1,
        source=dict(physics_id='physics', episode_id='episode', stream_id='stream',
                    step_index=2, simulation_time_s=.1, measurement_us=1_000_000),
        generated_us=1_010_000, navigation_sequence=5 if active else None,
        navigator_state='track', stage='ttc', reason='tracking' if active else 'target_hit',
        mission_phase='LOS_APPROACH' if active else 'COMPLETED', active=active,
        elevation_deg=20. if active else None, azimuth_deg=-10. if active else None, bearing_valid=active,
        elevation_rate_deg_s=.5 if active else None, azimuth_rate_deg_s=-.2 if active else None,
        rate_observable=active, optical_ttc_s=3. if active else None, ttc_valid=active,
        ttc_reason='qualified' if active else 'inactive', ttc_relative_residual=.02 if active else None)
    return value


@pytest.mark.parametrize('active', [True, False])
def test_schema_and_wire(active):
    value = packet(active)
    assert decode(encode(value, validate_los), validate_los) == value
    validate(value, json.loads(schema_bytes('los_telemetry.json')))


@pytest.mark.parametrize('changes', [dict(elevation_deg=float('nan')), dict(azimuth_rate_deg_s=float('inf')),
    dict(generated_us=True), dict(app_id='bad/id'), dict(navigation_sequence=0), dict(ttc_relative_residual=-1),
    dict(optical_ttc_s=0), dict(bearing_valid=False), dict(ttc_reason='not_closing'),
    dict(active=False), dict(navigator_state='unknown'), dict(sequence=-1), dict(rate_observable=1)])
def test_invalid_payload(changes):
    with pytest.raises(ValueError):
        validate_los(dict(packet(), **changes))


def test_json_schema_rejects_inactive_data_and_validity_mismatch():
    schema = json.loads(schema_bytes('los_telemetry.json'))
    for value in (dict(packet(False), elevation_deg=5.), dict(packet(), ttc_valid=False),
                  dict(packet(), navigation_sequence=None), dict(packet(), extra=True)):
        with pytest.raises(ValidationError):
            validate(value, schema)
