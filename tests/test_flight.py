import json

import pytest
from bullet_msgs import schema_bytes
from bullet_msgs.flight import validate_rc, validate_source, decode, encode
from bullet_msgs.shm import FrameWriter, FrameReader
from jsonschema import Draft202012Validator


SOURCE = dict(physics_id='physics', episode_id='episode', stream_id='stream',
              step_index=7, simulation_time_s=.1, measurement_us=1234567)
RC = dict(schema_version=1, physics_id='physics', episode_id='episode', app_id='app',
          sequence=1, issued_us=100, valid_until_us=100100, active=True,
          channels=[1500, 1500, 1000, 1500, 1000, 2000] + [1000] * 12)


@pytest.mark.parametrize('field,value', [('sequence', True), ('valid_until_us', 100101),
    ('active', 1), ('channels', [1500] * 17), ('physics_id', 'bad/id'), ('issued_us', -1)])
def test_reject_malformed_rc(field, value):
    with pytest.raises(ValueError):
        validate_rc(dict(RC, **{field: value}))


def test_wire_schema_and_nonfinite_bounds():
    assert decode(encode(RC, validate_rc), validate_rc) == RC
    Draft202012Validator(json.loads(schema_bytes('flight_rc.json'))).validate(RC)
    with pytest.raises(ValueError):
        validate_source(dict(SOURCE, simulation_time_s=10**1000))
    with pytest.raises(ValueError):
        decode(b'{"value":NaN}', validate_rc)
    with pytest.raises(ValueError):
        decode(b'[' * 1200 + b']' * 1200, validate_rc)


def test_stamped_descriptor_preserves_source_and_binary_header():
    writer = FrameWriter(2, 2)
    reader = FrameReader('camera/front/image')
    try:
        message = writer.write('camera/front/image', b'\x01' * 12, SOURCE['measurement_us'] * 1000, source=SOURCE)
        metadata, pixels = reader.read(message)
        assert metadata['version'] == 2 and metadata['source'] == SOURCE
        assert pixels == b'\x01' * 12
        Draft202012Validator(json.loads(schema_bytes('image.json'))).validate(metadata)
        with pytest.raises(ValueError):
            writer.write('camera/front/image', b'\x01' * 12, 1, source=SOURCE)
    finally:
        reader.close()
        writer.close()
