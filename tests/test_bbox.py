import json

import pytest
from jsonschema import ValidationError, validate

from bullet_msgs import schema_bytes


def packet():
    return dict(schema_version=1, image_topic="camera/front/image", session="a" * 32,
                sequence=1, capture_ns=123, bbox=dict(x=10, y=20, width=30, height=40))


def test_bbox_and_no_target_contract():
    value = packet()
    schema = json.loads(schema_bytes("bbox.json"))
    validate(value, schema)
    value["bbox"] = None
    validate(value, schema)


@pytest.mark.parametrize("change", [dict(schema_version=True), dict(sequence=0),
                                    dict(sequence=True), dict(capture_ns=-1),
                                    dict(capture_ns=2**64), dict(session="invalid"),
                                    dict(image_topic=""), dict(extra=1),
                                    dict(bbox={"x": -1, "y": 0, "width": 1, "height": 1}),
                                    dict(bbox={"x": 0, "y": 0, "width": 0, "height": 1}),
                                    dict(bbox={"x": True, "y": 0, "width": 1, "height": 1}),
                                    dict(bbox={"x": 0, "y": 0, "width": 1.5, "height": 1})])
def test_invalid_bbox_contract(change):
    value = packet()
    value.update(change)
    with pytest.raises(ValidationError):
        validate(value, json.loads(schema_bytes("bbox.json")))


def test_required_dimensions():
    value = packet()
    del value["bbox"]["height"]
    with pytest.raises(ValidationError):
        validate(value, json.loads(schema_bytes("bbox.json")))
