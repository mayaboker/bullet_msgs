"""Pose v2 includes world ground-truth linear velocity."""
import json

import pytest
from jsonschema import ValidationError, validate

from bullet_msgs import schema_bytes


def pose():
    return dict(schema_version=2, stream_id="stream", drone_id="drone_1",
                episode_id="episode", sequence=0, publisher_drops=0,
                step_index=0, simulation_time_s=0, position_m=[0, 0, 3],
                orientation_xyzw=[0, 0, 0, 1], linear_velocity_m_s=[1.25, -2.5, 0.75])


def test_pose_velocity_contract():
    validate(pose(), json.loads(schema_bytes("pose.json")))


@pytest.mark.parametrize("velocity", [[], [1, 2], [1, 2, 3, 4], [True, 0, 0],
                                      ["1", 0, 0], [1e10, 0, 0], [float("inf"), 0, 0]])
def test_invalid_velocity(velocity):
    value = pose()
    value["linear_velocity_m_s"] = velocity
    with pytest.raises(ValidationError):
        validate(value, json.loads(schema_bytes("pose.json")))


def test_missing_velocity_and_old_pose_version_rejected():
    value = pose()
    value.pop("linear_velocity_m_s")
    with pytest.raises(ValidationError):
        validate(value, json.loads(schema_bytes("pose.json")))
    value = pose()
    value["schema_version"] = 1
    with pytest.raises(ValidationError):
        validate(value, json.loads(schema_bytes("pose.json")))
