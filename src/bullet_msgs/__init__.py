"""Shared schemas and POSIX SHM images for the local Bullet pipeline."""
from importlib.resources import files
import json
import re


def schema_bytes(name):
    if name not in ("config.json", "pose.json", "image.json", "command.json", "state.json", "bbox.json", "flight_rc.json", "flight_config.json", "flight_feedback.json", "mission.json", "collision.json", "scenario_end.json", "los_telemetry.json", "spawn_request.json", "spawn_state.json"):
        raise ValueError("unknown bullet_msgs schema")
    return files("bullet_msgs").joinpath("schemas", name).read_bytes()


def validate_endpoint(endpoint):
    """Validate a local TCP endpoint shared by transport and app publishers."""
    match = re.fullmatch(r"tcp://127\.0\.0\.1:([1-9][0-9]{0,4})", endpoint) if isinstance(endpoint, str) else None
    if not match or not 1 <= int(match[1]) <= 65535:
        raise ValueError("expected loopback TCP endpoint tcp://127.0.0.1:<port 1-65535>")
    return endpoint


def validate_transport(profile):
    required = {"telemetry_endpoint", "image_endpoint", "control_endpoint"}
    if not isinstance(profile, dict) or not required <= set(profile) or set(profile) - required - {"flight_command_endpoint"}:
        raise ValueError("expected telemetry_endpoint, image_endpoint and control_endpoint")
    for endpoint in profile.values():
        validate_endpoint(endpoint)
    if len(set(profile.values())) != len(profile):
        raise ValueError("endpoints must differ")
    return profile


def load_transport(path):
    with open(path) as stream:
        return validate_transport(json.load(stream))
