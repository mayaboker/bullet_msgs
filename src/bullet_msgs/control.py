"""Best-effort simulation commands, heartbeats and authoritative physics state."""
import json
import re
import time
from uuid import uuid4

import zmq

COMMAND_TOPIC = b'simulation/command'
HEARTBEAT_TOPIC = b'simulation/heartbeat'
STATE_TOPIC = b'simulation/state'
HEARTBEAT_INTERVAL_S = 0.5
HEARTBEAT_TIMEOUT_S = 3.0
COMMAND_MAX_AGE_S = 1.0
STATE_INTERVAL_S = 1.0
MAX_CONTROL_BYTES = 4096
MAX_SEQUENCE = 2**53 - 1
ACTIONS = ('start', 'stop', 'reset', 'heartbeat')


def identity(value):
    return isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_-]{1,128}', value) is not None


def decode_json(payload):
    if not 0 < len(payload) <= MAX_CONTROL_BYTES:
        raise ValueError('control payload exceeds size limit')
    def invalid(value):
        raise ValueError(f'invalid JSON constant {value}')
    return json.loads(payload.decode('utf-8'), parse_constant=invalid)


def validate_command(value):
    if not isinstance(value, dict) or set(value) != {'schema_version', 'controller_id', 'sequence', 'action', 'issued_ms'}:
        raise ValueError('invalid command fields')
    if type(value['schema_version']) is not int or value['schema_version'] != 1 or not identity(value['controller_id']):
        raise ValueError('invalid command version or controller identity')
    for field in ('sequence', 'issued_ms'):
        if type(value[field]) is not int or not 0 <= value[field] <= MAX_SEQUENCE:
            raise ValueError(f'invalid command {field}')
    if value['sequence'] == 0 or value['action'] not in ACTIONS:
        raise ValueError('invalid command action or sequence')
    return value


def validate_state(value):
    fields = {'schema_version', 'physics_id', 'controller_id', 'episode_id', 'state', 'reason', 'simulation_time_s'}
    version = value.get('schema_version') if isinstance(value, dict) else None
    if version == 2:
        fields = fields | {'terminal_result'}
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('invalid simulation state fields')
    if type(value['schema_version']) is not int or value['schema_version'] not in (1, 2) or not identity(value['physics_id']):
        raise ValueError('invalid state version or physics identity')
    if value['controller_id'] is not None and not identity(value['controller_id']):
        raise ValueError('invalid state controller identity')
    if not identity(value['episode_id']) or value['state'] not in ('running', 'stopped', 'error'):
        raise ValueError('invalid episode or state')
    if not isinstance(value['reason'], str) or not 0 < len(value['reason']) <= 256:
        raise ValueError('invalid state reason')
    elapsed = value['simulation_time_s']
    if type(elapsed) not in (int, float) or not 0 <= elapsed <= 1e12:
        raise ValueError('invalid simulation time')
    if version == 2 and value['terminal_result'] is not None:
        from .scenario import validate_end
        end = validate_end(value['terminal_result'])
        final = end['final_source']
        if (value['state'] != 'stopped' or value['reason'] != end['reason']
                or any(value[k] != final[k] for k in ('physics_id', 'episode_id', 'simulation_time_s'))):
            raise ValueError('invalid terminal simulation state')
    return value


class ControlPublisher:
    """Python publisher for application-side checks; caller schedules heartbeats."""
    def __init__(self, endpoint):
        self.controller_id = uuid4().hex
        self.sequence = 0
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        try:
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.SNDHWM, 8)
            self.socket.bind(endpoint)
        except BaseException:
            self.close()
            raise

    def send(self, action):
        self.sequence += 1
        value = dict(schema_version=1, controller_id=self.controller_id, sequence=self.sequence,
                     action=action, issued_ms=int(time.monotonic() * 1000))
        validate_command(value)
        topic = HEARTBEAT_TOPIC if action == 'heartbeat' else COMMAND_TOPIC
        self.socket.send_multipart([topic, json.dumps(value).encode()], flags=zmq.DONTWAIT)
        return value

    def heartbeat(self):
        return self.send('heartbeat')

    def close(self):
        self.socket.close(linger=0)
        self.context.term()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
