"""Focused degree-based LOS logging; telemetry does not authorize flight."""
from typing import TypedDict

from .flight import _id, _integer, _number, _shape, _version, validate_source


LOS_TOPIC = b'tracking/front/los'
LOS_HZ = 30
NAVIGATOR_STATES = ('disabled', 'acquire', 'track', 'recover', 'fault')
TRACK_STAGES = ('bearing', 'ttc', 'terminal')
ESTIMATE_FIELDS = ('elevation_deg', 'azimuth_deg', 'elevation_rate_deg_s', 'azimuth_rate_deg_s', 'optical_ttc_s')


class LosTelemetry(TypedDict):
    schema_version: int
    app_id: str
    sequence: int
    source: dict
    generated_us: int
    navigation_sequence: int | None
    navigator_state: str | None
    stage: str | None
    reason: str
    mission_phase: str
    active: bool
    elevation_deg: float | None
    azimuth_deg: float | None
    bearing_valid: bool
    elevation_rate_deg_s: float | None
    azimuth_rate_deg_s: float | None
    rate_observable: bool
    optical_ttc_s: float | None
    ttc_valid: bool
    ttc_reason: str
    ttc_relative_residual: float | None


def validate_los(value):
    _shape(value, set(LosTelemetry.__annotations__))
    _version(value['schema_version'])
    _id(value['app_id'])
    _integer(value['sequence'], 1)
    validate_source(value['source'])
    _integer(value['generated_us'])
    if value['navigation_sequence'] is not None:
        _integer(value['navigation_sequence'], 1)
    if value['navigator_state'] is not None and value['navigator_state'] not in NAVIGATOR_STATES:
        raise ValueError('invalid LOS navigator state')
    if value['stage'] is not None and value['stage'] not in TRACK_STAGES:
        raise ValueError('invalid LOS stage')
    for key in ('reason', 'mission_phase', 'ttc_reason'):
        if not isinstance(value[key], str) or not 0 < len(value[key]) <= 256:
            raise ValueError('invalid LOS state/reason')
    for key in ('active', 'bearing_valid', 'rate_observable', 'ttc_valid'):
        if type(value[key]) is not bool:
            raise ValueError('invalid LOS validity flag')
    for key in (*ESTIMATE_FIELDS, 'ttc_relative_residual'):
        if value[key] is not None:
            _number(value[key])
    pairs = (('bearing_valid', ('elevation_deg', 'azimuth_deg')),
             ('rate_observable', ('elevation_rate_deg_s', 'azimuth_rate_deg_s')),
             ('ttc_valid', ('optical_ttc_s',)))
    for flag, fields in pairs:
        if any((value[k] is not None) != value[flag] for k in fields):
            raise ValueError('LOS estimates/validity mismatch')
    if value['rate_observable'] and not value['bearing_valid']:
        raise ValueError('LOS rate requires valid bearing')
    if value['ttc_valid'] and (value['ttc_reason'] != 'qualified' or value['optical_ttc_s'] <= 0):
        raise ValueError('invalid qualified TTC')
    if value['ttc_relative_residual'] is not None and value['ttc_relative_residual'] < 0:
        raise ValueError('negative TTC residual')
    if value['active']:
        if value['navigation_sequence'] is None or value['navigator_state'] is None or value['stage'] is None:
            raise ValueError('active LOS needs result identity')
    elif (any(value[k] is not None for k in (*ESTIMATE_FIELDS, 'ttc_relative_residual'))
          or any(value[k] for k in ('bearing_valid', 'rate_observable', 'ttc_valid'))
          or value['navigation_sequence'] is not None):
        raise ValueError('inactive LOS contains stale estimates')
    return value
