"""Shared flight analysis contracts stay strict at recording boundaries."""
import json
import pytest
from jsonschema import Draft202012Validator, ValidationError
from bullet_msgs import schema_bytes

SOURCE = dict(physics_id='p', episode_id='e', stream_id='s', step_index=0, simulation_time_s=0., measurement_us=1)
STATE = dict(episode_id='e', step_index=0, simulation_time_s=0., drone_id='drone_1', position_m=[0.,0.,1.], orientation_xyzw=[0.,0.,0.,1.], linear_velocity_m_s=[0.,0.,0.], angular_velocity_rad_s=[0.,0.,0.], rotor_speeds_rad_s=[0.]*4)


@pytest.mark.parametrize('name,value', [
 ('analysis_ground_truth',dict(schema_version=1,source=SOURCE,ground_truth=STATE,publisher_drops=0)),
 ('analysis_sensors',dict(schema_version=1,source=SOURCE,sensors=dict(imu=None,barometer=None,rangefinder=None),attitude_estimate=None,publisher_drops=0)),
 ('analysis_config',dict(schema_version=1,source=SOURCE,role='physics',settings={},revisions={'repo':None})),
 ('image_metadata',dict(schema_version=1,image_topic='camera/front/image',image_log_time_ns=10,image_record_sequence=0,metadata=dict(session='s',sequence=1,capture_ns=1000,source=SOURCE))),
 ('recording_health',dict(schema_version=1,complete=True,reason='closed',outcome='stop',skipped_images=0,peak_queue_bytes=0,counts={},launch_id=None)),
])
def test_analysis_schema_validates_and_rejects_unknown_fields(name,value):
    validator=Draft202012Validator(json.loads(schema_bytes(name+'.json')))
    validator.check_schema(validator.schema)
    validator.validate(value)
    with pytest.raises(ValidationError):
        validator.validate(dict(value,unexpected=True))
    with pytest.raises(ValidationError):
        validator.validate({k:v for k,v in value.items() if k!='schema_version'})


def test_revision_snapshots_report_dirty_and_unavailable(monkeypatch,tmp_path):
    from bullet_msgs import provenance
    def git(command,**kwargs):
        from pathlib import Path
        name=Path(command[2]).name
        if name=='bullet_render':
            raise OSError('git unavailable')
        if command[-1]=='HEAD':
            return 'a'*40+'\n'
        return ' M settings\n' if name=='bullet_app' else ''
    monkeypatch.setattr(provenance.subprocess,'check_output',git)
    result=provenance.revisions(tmp_path)
    assert len(result)==6
    assert result['bullet_app']=='a'*40+'+dirty'
    assert result['bullet_render'] is None
    assert result['bullet_drone']=='a'*40
