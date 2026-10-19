import fcntl
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

import pytest
from jsonschema import validate

from bullet_msgs import schema_bytes, validate_transport
from bullet_msgs.shm import (FrameReader, FrameWriter, HEADER, decode_descriptor,
                             encode_descriptor, topic_prefix)

TOPIC = 'camera/front/image'
PIXELS = bytes(range(12))


def test_protocol_roundtrip_padding_private_copy_and_schema():
    reader = FrameReader(TOPIC)
    with FrameWriter(2, 2, 'bgr8', step=8) as writer:
        message = writer.write(TOPIC, bytes(range(16)), capture_ns=42)
        metadata, pixels = reader.read(message)
        validate(metadata, json.loads(schema_bytes('image.json')))
        assert HEADER.size == 128 and metadata['step'] == 8 and metadata['capture_ns'] == 42
        for _ in range(3):
            writer.write(TOPIC, bytes(16))
        assert pixels == bytes(range(16))
        assert reader.read(message) is None
        assert reader.skipped == 1
    reader.close()


@pytest.mark.parametrize('change', [dict(version=True), dict(version=2), dict(slot=3),
    dict(session='../bad'), dict(shm_name='/other'), dict(bytes=13), dict(step=5),
    dict(sequence=0), dict(capture_ns=-1), dict(width=0), dict(encoding='jpeg'), dict(encoding=[]), dict(extra=1)])
def test_invalid_descriptor(change):
    with FrameWriter(2, 2) as writer:
        metadata = decode_descriptor(TOPIC, writer.write(TOPIC, PIXELS))
        metadata.update(change)
        with pytest.raises(ValueError):
            encode_descriptor(TOPIC, metadata)


@pytest.mark.parametrize('message', [b'', b'camera/front/image/extra\0',
    topic_prefix(TOPIC) + struct.pack('<I', 2048),
    topic_prefix(TOPIC) + struct.pack('<I', 2) + b'{}extra',
    topic_prefix(TOPIC) + struct.pack('<I', 1) + b'\xff'])
def test_bad_notification(message):
    with pytest.raises((ValueError, UnicodeError)):
        decode_descriptor(TOPIC, message)


def test_overwritten_incomplete_and_header_mismatch():
    reader = FrameReader(TOPIC)
    with FrameWriter(2, 2) as writer:
        stale = writer.write(TOPIC, PIXELS)
        for _ in range(3):
            current = writer.write(TOPIC, PIXELS)
        assert reader.read(stale) is None
        metadata = decode_descriptor(TOPIC, current)
        fd = writer.slots[metadata['slot']][1]
        os.pwrite(fd, bytes(8), 56)
        assert reader.read(current) is None
        os.pwrite(fd, (1).to_bytes(8, 'little'), 56)
        os.pwrite(fd, bytes(8), 48)
        with pytest.raises(ValueError, match='timestamp'):
            reader.read(current)
    reader.close()


def test_invalid_object_permissions_size_symlink_and_header():
    with FrameWriter(2, 2) as writer:
        message = writer.write(TOPIC, PIXELS)
        metadata = decode_descriptor(TOPIC, message)
        path, fd, _ = writer.slots[metadata['slot']]
        os.fchmod(fd, 0o644)
        reader = FrameReader(TOPIC)
        with pytest.raises(ValueError, match='permissions'):
            reader.read(message)
        os.fchmod(fd, 0o600)
        os.ftruncate(fd, HEADER.size + 13)
        with pytest.raises(ValueError, match='size'):
            reader.read(message)
        os.ftruncate(fd, HEADER.size + 12)
        path.unlink()
        path.symlink_to('/dev/null')
        with pytest.raises(OSError):
            reader.read(message)
        path.unlink()
    with FrameWriter(2, 2) as writer:
        message = writer.write(TOPIC, PIXELS)
        metadata = decode_descriptor(TOPIC, message)
        os.pwrite(writer.slots[metadata['slot']][1], b'BADMAGIC', 0)
        with pytest.raises(ValueError, match='header'):
            reader.read(message)
    reader.close()


def test_busy_slots_killed_reader_releases_lock_and_clean_unlink():
    with FrameWriter(2, 2) as writer:
        paths = [path for path, _, _ in writer.slots]
        # Child owns independent file descriptions, as real subscribers do.
        child = subprocess.Popen([sys.executable, '-c',
            'import fcntl,os,sys,time; fds=[os.open(p,os.O_RDONLY) for p in sys.argv[1:]]; '
            '[fcntl.flock(fd,fcntl.LOCK_SH) for fd in fds]; print("ready",flush=True); time.sleep(30)',
            *map(str, paths)], stdout=subprocess.PIPE, text=True)
        try:
            assert child.stdout.readline().strip() == 'ready'
            assert writer.write(TOPIC, PIXELS) is None
            child.kill()
            child.wait(5)
            assert writer.write(TOPIC, PIXELS) is not None
        finally:
            if child.poll() is None:
                child.kill()
                child.wait(5)
            child.stdout.close()
    assert not any(path.exists() for path in paths)
    writer.close()


def test_rotation_readers_restart_missing_slot_and_bounded_mappings():
    readers = [FrameReader(TOPIC), FrameReader(TOPIC)]
    with FrameWriter(2, 2) as first:
        for _ in range(9):
            message = first.write(TOPIC, PIXELS)
            assert [reader.read(message)[1] for reader in readers] == [PIXELS, PIXELS]
        assert all(len(reader.mappings) == 3 for reader in readers)
        locked_index = first.next_slot
        fd = os.open(first.slots[locked_index][0], os.O_RDONLY)
        fcntl.flock(fd, fcntl.LOCK_SH)
        try:
            message = first.write(TOPIC, PIXELS)
            assert decode_descriptor(TOPIC, message)['slot'] != locked_index
        finally:
            os.close(fd)
    with FrameWriter(2, 2) as second:
        message = second.write(TOPIC, PIXELS)
        assert readers[0].read(message)[0]['sequence'] == 1
        assert len(readers[0].mappings) == 1
        missing = second.write(TOPIC, PIXELS)
    assert readers[0].read(missing) is None
    for reader in readers:
        reader.close()
        reader.close()


@pytest.mark.parametrize('endpoint', ['ipc://@old', 'tcp://0.0.0.0:5555',
    'tcp://localhost:5555', 'tcp://127.0.0.1:0', 'tcp://127.0.0.1:65536',
    'tcp://127.0.0.1:05555', 'tcp://127.0.0.1:5555\0'])
def test_invalid_tcp_endpoint(endpoint):
    with pytest.raises(ValueError):
        validate_transport(dict(telemetry_endpoint=endpoint, image_endpoint='tcp://127.0.0.1:5556', control_endpoint='tcp://127.0.0.1:5557'))


def test_duplicate_endpoints_and_schemas():
    with pytest.raises(ValueError, match='differ'):
        validate_transport(dict(telemetry_endpoint='tcp://127.0.0.1:5555', image_endpoint='tcp://127.0.0.1:5555', control_endpoint='tcp://127.0.0.1:5557'))
    assert [v['properties']['schema_version']['const'] for v in json.loads(schema_bytes('pose.json'))['oneOf']] == [2, 3]
    with pytest.raises(ValueError):
        schema_bytes('../pose.json')


def test_allocation_failure_cleans_partial_slots(monkeypatch):
    from types import SimpleNamespace
    from uuid import uuid4
    import bullet_msgs.shm as shm
    session = uuid4().hex
    monkeypatch.setattr(shm, 'uuid4', lambda: SimpleNamespace(hex=session))
    original = os.posix_fallocate
    calls = 0
    def allocate(fd, offset, size):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError('simulated full SHM')
        original(fd, offset, size)
    monkeypatch.setattr(os, 'posix_fallocate', allocate)
    with pytest.raises(OSError, match='full SHM'):
        FrameWriter(2, 2)
    assert not any((Path('/dev/shm') / f'bullet_msgs_{session}_s{i}').exists() for i in range(3))


def test_reader_skips_exclusively_locked_slot():
    reader = FrameReader(TOPIC)
    with FrameWriter(2, 2) as writer:
        message = writer.write(TOPIC, PIXELS)
        metadata = decode_descriptor(TOPIC, message)
        fd = os.open(writer.slots[metadata['slot']][0], os.O_RDWR)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            assert reader.read(message) is None
            fcntl.flock(fd, fcntl.LOCK_UN)
            assert reader.read(message)[1] == PIXELS
        finally:
            os.close(fd)
    reader.close()


def test_killed_publisher_leaves_incomplete_frame_unreadable():
    code = '''import fcntl,os,time
from bullet_msgs.shm import FrameWriter
w=FrameWriter(2,2)
m=w.write("camera/front/image",bytes(12))
p,fd,mem=w.slots[0]
fcntl.flock(fd,fcntl.LOCK_EX)
os.pwrite(fd,bytes(8),56)
mem[128:132]=b"oops"
print(m.hex(),flush=True)
time.sleep(30)
'''
    child = subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE, text=True)
    reader = FrameReader(TOPIC)
    paths = []
    try:
        message = bytes.fromhex(child.stdout.readline().strip())
        metadata = decode_descriptor(TOPIC, message)
        paths = [Path('/dev/shm') / f'bullet_msgs_{metadata["session"]}_s{i}' for i in range(3)]
        child.kill()
        child.wait(5)
        assert reader.read(message) is None
        assert reader.skipped == 1
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(5)
        child.stdout.close()
        reader.close()
        for path in paths:
            path.unlink(missing_ok=True)
