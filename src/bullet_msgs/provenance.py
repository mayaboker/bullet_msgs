"""Application-owned revision collection; never invoked by physics models."""
from pathlib import Path
import subprocess


def revisions(workspace):
    result = {}
    for name in ('bullet_drone', 'bullet_msgs', 'bullet_render', 'bullet_app', 'los_tracking', 'mcap_bagger'):
        try:
            root = Path(workspace) / name
            commit = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], stderr=subprocess.DEVNULL, timeout=2, text=True).strip()
            dirty = subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain'], stderr=subprocess.DEVNULL, timeout=2, text=True).strip()
            result[name] = commit + ('+dirty' if dirty else '')
        except (OSError, subprocess.SubprocessError):
            result[name] = None
    return result
