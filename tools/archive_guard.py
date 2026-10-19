"""External-drive-only run allocation and quotas. Never deletes existing runs."""
import json
import os
from pathlib import Path
import shutil
import uuid

ARCHIVE_ROOT = Path('/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab')
GIB = 1024**3


def workspace_reference():
    """Resolve the real workspace, never infer its device from an install prefix.

    Standalone diagnostic packages may themselves be installed on the archive.
    Comparing against that Python file's parent rejects a perfectly valid drive.
    Require repository markers, or the explicit, existing deployment workspace.
    """
    declared = os.environ.get("ASTROBOT_WORKSPACE")
    if declared:
        candidates = [Path(declared)]
    else:
        candidates = [Path.cwd(), *Path.cwd().parents, Path("/workspace/astrobot-lab"), Path("/workspace/omnilrs"),
                      Path(__file__).resolve().parents[1]]
    for candidate in candidates:
        repository = ((candidate / "docker-compose.lunaryard.yml").is_file() and
                      (candidate / "astrobot-lab/src/slam_mapping/package.xml").is_file())
        # Isaac mounts only the OmniLRS submodule and the tools directory, not
        # the repository root. That real bind mount is its workspace reference.
        simulator = ((candidate / "run.py").is_file() and
                     (candidate / "cfg/environment/lunaryard_40m.yaml").is_file())
        if repository or simulator:
            return candidate.resolve(strict=True)
    raise ValueError("cannot resolve real workspace for external archive check; set ASTROBOT_WORKSPACE")


def check_capacity(root, required, *, workspace=None, external=True):
    root = Path(root).resolve(strict=True)
    if not root.is_dir() or not os.access(root, os.W_OK):
        raise ValueError('archive root must exist and be writable')
    if required < 0:
        raise ValueError('invalid archive capacity requirement')
    if external and root.stat().st_dev == (workspace_reference() if workspace is None else Path(workspace)).stat().st_dev:
        raise ValueError('archive is on workspace filesystem; refusing fallback')
    if external and root.stat().st_dev == Path('/').stat().st_dev:
        raise ValueError('archive is on root filesystem; refusing fallback')
    if shutil.disk_usage(root).free < required:
        raise ValueError('insufficient archive free-space reserve')
    return root


def allocate_run(root=ARCHIVE_ROOT, *, limit=64*GIB, reserve=50*GIB, external=True):
    if limit <= 0 or reserve < 0:
        raise ValueError('invalid run quota')
    root = check_capacity(root, limit+reserve, external=external)
    run = root / ('revision-' + uuid.uuid4().hex)
    run.mkdir(exist_ok=False)
    with (run/'archive_contract.json').open('x') as out:
        json.dump({'schema':1,'run':run.name,'limit_bytes':limit,'reserve_bytes':reserve,
                   'root':str(root),'device':root.stat().st_dev,'external_required':external},out,indent=2)
    return run


def archive_size(run):
    """Exact file size without constructing a Path graph for every saved tick.

    Keep the existing rglob semantics: count file symlinks, but do not descend
    through directory symlinks. Stream directory entries so live quota checks
    do not retain thousands of Path objects and trigger simulator-wide GC.
    """
    size = 0
    with os.scandir(run) as entries:
        for entry in entries:
            if entry.is_dir(follow_symlinks=False):
                size += archive_size(entry.path)
            elif entry.is_file():
                size += entry.stat().st_size
    return size


def check_run(run):
    run=Path(run)
    contract=json.loads((run/'archive_contract.json').read_text())
    root=check_capacity(contract['root'],contract['reserve_bytes'],external=contract['external_required'])
    if root.stat().st_dev != contract['device']:
        raise ValueError('archive device changed')
    size=archive_size(run)
    if size >= contract['limit_bytes']:
        raise ValueError('run archive quota exceeded')
    return size
