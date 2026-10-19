from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from revision_evidence import RevisionEvidence


def transform(x=0, yaw=0):
    c,s=np.cos(yaw),np.sin(yaw)
    p=np.eye(4); p[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]; p[0,3]=x
    return p


def test_nonuniform_revision_equals_fresh_reconstruction(tmp_path):
    db=RevisionEvidence(tmp_path/'revised.db','session',1,create=True)
    fresh=RevisionEvidence(tmp_path/'fresh.db','session',1,create=True)
    points=np.array([[.025,.025,0],[.035,.035,.08]],dtype=np.float32)
    for archive in (db,fresh):
        archive.add_scan(0,1_000_000_000,'vlp16',points)
        archive.add_scan(1,2_000_000_000,'vlp16',points)
    db.apply_revision(1,0,{0:transform(),1:transform(2)})
    before=db.height_evidence(1,as_of_ns=3_000_000_000)
    revised={0:transform(.3),1:transform(3,.5)}
    db.apply_revision(2,1,revised)
    fresh.apply_revision(1,0,revised)
    actual=db.height_evidence(2,as_of_ns=3_000_000_000)
    assert actual == fresh.height_evidence(1,as_of_ns=3_000_000_000)
    assert before.keys().isdisjoint(actual.keys())  # Old footprints really disappear.
    # One whole-map rigid translation cannot implement different submap changes.
    shifted={(x+3,y):v for (x,y),v in before.items()}
    assert shifted != actual
    assert db.height_evidence(2,as_of_ns=40_000_000_000) == {}  # No rejuvenation.
    db.close(); fresh.close()


def test_revision_identity_gap_snapshot_and_atomic_rejection(tmp_path):
    db=RevisionEvidence(tmp_path/'events.db','s',3,create=True)
    assert db.apply_revision(1,0,{0:transform()})
    assert not db.apply_revision(1,0,{0:transform()})
    with pytest.raises(ValueError,match='conflicting'):
        db.apply_revision(1,0,{0:transform(1)})
    with pytest.raises(ValueError,match='gap'):
        db.apply_revision(3,2,{0:transform(1)})
    invalid=np.eye(4); invalid[0,0]=2
    with pytest.raises(ValueError,match=r'SE\(3\)'):
        db.apply_revision(2,1,{0:transform(1),1:invalid})
    assert db.revision==1
    with pytest.raises(ValueError,match='omitted'):
        db.apply_revision(3,2,{1:transform()},snapshot=True)
    db.apply_revision(3,2,{0:transform(1)},snapshot=True)
    with pytest.raises(ValueError,match='deskew'):
        db.apply_revision(4,3,{0:transform(2)},deskew_changed=True)
    db.close()
    with pytest.raises(ValueError,match='epoch'):
        RevisionEvidence(tmp_path/'events.db','s',4)


def test_local_evidence_immutable_missing_pose_and_causal_cutoff(tmp_path):
    path=tmp_path/'immutable.db'
    db=RevisionEvidence(path,'s',1,create=True)
    original=np.array([[0.,0.,1.]])
    assert db.add_scan(0,10,'vlp16',original)
    assert not db.add_scan(0,10,'vlp16',original)
    original[0,2]=2
    with pytest.raises(ValueError,match='immutable'):
        db.add_scan(0,10,'vlp16',original)
    db.apply_revision(1,0,{})
    assert list(db.scans_at(1,as_of_ns=9))==[]
    with pytest.raises(ValueError,match='missing pose'):
        list(db.scans_at(1,as_of_ns=10))
    db.apply_revision(2,1,{0:transform(1)})
    frame,stamp,points,origin=next(db.scans_at(2,as_of_ns=10))
    np.testing.assert_array_equal(points,[[1,0,1]])
    np.testing.assert_array_equal(origin,[1,0,0])
    assert stamp==10
    db.close()
    reopened=RevisionEvidence(path,'s',1)
    assert reopened.revision==2
    reopened.close()
