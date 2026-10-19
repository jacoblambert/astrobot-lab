import sys
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from stress_terrain_resources import stress_chunks


def test_adversarial_resource_profile_fills_the_whole_grid_and_all_layers():
    chunks=stress_chunks()
    assert len(chunks)==10 and all(c.shape==(48000,3) for c in chunks)
    points=np.vstack(chunks)
    xy=np.floor((points[:,:2]+10.)/.1).astype(int)
    assert xy.min()==0 and xy.max()==199
    assert len(np.unique(xy,axis=0))==40000
    assert np.array_equal(np.unique(points[:,2]),[0.,.04,.08,.16])
    counts=np.bincount(xy[:,1]*200+xy[:,0],minlength=40000)
    assert (counts==12).all()


def test_dense_flat_resource_profile_keeps_input_budget_and_broad_ground_support():
    chunks=stress_chunks("dense_flat")
    assert len(chunks)==10 and all(c.shape==(48000,3) for c in chunks)
    points=np.vstack(chunks)
    xy=np.floor((points[:,:2]+10.)/.1).astype(int)
    assert len(np.unique(xy,axis=0))==40000 and (points[:,2]==0).all()
    assert (np.bincount(xy[:,1]*200+xy[:,0],minlength=40000)==12).all()
    first=points[(xy[:,0]==0)&(xy[:,1]==0),:2]
    assert np.allclose(np.ptp(first,axis=0),[.07,.07])
