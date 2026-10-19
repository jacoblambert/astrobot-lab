"""Analytic-return contracts, not simulated scan/detection acceptance.

Run with TERRAIN_LIBRARY=/path/liblayered_terrain.so. Fixed parameter sweep;
required dimensions are never reduced when the mapper misses an object.
"""
import os
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper

LIBRARY = os.environ.get("TERRAIN_LIBRARY")
pytestmark = pytest.mark.skipif(not LIBRARY, reason="set TERRAIN_LIBRARY to built C++ shared library")


def test_worker_counts_produce_identical_maps_through_expiry():
    """Independent processes set OpenMP before load; exercise >4096 cells."""
    import subprocess
    code = r'''
import hashlib, os
import numpy as np
from layered_terrain import TerrainMapper
rng=np.random.default_rng(8173)
xy=rng.uniform(-3.95,3.95,(50000,2))
z=.015*xy[:,0]+rng.normal(0,.002,len(xy))
z[(abs(xy[:,0]-1.)<.18)&(abs(xy[:,1])<.3)]+=.1
z[(abs(xy[:,0]+1.)<.25)&(abs(xy[:,1]-.8)<.25)]-=.12
points=np.column_stack([xy,z])
digest=hashlib.sha256()
with TerrainMapper(os.environ['TERRAIN_LIBRARY'],width=80,height=80,origin=(-4.,-4.),lifetime=1.2) as mapper:
    for index,stamp in enumerate((0.,.4,.8,1.3,1.8,2.4)):
        mapper.integrate(points[index::6],stamp,[index*.08,0.,.5])
        result=mapper.project(stamp)
        digest.update(result.tobytes())
    digest.update(mapper.project(4.).tobytes())
print(digest.hexdigest())
'''
    outputs=[]
    for workers in (1,2,4,4):
        env=dict(os.environ, OMP_NUM_THREADS=str(workers), OMP_DYNAMIC="FALSE")
        env["PYTHONPATH"]=str(Path(__file__).resolve().parents[1])+os.pathsep+env.get("PYTHONPATH", "")
        outputs.append(subprocess.check_output([sys.executable,"-c",code],env=env,text=True,timeout=30).strip())
    assert len(set(outputs))==1, "worker count/scheduling changed terrain evidence"


@pytest.mark.parametrize("phase", [0.,.025,.05,.075])
@pytest.mark.parametrize("angle", [0.,.31])
def test_floor_patch_cannot_certify_a_whole_hole_straddling_cell(phase,angle):
    """No wall returns: test finite support extent, not an observed-wall halo.

    This large void exceeds the unchanged 30cm inference span. It does not
    claim detection of every smaller, entirely unobserved unsupported patch.
    """
    c,s=np.cos(angle),np.sin(angle)
    rotation=np.array([[c,-s],[s,c]])
    center=np.array([.81,.83])
    x,y=np.meshgrid(np.arange(-2.99,3.,.02),np.arange(-2.99,3.,.02))
    xy=np.column_stack([x.ravel(),y.ravel()])
    outside=np.any(np.abs((xy-center)@rotation)>=.3,axis=1)
    points=np.column_stack([xy[outside],np.zeros(outside.sum())])
    origin=np.array([-3.+phase,-3.+phase])
    with TerrainMapper(LIBRARY,width=60,height=60,origin=origin,interpolate=False) as mapper:
        mapper.integrate(points,1.,[0.,0.,.5])
        grid=mapper.project(1.)
    # Independent fine sample centers, including cells containing both rim
    # floor and unsupported space. Never restrict to a predicted-known mask.
    fx,fy=np.meshgrid(np.arange(.25,1.4,.01),np.arange(.25,1.4,.01))
    samples=np.column_stack([fx.ravel(),fy.ravel()])
    hole=np.all(np.abs((samples-center)@rotation)<.3-1e-6,axis=1)
    cells=np.floor((samples[hole]-origin)/.1).astype(int)
    costs=grid[cells[:,1],cells[:,0],0]
    assert not ((costs>=0)&(costs<100)).any(), "partial rim floor certified unsupported cell extent"
    flat_cell=np.floor((np.array([-.5,-.5])-origin)/.1).astype(int)
    assert grid[flat_cell[1],flat_cell[0],0]==0, "extent guard rejected fully supported flat terrain"


def test_spatial_support_certificates_expire_with_their_original_buckets():
    x,y=np.meshgrid(np.arange(-2.99,3.,.02),np.arange(-2.99,3.,.02))
    dense=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    tiny=np.array([[-3+(x+.5)*.1+.005*k,-3+(y+.5)*.1+.005*j,0.]
                   for y in range(1,59) for x in range(1,59) for k in range(3) for j in range(3)])
    with TerrainMapper(LIBRARY,width=60,height=60,origin=(-3.,-3.),lifetime=1.2) as retained, \
         TerrainMapper(LIBRARY,width=60,height=60,origin=(-3.,-3.),lifetime=1.2) as fresh:
        retained.integrate(dense,1.,[0.,0.,.5])
        retained.integrate(tiny,2.1,[0.,0.,.5])
        before=retained.project(2.1)
        after=retained.project(2.3)
        fresh.integrate(tiny,2.1,[0.,0.,.5])
        expected=fresh.project(2.3)
        assert np.array_equal(after,expected,equal_nan=True), "expired angular/XY support was rejuvenated"
        assert np.count_nonzero(before[:,:,0]==0)>np.count_nonzero(after[:,:,0]==0)


@pytest.mark.parametrize("height", [.05,.08,.10,.14,.20])
@pytest.mark.parametrize("width", [.05,.10,.20])
@pytest.mark.parametrize("phase", [.001,.024,.049,.074])
def test_single_ring_object_not_averaged_into_ground(height, width, phase):
    xs, ys = np.meshgrid(np.arange(-2.975,3,.05), np.arange(-2.975,3,.05))
    center = np.array([1.2+phase, .6+phase])
    on_object = (abs(xs-center[0]) <= width/2) & (abs(ys-center[1]) <= width/2)
    # No hidden floor returns underneath the object. One horizontal row of
    # top returns, independent of vertical ring count and cell-height voxels.
    floor = np.column_stack([xs[~on_object], ys[~on_object], np.zeros((~on_object).sum())])
    top_x = center[0]+np.arange(-width/2+.0025, width/2, .005)
    top = np.column_stack([top_x, np.full(len(top_x),center[1]), np.full(len(top_x),height)])
    with TerrainMapper(LIBRARY, width=60, height=60, origin=(-3.,-3.), interpolate=False) as mapper:
        mapper.integrate(np.vstack([floor,top]), 1., [0.,0.,.5])
        grid = mapper.project(1.)
    ix = np.floor((top[:,:2]+3)/.1).astype(int)
    costs = grid[ix[:,1],ix[:,0],0]
    if height >= .08:
        assert (costs == 100).any(), (height,width,phase,costs)
        assert not ((costs >= 0) & (costs < 100)).any(), "required object labelled free"
    else:
        assert not (costs == 100).any(), "5cm object incorrectly exceeds 8cm requirement"


def test_wrapper_lifetime_and_close():
    with TerrainMapper(LIBRARY) as mapper:
        with pytest.raises(ValueError):
            mapper.integrate(np.zeros((1,2)), 1., [0,0,0])
        mapper.integrate(np.empty((0,3)), 1., [0,0,0])
        with pytest.raises(ValueError):
            mapper.integrate(np.empty((0,3)), 1., [0,0,0])
    with pytest.raises(RuntimeError):
        mapper.project(1.)


@pytest.mark.parametrize("start,lifetime",[(0.,.05),(1.1,1.2),(1000000.1,30.)])
def test_cached_expiry_preserves_boundary_empty_map_and_same_stamp_reintegration(start,lifetime):
    x,y=np.meshgrid(np.arange(-2.99,3.,.02),np.arange(-2.99,3.,.02))
    floor=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    # Find the final representable time that satisfies the original predicate,
    # not an assumed equivalence to a rounded start+lifetime deadline.
    boundary=start+lifetime
    while boundary-start>lifetime:
        boundary=np.nextafter(boundary,-np.inf)
    expired=np.nextafter(boundary,np.inf)
    while expired-start<=lifetime:
        expired=np.nextafter(expired,np.inf)
    with TerrainMapper(LIBRARY,width=60,height=60,origin=(-3.,-3.),lifetime=lifetime) as mapper:
        assert (mapper.project(start)[:,:,0]==-1).all()
        mapper.integrate(floor,start,[0.,0.,.5])
        assert mapper.project(boundary)[25,25,0]==0
        assert (mapper.project(expired)[:,:,0]==-1).all()
        assert (mapper.project(expired)[:,:,0]==-1).all()
        mapper.integrate(floor,expired,[0.,0.,.5])
        assert mapper.project(expired)[25,25,0]==0
        assert (mapper.project(expired+lifetime+1.)[:,:,0]==-1).all()


@pytest.mark.parametrize("phase", [0., .023, .071])
@pytest.mark.parametrize("angle", [0., .31])
@pytest.mark.parametrize("width", [.2, .6])
def test_observed_hole_walls_remain_unsafe_across_grid_phases(phase, angle, width):
    """Analytic observed-wall contract, not proof for invisible negative edges."""
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    center = np.array([.8 + phase, .8 + phase])
    x, y = np.meshgrid(np.arange(-1.99, 2., .02), np.arange(-1.99, 2., .02))
    xy = np.column_stack([x.ravel(), y.ravel()])
    local = (xy - center) @ rotation
    outside = np.any(abs(local) >= width / 2, axis=1)
    floor = np.column_stack([xy[outside], np.zeros(outside.sum())])
    along = np.arange(-width / 2 + .002, width / 2, .005)
    walls = []
    for sign in (-1, 1):
        for axis in (0, 1):
            edge = np.column_stack([along, np.full(len(along), sign * (width / 2 - .001))])
            if axis:
                edge = edge[:, ::-1]
            world = edge @ rotation.T + center
            for z in (-.002, -.018, -.06, -.12):
                walls.append(np.column_stack([world, np.full(len(world), z)]))
    # Independent fine-grid hole points sample the exact output cells. No GT
    # dilation or candidate-derived mask; include partially covered edge cells.
    hx, hy = np.meshgrid(np.arange(-width / 2 + .001, width / 2, .005),
                         np.arange(-width / 2 + .001, width / 2, .005))
    hole = np.column_stack([hx.ravel(), hy.ravel()]) @ rotation.T + center
    indices = np.floor((hole + 2.) / .1).astype(int)
    with TerrainMapper(LIBRARY, width=40, height=40, origin=(-2., -2.)) as mapper:
        for stamp in (1., 1.5, 30.5, 31.5):
            mapper.integrate(np.vstack([floor, *walls]), stamp, [0., 0., .5])
            grid = mapper.project(stamp)
            costs = grid[indices[:, 1], indices[:, 0], 0]
            assert not ((costs >= 0) & (costs < 100)).any(), (phase, angle, width, stamp)
            assert grid[10, 10, 0] == 0, "edge guard erased distant measured ground"


def test_visible_floor_below_a_known_suspended_front_edge_is_not_drivable():
    x, y = np.meshgrid(np.arange(-2.99, 3., .02), np.arange(-2.99, 3., .02))
    floor = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    # Only the front edge is visible; there are no invented ceiling returns
    # behind it. Floor rays can pass underneath this edge.
    edge_y, edge_z = np.meshgrid(np.arange(-.5, .51, .01), [.4, .45, .5])
    edge = np.column_stack([np.full(edge_y.size, 1.01), edge_y.ravel(), edge_z.ravel()])
    with TerrainMapper(LIBRARY, width=60, height=60, origin=(-3., -3.)) as mapper:
        mapper.integrate(np.vstack([floor, edge]), 1., [0., 0., .395])
        grid = mapper.project(1.)
        assert grid[30, 45, 0] == -1 and grid[30, 45, 1] == 10, "floor under hidden roof became drivable"
        assert grid[30, 35, 0] == 0, "guard erased unobstructed foreground"
        assert grid[45, 45, 0] == 0, "guard became radial obstacle dilation"
        assert grid[30, 40, 0] == 100, "guard erased observed positive hazard"
        mapper.integrate(floor, 32., [0., 0., .395])
        assert mapper.project(32.)[30, 45, 0] == 0, "expired edge left an immortal clearance shadow"


def test_low_rock_does_not_cast_an_infinite_clearance_shadow():
    x, y = np.meshgrid(np.arange(-2.99, 3., .02), np.arange(-2.99, 3., .02))
    floor = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    edge = np.column_stack([np.full(51, 1.01), np.linspace(-.25, .25, 51), np.full(51, .08)])
    with TerrainMapper(LIBRARY, width=60, height=60, origin=(-3., -3.)) as mapper:
        mapper.integrate(np.vstack([floor, edge]), 1., [0., 0., .5])
        assert mapper.project(1.)[30, 55, 0] == 0, "ground sightline above a low rock was rejected"


@pytest.mark.parametrize("sensor_x,inside", [(-3.,True),(2.999,True),(-3.001,False),(3.,False)])
def test_empty_occupancy_clearance_fast_path_preserves_grid_boundary_guard(sensor_x,inside):
    x,y=np.meshgrid(np.arange(-2.99,3.,.02),np.arange(-2.99,3.,.02))
    floor=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    with TerrainMapper(LIBRARY,width=60,height=60,origin=(-3.,-3.)) as mapper:
        mapper.integrate(floor,1.,[sensor_x,0.,.5])
        grid=mapper.project(1.)
    assert not (grid[:,:,0]==100).any()
    if inside:
        assert np.count_nonzero(grid[:,:,0]==0)>1000
    else:
        assert not ((grid[:,:,0]>=0)&(grid[:,:,0]<100)).any()
        assert np.count_nonzero(grid[:,:,1]==10)>1000, "outside origin bypassed fail-closed clearance"


@pytest.mark.parametrize("height", [.045, .06, .068])
def test_partial_raised_face_is_not_a_measured_object_top(height):
    x, y = np.meshgrid(np.arange(-1.99, 2., .02), np.arange(-1.99, 2., .02))
    floor = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    # A single row on an object's face can precede its top by many scans.
    # Repeating it, or also seeing nearby floor, cannot establish its height.
    face = np.column_stack([np.linspace(.51, .59, 15), np.full(15, .55), np.full(15, height)])
    top_x, top_y = np.meshgrid(np.linspace(.51, .59, 5), np.linspace(.51, .59, 5))
    with TerrainMapper(LIBRARY, width=40, height=40, origin=(-2., -2.)) as mapper:
        for stamp in (1., 1.5, 2., 30.5, 31.5):
            mapper.integrate(np.vstack([floor, face]), stamp, [0., 0., .5])
            grid = mapper.project(stamp)
            assert grid[25, 25, 0] == -1 and grid[25, 25, 1] == 7
            assert grid[10, 10, 0] == 0
        top = np.column_stack([top_x.ravel(), top_y.ravel(), np.full(top_x.size, .08)])
        mapper.integrate(np.vstack([floor, top]), 32., [0., 0., .5])
        assert mapper.project(32.)[25, 25, 0] == 100


def test_resolved_five_centimeter_top_is_not_a_required_hazard():
    x, y = np.meshgrid(np.arange(-1.99, 2., .02), np.arange(-1.99, 2., .02))
    floor = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    top_x, top_y = np.meshgrid(np.linspace(.51, .59, 5), np.linspace(.51, .59, 5))
    top = np.column_stack([top_x.ravel(), top_y.ravel(), np.full(top_x.size, .05)])
    with TerrainMapper(LIBRARY, width=40, height=40, origin=(-2., -2.)) as mapper:
        mapper.integrate(np.vstack([floor, top]), 1., [0., 0., .5])
        assert mapper.project(1.)[25, 25, 0] == 20


@pytest.mark.parametrize("edge_height", [.2, .5])
def test_historical_ground_view_clears_only_low_shadows_and_expires(edge_height):
    x, y = np.meshgrid(np.arange(-2.99, 3., .02), np.arange(-2.99, 3., .02))
    floor = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    edge = np.column_stack([np.full(51, 1.01), np.linspace(-.25, .25, 51), np.full(51, edge_height)])
    with TerrainMapper(LIBRARY, width=60, height=60, origin=(-3., -3.)) as mapper:
        # At y=1.5 both the recorded endpoint and cell-center rays pass outside
        # the full occupied 10cm cell, including its far corner (not just the
        # thinner analytic edge). At y=1.0 that far corner remains occluded.
        mapper.integrate(np.vstack([floor, edge]), 1., [0., 1.5, .5])
        mapper.integrate(np.vstack([floor, edge]), 2., [0., 0., .5])
        assert mapper.project(2.)[30, 45, 0] == (0 if edge_height == .2 else -1)
        # Re-observation of the edge/current ground cannot rejuvenate the old
        # unobstructed viewpoint. Tall geometry never uses that bypass at all.
        mapper.integrate(np.vstack([floor, edge]), 32., [0., 0., .5])
        assert mapper.project(32.)[30, 45, 0] == -1


def test_unobserved_viewpoint_cannot_clear_a_low_shadow():
    x, y = np.meshgrid(np.arange(-2.99, 3., .02), np.arange(-2.99, 3., .02))
    floor = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    edge = np.column_stack([np.full(51, 1.01), np.linspace(-.25, .25, 51), np.full(51, .2)])
    with TerrainMapper(LIBRARY, width=60, height=60, origin=(-3., -3.)) as mapper:
        mapper.integrate(np.empty((0, 3)), 1., [0., 1.5, .5])
        mapper.integrate(np.vstack([floor, edge]), 2., [0., 0., .5])
        assert mapper.project(2.)[30, 45, 0] == -1


@pytest.mark.parametrize("phase", [.001, .008])
def test_two_scan_rings_on_a_vertical_corner_are_not_two_ground_planes(phase):
    x, y = np.meshgrid(np.arange(-1.99, 2., .02), np.arange(-1.99, 2., .02))
    keep=~((x>=.5)&(x<.6)&(y>=.5)&(y<.6))
    floor=np.column_stack([x[keep],y[keep],np.zeros(keep.sum())])
    along=np.linspace(.51+phase,.59,12)
    corner=np.vstack([np.column_stack([np.full(12,.51+phase),along]),
                      np.column_stack([along,np.full(12,.59)])])
    # Each ring is perfectly planar with an apparently traversable normal,
    # and spans two XY directions. Together they resolve the upright corner.
    low=.006+.2*(corner[:,1]-.51)-.05*(corner[:,0]-.51)
    rings=np.vstack([np.column_stack([corner,low]),np.column_stack([corner,low+.04])])
    with TerrainMapper(LIBRARY,width=40,height=40,origin=(-2.,-2.)) as mapper:
        for stamp in (1.,2.,31.5):
            mapper.integrate(np.vstack([floor,rings]),stamp,[0.,0.,.5])
            grid=mapper.project(stamp)
            assert grid[25,25,0]==-1, "height splitting erased the joint vertical normal"
            assert grid[10,10,0]==0


def test_unresolved_positive_extent_does_not_certify_adjacent_partial_cells():
    x,y=np.meshgrid(np.arange(-1.99,2.,.02),np.arange(-1.99,2.,.02))
    floor=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    face=np.column_stack([np.linspace(.51,.59,15),np.full(15,.55),np.full(15,.06)])
    with TerrainMapper(LIBRARY,width=40,height=40,origin=(-2.,-2.)) as mapper:
        mapper.integrate(np.vstack([floor,face]),1.,[0.,0.,.5])
        grid=mapper.project(1.)
        assert grid[25,25,1]==7 and grid[25,24,1]==11 and grid[25,24,0]==-1
        assert grid[25,23,0]==0, "positive-edge guard propagated recursively"
        assert not (grid[:,:,0]==100).any(), "uncertainty inflated occupied space"
        mapper.integrate(floor,32.,[0.,0.,.5])
        assert mapper.project(32.)[25,24,0]==0, "expired extent uncertainty remained immortal"


def test_origin_metadata_capacity_can_only_remove_historical_clearance():
    x,y=np.meshgrid(np.arange(-2.99,3.,.02),np.arange(-2.99,3.,.02))
    floor=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    edge=np.column_stack([np.full(51,1.01),np.linspace(-.25,.25,51),np.full(51,.2)])
    with TerrainMapper(LIBRARY,width=60,height=60,origin=(-3.,-3.)) as mapper:
        mapper.integrate(np.vstack([floor,edge]),1.,[0.,1.5,.5])
        mapper.integrate(np.vstack([floor,edge]),2.,[0.,0.,.5])
        assert mapper.project(2.)[30,45,0]==0
        for i in range(4096):
            mapper.integrate(np.array([[99.,99.,0.]]),2.0001+i*.0001,[0.,0.,.5])
        assert mapper.project(2.5)[30,45,0]==-1, "evicted origin was invented or borrowed from another tick"


@pytest.mark.parametrize("lifetime", [.05,.5,1.,1.2,3.])
def test_fractional_lifetime_respects_the_immutable_bucket_capacity(lifetime):
    with TerrainMapper(LIBRARY,width=10,height=10,origin=(-.5,-.5),lifetime=lifetime) as mapper:
        for i in range(150):
            mapper.integrate(np.array([[.01,.01,0.]]),.019+i*.037,[0.,0.,.5])
            mapper.project(.019+i*.037)
