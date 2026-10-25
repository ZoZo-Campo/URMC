import math,sys,json
from pathlib import Path
import numpy as np,pytest
from urmc_core.enclosure_map import Grid,astar,swept_clear,validate_route
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
from map_store import MapStore,slug

def room():
 a=np.zeros((80,100),np.int16);a[[0,-1],:]=100;a[:,[0,-1]]=100
 return Grid(a,.03,(-.3,-1.2,0.))

def test_dynamic_unmapped_obstacle_changes_path():
 g=room();start=(0.,0.);goal=(2.,0.)
 p=astar(g,start,goal,.12);q=astar(g,start,goal,.12,[(1.,v) for v in np.linspace(-.15,.15,15)])
 assert p and q
 assert max(abs(x[1]) for x in p)<.08
 assert max(abs(x[1]) for x in q)>.24

def test_unknown_and_too_narrow_are_blocked():
 g=room();g.data[:,40:45]=-1
 assert not astar(g,(0.,0.),(2.,0.),.12)
 g=room();g.data[:,40:45]=100;g.data[37:43,40:45]=0
 assert not astar(g,(0.,0.),(2.,0.),.12)

def test_no_corner_cutting():
 g=Grid(np.array([[0,100],[100,0]]),.1,(0,0,0))
 assert not astar(g,(.05,.05),(.15,.15),0.)

def test_scan_collision_stops_including_rotation():
 g=room()
 assert not swept_clear((0,0,0),.1,0,g,.12,[(.16,0)])
 assert not swept_clear((0,0,0),0,.4,g,.12,[(.1,0)])
 assert swept_clear((0,0,0),.08,0,g,.12,[(.5,.5)])

def test_rotated_map_coordinates_roundtrip():
 g=Grid(np.zeros((20,30)),.03,(2.,-1.,.8))
 for p in [(1,1),(10,15),(29,19)]:assert g.cell(*g.world(*p))==p

def test_route_validation_rejects_occupied_and_unknown_goals():
 g=room();route=dict(entry=[0,0],inside=[1,0],exit=[2,0])
 assert validate_route(route,g)
 with pytest.raises(ValueError):validate_route(dict(route,exit=[20.,0.]),g)
 with pytest.raises(ValueError):validate_route(dict(entry=[0,0]),g)

@pytest.mark.parametrize('value',['../x','/tmp/x','a/b','',None,'x'*65])
def test_map_names_cannot_escape_storage(value):
 with pytest.raises(ValueError):slug(value)

def test_map_roundtrip_export_and_no_overwrite(tmp_path):
 store=MapStore(tmp_path);p=store.folder('test');p.mkdir();g=room();meta=dict(created='2026',robot_id=11,route={})
 store.write_grid(p,g,meta);out,m=store.load('test')
 assert np.array_equal(out.data,g.data) and out.origin==g.origin
 assert store.list()[0]['has_graph'] is False
 assert store.export('test').startswith(b'PK')
 assert (p/'map.yaml').read_text().startswith('image: map.pgm')


def test_zone_forbids_circumventing_the_enclosure_outside():
 from urmc_core.enclosure_map import bounded_grid
 g=room();restricted=bounded_grid(g,[-.20,-.5,2.2,.5])
 assert astar(restricted,(0,0),(2,0),.12)
 restricted.data[:,45:50]=100
 assert not astar(restricted,(0,0),(2,0),.12)


def test_inside_goal_can_move_away_from_unmapped_obstacle():
 from urmc_core.enclosure_map import plan_goal
 g=room();points=[(.85,y) for y in np.linspace(-.14,.12,10)]
 path,target=plan_goal(g,(0,0),(.98,0),.13,points,True)
 assert path and math.dist(target,(.98,0))>.1 and math.dist(target,(.98,0))<=.251
