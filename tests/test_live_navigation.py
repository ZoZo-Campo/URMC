import math
import cv2,numpy as np,pytest
from urmc_core.enclosure_map import Grid,astar
from urmc_core.live_navigation import LocalView,LiveGridLayer,CameraWatch,validate_destination

def grid():return Grid(np.zeros((60,90),np.int16),.04,(-.4,-1.2,0.))

def test_new_obstacle_blocks_before_next_global_map():
 g=grid();layer=LiveGridLayer();point=(.8,0.)
 assert astar(g,(0,0),(2.,0),.12)
 layer.observe(g,(0,0),[(point[0],point[1],True)],1)
 cell=g.cell(*point);assert layer.apply(g).data[cell[1],cell[0]]==100
 assert g.data[cell[1],cell[0]]==0 # Saved map remains unchanged.


def test_old_map_obstacle_needs_three_distinct_free_scans_to_clear():
 g=grid();p=g.cell(.8,0);g.data[p[1],p[0]]=100;layer=LiveGridLayer()
 for stamp in [1,1,1]:layer.observe(g,(0,0),[(1.5,0,False)],stamp)
 assert layer.apply(g).data[p[1],p[0]]==100
 layer.observe(g,(0,0),[(1.5,0,False)],2);assert layer.apply(g).data[p[1],p[0]]==100
 layer.observe(g,(0,0),[(1.5,0,False)],3);assert layer.apply(g).data[p[1],p[0]]==0 and layer.cleared==1
 assert g.data[p[1],p[0]]==100
 layer.observe(g,(0,0),[(.8,0,True)],4);assert layer.apply(g).data[p[1],p[0]]==100


def test_occluded_old_obstacle_is_not_assumed_free():
 g=grid();p=g.cell(1.,0);g.data[p[1],p[0]]=100;layer=LiveGridLayer()
 for stamp in range(5):layer.observe(g,(0,0),[(.5,0,True)],stamp)
 assert layer.apply(g).data[p[1],p[0]]==100


def test_unknown_saved_area_does_not_become_traversable_from_one_ray():
 g=grid();g.data[:]=-1;layer=LiveGridLayer()
 for stamp in range(4):layer.observe(g,(0,0),[(1.5,0,False)],stamp)
 assert np.count_nonzero(layer.apply(g).data==0)==0


def test_local_view_builds_and_clears_without_slam():
 view=LocalView();assert (view.grid.data==-1).all()
 view.observe((0,0),[(.6,0,True)],1);cell=view.grid.cell(.6,0)
 assert view.grid.data[cell[1],cell[0]]==100
 view.observe((0,0),[(1.2,0,False)],2);assert view.grid.data[cell[1],cell[0]]==0
 assert not view.observe((0,0),[(.6,0,True)],2)
 assert view.grid.data[cell[1],cell[0]]==0


def test_destination_is_the_selected_point_not_a_nearby_goal():
 g=grid();point=[1.8,.2]
 assert validate_destination({'exit':point},g,.13)==point
 assert point==[1.8,.2]
 for goal in [None,[math.nan,0],[5.,0],[True,1]]:
  with pytest.raises(ValueError):validate_destination({'exit':goal},g,.13)


def test_camera_analysis_is_quality_and_change_at_rest():
 watch=CameraWatch();base=np.full((240,320,3),60,np.uint8)
 watch.image(base,0.,1.);assert watch.report(.1)['allowed']
 changed=base.copy();cv2.rectangle(changed,(40,100),(280,230),(200,200,200),-1)
 watch.image(changed,.2,2.);assert watch.report(.21)['state']=='CHANGEMENT_A_L_ARRET' and not watch.report(.21)['allowed']
 watch.image(changed,1.2,3.);assert watch.report(1.25)['allowed']
 assert not watch.report(2.)['allowed']


def test_camera_does_not_label_ego_motion_as_an_obstacle():
 watch=CameraWatch();base=np.full((240,320,3),50,np.uint8)
 watch.image(base,0.,1.);watch.image(np.full_like(base,150),.1,2.,moving=True)
 assert watch.report(.15)['allowed'] and watch.report(.15)['change_ratio']==0
 watch.image(np.zeros_like(base),.2,3.,moving=True)
 assert watch.report(.25)['state']=='IMAGE_SOMBRE'


def test_camera_repeated_stamp_cannot_refresh_a_frozen_stream():
 watch=CameraWatch();base=np.full((240,320,3),60,np.uint8)
 watch.image(base,0.,1.);assert not watch.image(base,.8,1.)
 assert not watch.report(.8)['fresh']


def test_known_free_start_can_leave_conservative_local_inflation_only_with_scan_clearance():
 g=grid();start=(.679,.239);goal=(1.8,.3);points=[(.62,.12)]
 # A finite-size raster hit conservatively covers a cell slightly above the return.
 p=g.cell(*points[0]);g.data[p[1],p[0]]=100
 assert not astar(g,start,goal,.13,points)
 assert astar(g,start,goal,.13,points,allow_start=True)
 assert not astar(g,start,goal,.13,[(.68,.24)],allow_start=True)
