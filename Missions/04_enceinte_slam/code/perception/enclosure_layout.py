"""Physical enclosure reference. Its outline never replaces measured LiDAR walls."""
import math

def validate_layout(value):
 if not isinstance(value,dict):raise ValueError('Placer le repère de l’enceinte sur la carte.')
 width,depth,origin=value.get('width'),value.get('depth'),value.get('origin')
 if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not .5<=v<=3. for v in (width,depth)):raise ValueError('Dimensions de l’enceinte : entre 0,5 et 3 m.')
 if not isinstance(origin,(list,tuple)) or len(origin)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in origin):raise ValueError('Cliquer sur le coin de l’enceinte puis tirer vers sa profondeur.')
 return dict(width=float(width),depth=float(depth),origin=[float(v) for v in origin])

def local_point(layout,point):
 x,y,a=layout['origin'];dx,dy=point[0]-x,point[1]-y;c,s=math.cos(a),math.sin(a)
 return dx*c+dy*s,-dx*s+dy*c

def contains(layout,point,margin=0.):
 x,y=local_point(layout,point)
 return margin<=x<=layout['depth']-margin and margin<=y<=layout['width']-margin

def outside(layout,point,radius=0.):
 x,y=local_point(layout,point)
 return math.hypot(max(0.,-x,x-layout['depth']),max(0.,-y,y-layout['width']))>radius

def validate_enclosure_route(layout,route,pose,radius):
 layout=validate_layout(layout)
 if not pose or not outside(layout,pose,radius):raise ValueError('Départ requis à l’extérieur de l’enceinte, avec la marge du robot.')
 if not outside(layout,route['entry'],radius):raise ValueError('Repère A : placer l’approche à l’extérieur, avant l’ouverture.')
 if not contains(layout,route['inside'],radius):raise ValueError('Repère B : placer le point intérieur dans l’enceinte avec sa marge.')
 if not outside(layout,route['exit'],radius):raise ValueError('Repère C : placer l’arrivée entièrement à l’extérieur après la sortie.')
 return layout
