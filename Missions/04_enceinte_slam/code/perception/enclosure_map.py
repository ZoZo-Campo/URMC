"""Metric grid navigation for mission 4. Unknown cells are not traversable."""
import heapq,math
import cv2,numpy as np
from .control import wrap

class Grid:
    def __init__(self,data,resolution,origin):
        self.data=np.asarray(data,dtype=np.int16)
        if self.data.ndim!=2 or not 0<resolution<=.2 or self.data.size>1_000_000:raise ValueError('Carte invalide ou trop grande.')
        if len(origin)!=3 or not all(math.isfinite(v) for v in origin):raise ValueError('Origine invalide.')
        self.res=float(resolution);self.origin=tuple(origin);self.h,self.w=self.data.shape
    def cell(self,x,y):
        dx,dy=x-self.origin[0],y-self.origin[1];c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        return math.floor((dx*c+dy*s)/self.res),math.floor((-dx*s+dy*c)/self.res)
    def world(self,x,y):
        a,b=(x+.5)*self.res,(y+.5)*self.res;c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        return self.origin[0]+a*c-b*s,self.origin[1]+a*s+b*c
    def inside(self,p):return 0<=p[0]<self.w and 0<=p[1]<self.h
    def blocked(self,radius,points=()):
        mask=((self.data<0)|(self.data>=50)).astype(np.uint8)
        for x,y in points:
            p=self.cell(x,y)
            if self.inside(p):mask[p[1],p[0]]=1
        n=math.ceil(radius/self.res)
        yy,xx=np.ogrid[-n:n+1,-n:n+1]
        kernel=(xx*xx+yy*yy<=(radius/self.res+.7072)**2).astype(np.uint8)
        return cv2.dilate(mask,kernel,borderType=cv2.BORDER_CONSTANT,borderValue=1).astype(bool)
    def png(self):
        pixels=np.full(self.data.shape,155,np.uint8);pixels[self.data==0]=245;pixels[self.data>=50]=25
        return cv2.imencode('.png',np.flipud(pixels))[1].tobytes()


def astar(grid,start,goal,radius=.12,points=(),limit=150000):
    blocked=grid.blocked(radius,points);a,b=grid.cell(*start),grid.cell(*goal)
    if not grid.inside(a) or not grid.inside(b) or blocked[a[1],a[0]] or blocked[b[1],b[0]]:return []
    heap=[(0.,0.,a)];cost={a:0.};parent={};count=0
    while heap and count<limit:
        _,g,u=heapq.heappop(heap)
        if g!=cost.get(u):continue
        count+=1
        if u==b:
            cells=[u]
            while u in parent:u=parent[u];cells.append(u)
            return [grid.world(*p) for p in reversed(cells)]
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            v=u[0]+dx,u[1]+dy
            if not grid.inside(v) or blocked[v[1],v[0]]:continue
            if dx and dy and (blocked[u[1],v[0]] or blocked[v[1],u[0]]):continue
            ng=g+math.hypot(dx,dy)
            if ng<cost.get(v,math.inf):
                cost[v]=ng;parent[v]=u
                heapq.heappush(heap,(ng+math.hypot(v[0]-b[0],v[1]-b[1]),ng,v))
    return []


def path_command(pose,path,speed=.07,lookahead=.07):
    if not path:return 0.,0.
    nearest=min(range(len(path)),key=lambda i:math.hypot(path[i][0]-pose[0],path[i][1]-pose[1]))
    target=path[-1]
    for p in path[nearest:]:
        if math.hypot(p[0]-pose[0],p[1]-pose[1])>=lookahead:target=p;break
    angle=wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2])
    w=max(-.45,min(.45,1.7*angle))
    v=min(speed,math.hypot(path[-1][0]-pose[0],path[-1][1]-pose[1])*.6) if abs(angle)<.35 else 0.
    return v/(1+2*abs(angle)),w


def swept_clear(pose,v,w,grid,radius,local_points,horizon=.7):
    """Check footprint against the grid and CURRENT raw scan, including rotations."""
    blocked=grid.blocked(radius) if grid else None
    x=y=yaw=0.
    for _ in range(15):
        if any(math.hypot(px-x,py-y)<=radius for px,py in local_points):return False
        if grid:
            c,s=math.cos(pose[2]),math.sin(pose[2]);p=grid.cell(pose[0]+x*c-y*s,pose[1]+x*s+y*c)
            if not grid.inside(p) or blocked[p[1],p[0]]:return False
        dt=horizon/14;x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt;yaw+=w*dt
    return True


def validate_route(route,grid,radius=.12):
    if set(route)!={'entry','inside','exit'}:raise ValueError('Définir les trois repères : entrée, intérieur, sortie.')
    blocked=grid.blocked(radius)
    for key,point in route.items():
        if not isinstance(point,(list,tuple)) or len(point)!=2 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in point):raise ValueError('Repère invalide : '+key)
        p=grid.cell(*point)
        if not grid.inside(p) or blocked[p[1],p[0]]:raise ValueError('Repère occupé, inconnu ou trop proche d’une paroi : '+key)
    if min(math.dist(route[a],route[b]) for a,b in [('entry','inside'),('inside','exit'),('entry','exit')])<.25:raise ValueError('Espacer les repères d’au moins 25 cm.')
    for a,b in [('entry','inside'),('inside','exit')]:
        if not astar(grid,route[a],route[b],radius):raise ValueError('Aucun passage sûr entre '+a+' et '+b+'.')
    return {key:[float(v) for v in point] for key,point in route.items()}


def plan_goal(grid,start,goal,radius,points=(),allow_nearby=False):
    candidates=[goal]
    if allow_nearby:
        for distance in (.15,.25):
            candidates.extend((goal[0]+distance*math.cos(a),goal[1]+distance*math.sin(a)) for a in np.linspace(0,2*math.pi,8,endpoint=False))
    clearance=grid.blocked(radius+.045,points)
    for target in candidates:
        cell=grid.cell(*target)
        if not grid.inside(cell) or clearance[cell[1],cell[0]]:continue
        path=astar(grid,start,target,radius,points)
        if path:return path,target
    return [],goal


def bounded_grid(grid,zone):
    if not isinstance(zone,(list,tuple)) or len(zone)!=4 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in zone):raise ValueError('Délimiter la zone autorisée sur la carte.')
    x0,y0,x1,y1=zone
    if x1-x0<.5 or y1-y0<.4:raise ValueError('Zone autorisée trop petite.')
    yy,xx=np.indices(grid.data.shape);a=(xx+.5)*grid.res;b=(yy+.5)*grid.res;c,s=math.cos(grid.origin[2]),math.sin(grid.origin[2]);wx=grid.origin[0]+a*c-b*s;wy=grid.origin[1]+a*s+b*c
    data=grid.data.copy();data[(wx<x0)|(wx>x1)|(wy<y0)|(wy>y1)]=100
    return Grid(data,grid.res,grid.origin)
