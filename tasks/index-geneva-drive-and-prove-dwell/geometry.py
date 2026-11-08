"""Convex contact pieces for a four slot Geneva drive. Dimensions are metres."""
import math
import numpy as np
D=.080;R=D/math.sqrt(2);LOCK=.043;CUTOUT=.0445;OUTER=.060;SLOT=.0055;THICK=.012
def prism(points,z=0.,height=THICK,color=(.9,.55,.05)):
 points=np.asarray(points,dtype=float)
 vertices=np.array([[x,y,zz] for zz in (z-height/2,z+height/2) for x,y in points])
 low=vertices.min(axis=0);high=vertices.max(axis=0);centre=(low+high)/2
 n=len(points);faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
 return dict(shape='mesh',pos=centre.tolist(),size=(high-low).tolist(),vertices=(vertices-centre).tolist(),faces=faces,color=color)
def rotate(points,angle):
 c=math.cos(angle);s=math.sin(angle)
 return [[c*x-s*y,s*x+c*y] for x,y in points]
def star_parts():
 parts=[];c=D/math.sqrt(2)
 def upper(x):
  limit=min(x,math.sqrt(max(0.,OUTER*OUTER-x*x)))
  if abs(x-c)<CUTOUT:limit=min(limit,c-math.sqrt(CUTOUT*CUTOUT-(x-c)**2))
  return max(SLOT,limit)
 edges=np.linspace(SLOT,math.sqrt(OUTER*OUTER-SLOT*SLOT),33)
 for a,b in zip(edges[:-1],edges[1:]):
  polygon=[[a,SLOT],[b,SLOT],[b,upper(b)],[a,upper(a)]]
  # Remove repeated points at the tips before convex extrusion.
  polygon=[point for i,point in enumerate(polygon) if not any(np.linalg.norm(np.array(point)-prior)<1e-9 for prior in polygon[:i])]
  if len(polygon)<3:continue
  for swap in (False,True):
   for sx in (-1,1):
    for sy in (-1,1):
     points=[[sx*(y if swap else x),sy*(x if swap else y)] for x,y in polygon]
     parts.append(prism(rotate(points,math.pi/4)))
 parts.append(dict(shape='cylinder',pos=(0,0,0),size=(.018,.018,THICK),radius=.009,height=THICK,axis='Z',color=(.9,.55,.05)))
 return tuple(parts)
def driver_parts(with_pin=True):
 parts=[]
 # The missing 120 degree sector opens towards the roller during indexing.
 angles=np.linspace(math.radians(150),math.radians(390),65)
 for a,b in zip(angles[:-1],angles[1:]):parts.append(prism([[0,0],[LOCK*math.cos(a),LOCK*math.sin(a)],[LOCK*math.cos(b),LOCK*math.sin(b)]],color=(.12,.35,.85)))
 parts.append(dict(pos=(0,R/2,.020),size=(.012,R,.012),color=(.12,.35,.85)))
 if with_pin:parts.append(dict(shape='cylinder',pos=(0,R,.032),size=(.008,.008,.076),radius=.004,height=.076,axis='Z',color=(.65,.65,.65)))
 return tuple(parts)
