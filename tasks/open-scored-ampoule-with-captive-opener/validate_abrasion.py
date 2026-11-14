"""Checks the contact-gated abrasion integrator; these are not native trials."""
import math,numpy as np
from scoring_collar import Collar,BINS
class Body:
 def __init__(self,xyz):self.xyz=xyz
 def get_transforms(self):return np.array([[*self.xyz,0,0,0,1.]])
class Contact:
 def __init__(self):self.angle=0.;self.touch=True;self.force=.1;self.height=.0198
 def get_contact_data(self,dt):
  return (np.array([self.force]),np.array([[.003*math.cos(self.angle),.003*math.sin(self.angle),self.height]]),
    np.zeros((1,3)),np.array([-.00001]),np.array([int(self.touch)]),np.array([0]))
class Attr:
 def __init__(self):self.value=.2
 def Set(self,value):self.value=value
 def Get(self):return self.value
class Joint:
 def __init__(self):self.attr=Attr()
 def GetBreakTorqueAttr(self):return self.attr
def monitor():
 c=Collar.__new__(Collar);c.body=Body([0,0,0]);c.files=[Body([0,0,.0198]) for _ in range(4)]
 c.views=[Contact() for _ in range(4)];c.progress=np.zeros(BINS);c.last=[None]*4
 c.scored=False;c.travel=0.;c.contacts=0;c.marked=set();c.witness={};c.joint=Joint()
 return c
def update(c,angle,centroid_jitter=0):
 for i,(body,v) in enumerate(zip(c.files,c.views)):
  theta=angle+i*math.pi/2;body.xyz=[.00345*math.cos(theta),.00345*math.sin(theta),.0198]
  v.angle=theta+centroid_jitter
 c.step(.004)
def sweep(c):
 for a in np.linspace(-.15,math.pi/2+.15,1000):update(c,a)
 for a in np.linspace(math.pi/2+.15,-.15,1000):update(c,a)
def check():
 c=monitor()
 for i in range(2000):update(c,0,.2*math.sin(i))
 assert c.travel==0 and not c.scored,'stationary file / changing contact centroid'
 for attr,value in [('touch',False),('force',0.),('height',.023)]:
  c=monitor()
  for v in c.views:setattr(v,attr,value)
  sweep(c);assert c.travel==0 and not c.scored,attr
 c=monitor()
 for a in np.linspace(0,.2,500):update(c,a)
 assert not c.scored and c.joint.attr.value==.2,'partial arc'
 c=monitor();sweep(c)
 assert c.scored and c.joint.attr.value==.020 and (c.progress>=.00025).sum()>=34,'full contact sweep'
 print('6 abrasion controls passed; native validation still required')
if __name__=='__main__':check()
