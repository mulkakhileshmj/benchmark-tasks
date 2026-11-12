import sys,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root))
from observed_rest import ObservedRest
v=np.array([[0.,0.,0.],[.02,0.,0.]])
q=np.array([[0.,0.,0.,1.]])
checks=[]
def check(n,x):
 assert x,n
 checks.append(n)
m=ObservedRest()
for i in range(201):result=m.update(v,q,.005,True)
check('native stationary second passes',result)
check('contact gate resets immediately',not m.update(v,q,.005,False) and m.elapsed==0.)
m=ObservedRest()
for i in range(1000):result=m.update(v+[.001*(i%2),0,0],q,.005,True)
check('sub20Hz translation oscillation rejected',not result and not m.settled)
m=ObservedRest()
for i in range(1000):
 theta=math.radians(2)*(i%2);qq=np.array([[0,0,math.sin(theta/2),math.cos(theta/2)]])
 result=m.update(v,qq,.005,True)
check('sub20Hz rotation oscillation rejected',not result and not m.settled)
m=ObservedRest()
for i in range(1000):result=m.update(v+[i*.00001,0,0],q,.005,True)
check('continuous drift cannot accumulate full second',not result)
m=ObservedRest()
for i in range(201):result=m.update(v+[.0001*(i%2),0,0],q,.005,True)
check('bounded harmless motion passes',result)
check('late excursion revokes settled state',not m.update(v+[.001,0,0],q,.005,True))
for i in range(200):result=m.update(v+[.001,0,0],q,.005,True)
check('new complete stable second required',result)
try:ObservedRest().update(v,q,.05,True)
except ValueError:checks.append('20Hz alias sample rejected')
else:raise AssertionError('20Hz alias')
print(checks)
