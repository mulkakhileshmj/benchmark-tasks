"""Inspect Robots with lossless frame storage. Simulation and policy are unchanged."""
import io,zipfile
from pathlib import Path
import numpy as np
from inspect_robots.frames import FrameStore,FrameRef,_frame_filename
from importlib.metadata import entry_points
def put(self,trial_id,t,camera,image):
    path=self.root/_frame_filename(trial_id,camera,t)
    data=io.BytesIO();np.save(data,image)
    with zipfile.ZipFile(path,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as archive:
        archive.writestr('frame.npy',data.getvalue())
    with np.load(path) as checked:
        if not np.array_equal(image,checked['frame']):raise RuntimeError('Frame verification failed')
    self.count+=1
    return FrameRef(camera=camera,t=t,path=str(path))
def load(self):
    value=np.load(self.path)
    if isinstance(value,np.lib.npyio.NpzFile):
        result=value['frame'];value.close();return result
    return np.asarray(value,dtype=np.uint8)
FrameStore.put=put
FrameRef.load=load
next(iter(entry_points(group='console_scripts').select(name='inspect-robots'))).load()()
