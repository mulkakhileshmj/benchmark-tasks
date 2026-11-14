"""Export complete native 720p camera streams, including lossless ZIP frame stores."""
from pathlib import Path
import argparse,subprocess,json,hashlib
import numpy as np
p=argparse.ArgumentParser()
p.add_argument('run',type=Path);p.add_argument('output',type=Path);p.add_argument('--role',choices=['demo','dream'],required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
trials=[f for f in a.run.glob('adhoc*.json') if '.live.' not in f.name]
if len(trials)!=1:raise ValueError('Expected one completed trial')
trial=json.loads(trials[0].read_text());steps=trial['stats']['total_steps'];score=trial['results']['metrics']['task_score']
if trial['status']!='success' or trial.get('error'):raise ValueError('Incomplete or errored trial')
if a.role=='demo' and score!=1.:raise ValueError('Scripted demo must score 1.0')
if a.role=='dream' and steps!=2400:raise ValueError('Dream must run 2400 steps')
report={'role':a.role,'score':score,'steps':steps,'videos':{}}
for camera in ('top_cam','left_cam','right_cam'):
 frames=sorted((f for f in a.run.glob('frames/**/*') if f.is_file() and camera+'_' in f.name and f.suffix in ('.npy','.npz')),key=lambda f:int(f.stem.rsplit('_',1)[1]))
 if len(frames)!=steps+1:raise ValueError((camera,len(frames),steps+1))
 out=a.output/(camera+'.mp4')
 proc=subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','20','-i','-','-an','-c:v','libx264','-threads','2','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE)
 try:
  for f in frames:
   v=np.load(f)
   if isinstance(v,np.lib.npyio.NpzFile):rgb=v['frame'];v.close()
   else:rgb=v
   if rgb.shape!=(720,1280,3) or rgb.dtype!=np.uint8:raise ValueError((f,rgb.shape,rgb.dtype))
   proc.stdin.write(rgb.tobytes())
 finally:proc.stdin.close()
 if proc.wait():raise RuntimeError('Encoding failed')
 probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-select_streams','v:0','-show_entries','stream=codec_name,width,height,pix_fmt,avg_frame_rate,nb_read_frames','-of','json',str(out)]))['streams'][0]
 if (probe['codec_name'],probe['width'],probe['height'],probe['pix_fmt'],probe['avg_frame_rate'],int(probe['nb_read_frames']))!=('h264',1280,720,'yuv420p','20/1',steps+1):raise ValueError(probe)
 if out.stat().st_size>50_000_000:raise ValueError('Upload video exceeds 50 MB')
 report['videos'][camera]={'bytes':out.stat().st_size,'sha256':hashlib.sha256(out.read_bytes()).hexdigest(),**probe}
subprocess.run(['ffmpeg','-v','error','-y','-i',str(a.output/'top_cam.mp4'),'-frames:v','1',str(a.output/'setup.png')],check=True)
(a.output/'trial.json').write_text(trials[0].read_text())
(a.output/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
