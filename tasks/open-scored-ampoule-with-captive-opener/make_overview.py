from pathlib import Path
import subprocess,json,hashlib,sys
root=Path(sys.argv[1])
out=root/'overview.mp4'
subprocess.run(['ffmpeg','-nostdin','-v','error','-y','-i',str(root/'top_cam.mp4'),'-i',str(root/'left_cam.mp4'),'-i',str(root/'right_cam.mp4'),'-filter_complex','[1:v]scale=384:216[left];[2:v]scale=384:216[right];[0:v][left]overlay=8:8[one];[one][right]overlay=W-w-8:8[out]','-map','[out]','-an','-c:v','libx264','-threads','2','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],check=True)
s=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-select_streams','v:0','-show_entries','stream=codec_name,width,height,pix_fmt,avg_frame_rate,nb_read_frames','-of','json',str(out)]))['streams'][0]
expected=json.loads((root/'verification.json').read_text())['steps']+1
assert (s['codec_name'],s['width'],s['height'],s['pix_fmt'],s['avg_frame_rate'],int(s['nb_read_frames']))==('h264',1280,720,'yuv420p','20/1',expected)
assert out.stat().st_size<50_000_000
(root/'overview-verification.json').write_text(json.dumps({'layout':'Unscaled native top camera with synchronized left/right wrist insets, each 384x216; no time cuts or synthesized frames','bytes':out.stat().st_size,'sha256':hashlib.sha256(out.read_bytes()).hexdigest(),**s},indent=2)+'\n')
