"""Source-identity and native-evidence gates. Never inherits upstream scores."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from fractions import Fraction

ROOT=Path(__file__).resolve().parent


def sources():
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(ROOT.iterdir()) if p.suffix in {'.py','.sh','.toml'}}


def qualify(log,model=False):
    log=log.resolve();trial=json.loads(log.read_text())
    if log.parent.parent.name!=('dream' if model else 'demo'):raise ValueError('Wrong native run directory')
    if json.loads((log.parent/'sources.json').read_text())!=sources():raise ValueError('Demo source differs from current source')
    if not model and (trial['status']!='success' or trial.get('error')):raise ValueError('Demo errored')
    if model and trial['status'] not in ('success','error'):raise ValueError('Model run has no final result')
    result=trial['results'];spec=trial['eval'];steps=trial['stats']['total_steps']
    score=result['metrics']['task_score']
    if result['total_trials']!=1 or not isinstance(score,(int,float)) or not 0<=score<=1:raise ValueError('Require one measured trial score')
    if not model and result['errored_trials']:raise ValueError('Scripted demo must be clean')
    if not model and score!=1.:raise ValueError('Require a full-score demo')
    expected=spec['policy'] in ('dream','dream_current') if model else spec['policy']=='coffee_servo'
    if spec['max_steps']!=2400 or spec['embodiment_info']['control_hz']!=20 or not expected:raise ValueError('Wrong run protocol')
    rows=[json.loads(line) for line in (log.parent/'trace.jsonl').read_text().splitlines()]
    if not(0<steps<=2400 and len(rows)==steps):raise ValueError('Incomplete native trace')
    for i,row in enumerate(rows):
        if row.get('diagnostic_filled_fixture') or row.get('diagnostic_camera_off'):raise ValueError('Diagnostic fixture or camera-off run cannot qualify')
        shapes=row.get('native_camera_shapes',{})
        if set(shapes)!= {'top_cam','left_cam','right_cam'} or any(shape[-3:-1]!=[720,1280] for shape in shapes.values()):raise ValueError('Require native 1280x720 camera sensors')
        if 'native_force_readback' not in row:raise ValueError('Missing native GPU contact-force evidence')
        allowed={'cracked_cup_used','hand_in_hot_stream','hot_wall_contact','cup_off_worktop','machine_mount_moved'} if model else set()
        faults=set(row.get('faults',[]))
        if abs(row['t']-(i+1)*.05)>1e-4 or faults-allowed or 'error' in row:raise ValueError('Invalid native trace')
        if faults and row['score']!=0.:raise ValueError('Safety fault must latch zero')
        if abs(row['emitted_ml']-row['spilled_ml']-sum(row['retained_ml'].values()))>1e-6:raise ValueError('Liquid not conserved')
    if not any(r['native_force_readback']['cm_mug_cracked']['saucer_normal_n']>.1 for r in rows[:20]):raise ValueError('Native saucer-contact positive control failed')
    if rows[-1]['score']!=score or not rows[-1]['contacts_available'] or not rows[-1]['mass_readback_available']:raise ValueError('Missing native capabilities or score mismatch')
    return dict(score=score,steps=steps,control_hz=20,source_verified=True,run_status=trial['status'],execution_error=trial.get('error'))


def export(log):
    log=log.resolve();meta=qualify(log,model=log.parent.parent.name=='dream')
    out=log.parent/'evidence';out.mkdir(exist_ok=True)
    rendered=out/'native_render'
    subprocess.run(['inspect-robots','video',str(log),'--out',str(rendered)],check=True)
    hashes={}
    for camera in ('top_cam','left_cam','right_cam'):
        videos=list(rendered.rglob('*'+camera+'.mp4'))
        if len(videos)!=1:raise ValueError('Missing camera stream: '+camera)
        target=out/(camera+'.mp4')
        subprocess.run(['ffmpeg','-v','error','-y','-i',str(videos[0]),'-c:v','libx264','-crf','22',
                        '-pix_fmt','yuv420p','-movflags','+faststart',str(target)],check=True)
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
            '-show_entries','stream=codec_name,width,height,r_frame_rate,nb_frames','-of','json',str(target)]))['streams'][0]
        if (probe['width'],probe['height'],probe['codec_name'])!=(1280,720,'h264') or Fraction(probe['r_frame_rate'])!=20 or int(probe['nb_frames'])!=meta['steps']+1 or target.stat().st_size>50_000_000:
            raise ValueError('Camera recording does not match HD evidence protocol')
        hashes[target.name]=hashlib.sha256(target.read_bytes()).hexdigest()
    subprocess.run(['ffmpeg','-v','error','-y','-i',str(out/'top_cam.mp4'),'-frames:v','1',str(out/'setup.png')],check=True)
    (out/'manifest.json').write_text(json.dumps(dict(**meta,sha256=hashes),indent=2)+'\n')
    print(out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=('snapshot','qualify','video'));p.add_argument('path',type=Path);a=p.parse_args()
    if a.mode=='snapshot':a.path.write_text(json.dumps(sources(),indent=2)+'\n')
    elif a.mode=='qualify':print(json.dumps(qualify(a.path),indent=2))
    else:
        export(a.path)
