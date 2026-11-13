"""Synthetic protocol fixtures; these never become submitted robot evidence."""
import json
from pathlib import Path
import tempfile
import unittest
from evidence import sources,qualify


class EvidenceControls(unittest.TestCase):
    def fixture(self,root,fault):
        folder=root/'dream'/'synthetic';folder.mkdir(parents=True)
        (folder/'sources.json').write_text(json.dumps(sources()))
        trial={'status':'success','error':None,
               'results':{'errored_trials':0,'total_trials':1,'metrics':{'task_score':0.}},
               'stats':{'total_steps':2400},
               'eval':{'policy':'dream','max_steps':2400,'embodiment_info':{'control_hz':20}}}
        log=folder/'trial.json';log.write_text(json.dumps(trial))
        rows=[dict(t=(i+1)*.05,score=0.,faults=[fault],emitted_ml=0.,spilled_ml=0.,
                   retained_ml={'a':0.},contacts_available=True,mass_readback_available=True,
                   native_camera_shapes={n:[1,720,1280,3] for n in ('top_cam','left_cam','right_cam')},
                   native_force_readback={'cm_mug_cracked':{'saucer_normal_n':3.4}})
              for i in range(2400)]
        (folder/'trace.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
        return log

    def test_real_task_failure_is_valid_model_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            log=self.fixture(Path(folder),'hot_wall_contact')
            self.assertEqual(qualify(log,model=True)['score'],0.)

    def test_infrastructure_failure_is_not_valid_model_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            log=self.fixture(Path(folder),'invalid_telemetry')
            with self.assertRaises(ValueError):qualify(log,model=True)
