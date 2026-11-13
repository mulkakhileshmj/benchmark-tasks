"""Local checks only. Passing this script does not qualify a robot demo."""
import ast
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import types
import unittest
import zipfile

ROOT=Path(__file__).resolve().parent


def main():
    syntax=[]
    for p in ROOT.glob('*.py'):
        ast.parse(p.read_text(),filename=str(p));syntax.append(p.name)
    suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'))
    result=unittest.TextTestRunner(verbosity=1).run(suite)
    if not result.wasSuccessful():raise SystemExit(1)
    @dataclass
    class Waypoint:
        duration: float
        pos: tuple
        finger: float
        label: str
        yaw: float=0.
    modules={k:sys.modules.get(k) for k in ('yam_arena','yam_arena.constants','yam_arena.demo')}
    try:
        pkg=types.ModuleType('yam_arena');pkg.__path__=[]
        constants=types.ModuleType('yam_arena.constants')
        constants.FINGER_OPEN=-.04695;constants.FINGER_CLOSED=0.;constants.TABLE_HEIGHT=.7475
        demo=types.ModuleType('yam_arena.demo');demo.Waypoint=Waypoint
        sys.modules.update({'yam_arena':pkg,'yam_arena.constants':constants,'yam_arena.demo':demo})
        import demo_plan
        durations={arm:sum(w.duration for w in path) for arm,path in demo_plan.make_demo_plan().items()}
    finally:
        for name,old in modules.items():
            if old is None:sys.modules.pop(name,None)
            else:sys.modules[name]=old
    import geometry as g
    assert max(durations.values())<=120, durations
    assert abs(g.MUG_INNER_R-.0365)<1e-10 and g.MUG_ROD_COUNT==96
    assert max(s[1][1] for s in g.HANDLE_BOXES)<=.010
    assert g.MACHINE_MASS_KG==3 and g.BREW_STEPS==500
    report=dict(local_controls_passed=result.testsRun,syntax_files=syntax,
                candidate_schedule_s=durations,tests_are_synthetic=True,native_demo_score=None,
                native_mass_com_inertia_validated=False,native_contact_paths_validated=False,
                native_ik_and_grasps_validated=False,dream_steps=0,submission_ready=False)
    (ROOT/'local_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    dist=ROOT/'dist';dist.mkdir(exist_ok=True)
    with zipfile.ZipFile(dist/'coffee-shelf-physical-service-local.zip','w',compression=zipfile.ZIP_DEFLATED) as archive:
        for p in ROOT.rglob('*'):
            if not p.is_file() or any(x in p.relative_to(ROOT).parts for x in ('dist','__pycache__','runs','upstream_tests')):continue
            if p.name.startswith('upstream_'):continue
            archive.write(p,Path(ROOT.name)/p.relative_to(ROOT))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
