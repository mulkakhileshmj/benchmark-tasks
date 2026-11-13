"""Synthetic scorer controls, not robot or native-physics results."""
import math
import unittest
from dataclasses import replace
from engine import Config,Cup,Capsule,Sample,Engine,capacity_ml,stream_intersection,stream_hits_capsule

C=Config()
FAR=tuple(Capsule((.05,-.4+i*.01,1.1),(.05,-.4+i*.01,1.2),.01) for i in range(6))


def frame(t,position=None,**kw):
    cups={C.good:Cup(position or (.43,0.,.8035)),C.cracked:Cup((.55,-.25,.7575))}
    values=dict(t=t,cups=cups,shelf_out=0.,button_depth=0.,machine_position=(.43,0.,.7475),hands=FAR,
                contacts_available=True,mass_readback_available=True,fingers_open=True)
    values.update(kw)
    return Sample(**values)


def run_brew(engine):
    for i in range(500):engine.update(frame((i+1)*.05,button_depth=.007))


class Controls(unittest.TestCase):
    def test_idle_zero(self):
        e=Engine()
        for i in range(60):e.update(frame((i+1)*.05))
        self.assertEqual(e.value,0.)

    def test_direct_geometric_capture_without_shelf_history(self):
        e=Engine();run_brew(e)
        self.assertAlmostEqual(e.volume[C.good],120,6)
        self.assertEqual(e.value,.5)

    def test_volume_conserved(self):
        e=Engine();run_brew(e)
        self.assertAlmostEqual(e.emitted,sum(e.volume.values())+e.spilled,6)

    def test_no_cup_spills_all(self):
        e=Engine()
        for i in range(500):e.update(frame((i+1)*.05,position=(.2,-.2,.7575),button_depth=.007))
        self.assertAlmostEqual(e.spilled,120,6)
        self.assertEqual(e.value,0.)

    def test_above_nozzle_cannot_fill(self):
        self.assertIsNone(stream_intersection(Cup((.43,0.,1.2)),C))

    def test_edge_outside_opening_cannot_fill(self):
        self.assertIsNone(stream_intersection(Cup((.43+C.inner_radius,0.,.8035)),C))

    def test_nozzle_already_inside_open_cup_can_fill(self):
        e=Engine();e.update(frame(.05,position=(.43,0.,.90),button_depth=.007))
        self.assertAlmostEqual(e.volume[C.good],.24)

    def test_lower_cup_does_not_catch_through_upper_cup(self):
        e=Engine()
        cups={C.good:Cup((.43,0.,.8035)),C.cracked:Cup((.43,0.,.9035))}
        e.update(frame(.05,cups=cups,button_depth=.007))
        self.assertEqual(e.volume[C.good],0.)
        self.assertIn('cracked_cup_used',e.faults)

    def test_held_button_single_brew(self):
        e=Engine()
        for i in range(1000):e.update(frame((i+1)*.05,button_depth=.007))
        self.assertEqual(e.brews,1)

    def test_press_during_brew_consumed(self):
        e=Engine()
        for i in range(600):e.update(frame((i+1)*.05,button_depth=0. if i==100 else .007))
        self.assertEqual(e.brews,1)

    def test_deliberate_second_brew_blocks_full(self):
        e=Engine();run_brew(e)
        e.update(frame(25.05,button_depth=0.))
        e.update(frame(25.1,button_depth=.007))
        self.assertEqual(e.brews,2)

    def test_tipping_spills_instead_of_freezing_level(self):
        e=Engine();run_brew(e)
        s=frame(25.05)
        cups=dict(s.cups);cups[C.good]=Cup((.43,0.,.85),(1.,0.,0.,0.))
        e.update(replace(s,cups=cups))
        self.assertEqual(e.volume[C.good],0.)
        self.assertAlmostEqual(e.spilled,120,6)
        self.assertEqual(e.value,0.)

    def test_tilted_capacity_monotonic(self):
        values=[capacity_ml(Cup((0,0,0),(math.sin(a/2),0,0,math.cos(a/2))),C)
                for a in (0.,.2,.5,1.,math.pi/2)]
        self.assertEqual(values,sorted(values,reverse=True))

    def test_rapid_upright_acceleration_can_spill(self):
        e=Engine();run_brew(e);s=frame(25.05)
        cups=dict(s.cups);cups[C.good]=Cup((.43,0.,.8035),linear_velocity=(5.,0.,0.))
        e.update(replace(s,cups=cups))
        self.assertGreater(e.spilled,2.)
        self.assertLess(e.volume[C.good],120.)

    def test_nonfinite_latches_zero(self):
        e=Engine();e.update(frame(.05,shelf_out=float('nan')))
        e.update(frame(.1))
        self.assertEqual(e.value,0.);self.assertTrue(e.faults)

    def test_quaternion_invalid_latches_zero(self):
        e=Engine();s=frame(.05);cups=dict(s.cups);cups[C.good]=Cup((.43,0,.8),(0,0,0,0))
        e.update(replace(s,cups=cups));self.assertIn('invalid_telemetry',e.faults)

    def test_missing_cup_fails_closed(self):
        e=Engine();e.update(frame(.05,cups={C.good:Cup((.43,0,.8))}))
        self.assertIn('invalid_telemetry',e.faults)

    def test_malformed_position_fails_closed(self):
        e=Engine();s=frame(.05);cups=dict(s.cups);cups[C.good]=Cup((.43,0.))
        e.update(replace(s,cups=cups));self.assertIn('invalid_telemetry',e.faults)

    def test_nonfinite_mass_readback_fails_closed(self):
        e=Engine();e.update(frame(.05,mass_kg={C.good:float('nan')}))
        self.assertIn('invalid_telemetry',e.faults)

    def test_dropped_clock_sample_latches_zero(self):
        e=Engine();e.update(frame(.05));e.update(frame(.15))
        self.assertIn('invalid_telemetry',e.faults)

    def test_duplicate_clock_sample_latches_zero(self):
        e=Engine();e.update(frame(.05));e.update(frame(.05))
        self.assertIn('invalid_telemetry',e.faults)

    def test_bad_first_clock_fails_closed(self):
        e=Engine();e.update(frame(2.));self.assertTrue(e.faults)

    def test_stream_capsule_exact(self):
        self.assertTrue(stream_hits_capsule(Capsule((.43,0,.87),(.43,0,.9),.01),C.nozzle,.85))
        self.assertFalse(stream_hits_capsule(Capsule((.43,.1,.87),(.43,.1,.9),.01),C.nozzle,.85))

    def test_between_frame_stream_crossing_detected(self):
        e=Engine()
        a=Capsule((.35,0,.90),(.35,0,.91),.005)
        b=Capsule((.51,0,.90),(.51,0,.91),.005)
        e.update(frame(.05,hands=(a,)+FAR[1:]))
        e.update(frame(.1,button_depth=.007,hands=(b,)+FAR[1:]))
        self.assertIn('hand_in_hot_stream',e.faults)

    def test_native_hot_wall_contact_latches(self):
        e=Engine();run_brew(e)
        self.assertGreater(e.wall_temp[C.good],43.)
        e.update(frame(25.05,wall_contacts=frozenset({C.good})))
        self.assertIn('hot_wall_contact',e.faults)

    def final_samples(self,e,**changes):
        for i in range(45):
            masses={n:C.dry_mass+v/1000 for n,v in e.volume.items()}
            e.update(frame((501+i)*.05,position=C.saucer,saucer_contacts=frozenset({C.good}),
                           mass_kg=masses,mass_volume_ml=dict(e.volume),**changes))

    def test_synthetic_complete_positive(self):
        e=Engine();run_brew(e);self.final_samples(e)
        self.assertEqual(e.value,1.)

    def test_missing_native_contact_capability_blocks_full(self):
        e=Engine();run_brew(e);self.final_samples(e,contacts_available=False)
        self.assertEqual(e.value,.75)

    def test_missing_native_mass_capability_blocks_full(self):
        e=Engine();run_brew(e);self.final_samples(e,mass_readback_available=False)
        self.assertEqual(e.value,.75)

    def test_held_or_hovering_cup_cannot_full_score(self):
        e=Engine();run_brew(e)
        for i in range(45):e.update(frame((501+i)*.05,position=C.saucer))
        self.assertEqual(e.value,.5)

    def test_wrong_saucer_not_full(self):
        e=Engine();run_brew(e)
        for i in range(45):e.update(frame((501+i)*.05,position=(.39,-.19,C.saucer[2]),saucer_contacts=frozenset({C.good})))
        self.assertEqual(e.value,.5)

    def test_no_positive_with_missing_hand_geometry(self):
        e=Engine();e.update(frame(.05,hands=()))
        self.assertTrue(e.faults)


if __name__=='__main__':unittest.main()
