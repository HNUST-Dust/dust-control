"""Independent static force/moment checks for the empirical trim candidate."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools'))
import derive_lqr14 as model
import lqr14_pitch_trim as trim

class TrimModel(unittest.TestCase):
    def test_static_balance_and_anchor(self):
        samples=model.legacy.read_leg_samples(model.ROOT/'tools/data/leg_mass_properties0820.csv')
        params=model.read_parameters(model.ROOT/'tools/data/lqr14_trim7_candidate.json')
        robot=model.legacy.RobotParameters()
        offset=trim.forward_offset(samples, params)
        for length in np.linspace(samples[0]['leg_length_m'], samples[-1]['leg_length_m'], 101):
            sample,_=model.sample_at(samples,float(length))
            x,u,residual=trim.operating_point(robot,sample,offset)
            moment=(robot.body_mass_kg*length+2*robot.leg_mass_each_kg*sample['leg_com_from_wheel_m'])*np.sin(x[0])+robot.body_mass_kg*offset
            self.assertAlmostEqual(moment,0.,places=12)
            self.assertAlmostEqual(u[1],robot.body_mass_kg*robot.gravity_m_s2*offset)
            self.assertLess(np.linalg.norm(residual),1e-8)
        anchor=params['pitch_trim']['reference_length_m']
        sample,_=model.sample_at(samples,anchor)
        x,_,_=trim.operating_point(robot,sample,offset)
        self.assertAlmostEqual(np.rad2deg(x[0]),7.,places=10)

    def test_zero_trim_preserves_historical_dynamics(self):
        samples=model.legacy.read_leg_samples(model.ROOT/'tools/data/leg_mass_properties0820.csv')
        robot=model.legacy.RobotParameters()
        x=np.array([.02,.1,.2,.05,-.01,.03]);u=np.array([.1,-.2])
        for sample in samples:
            np.testing.assert_array_equal(trim.derivative(robot,sample,x,u,0.),model.legacy.planar_state_derivative(robot,sample,x,u))

if __name__=='__main__':unittest.main()
