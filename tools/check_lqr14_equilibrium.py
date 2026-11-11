#!/usr/bin/env python3
"""Nominal-model reference consistency check; NOT a physical robot equilibrium calibration."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import root

import derive_lqr14 as model
import design_lqr14 as design


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--length-m', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    samples = model.legacy.read_leg_samples(model.ROOT/'tools/data/leg_mass_properties0820.csv')
    parameters = model.read_parameters(model.ROOT/'tools/data/lqr14_development.json')
    lengths, _ = model.validate_samples(samples)
    sample, _ = model.sample_at(samples, args.length_m)
    gains = np.array([design.synthesize(model.derive_model(samples, float(length), parameters))['gain']
                      for length in lengths])
    # Same linear gain interpolation as the runtime schedule.
    gain = np.array([[np.interp(args.length_m, lengths, gains[:, i, j]) for j in range(14)] for i in range(6)])
    robot = model.legacy.RobotParameters()
    results = []
    for degrees in (0., 5., 7.):
        reference = np.deg2rad(degrees)
        def residual(state):
            position, theta, pitch = state
            error = np.zeros(14)
            error[0] = position  # Deliberately unclamped to find required equilibrium.
            error[4] = error[6] = theta-reference
            error[8] = pitch
            inputs = -gain@error
            derivative = model.legacy.planar_state_derivative(robot, sample,
                [theta, 0., position, 0., pitch, 0.], [inputs[0]+inputs[1], inputs[2]+inputs[3]])
            return derivative[[1, 3, 5]]
        solution = root(residual, [0., reference, 0.])
        results.append({'theta_reference_deg': degrees, 'converged': bool(solution.success),
                        'unclamped_equilibrium_position_error_m': float(solution.x[0]),
                        'theta_deg': float(np.rad2deg(solution.x[1])),
                        'pitch_deg': float(np.rad2deg(solution.x[2])),
                        'residual_norm': float(np.linalg.norm(solution.fun)),
                        'within_runtime_position_clip': bool(abs(solution.x[0]) <= .15)})
    report = {'nominal_model_only': True, 'length_m': args.length_m, 'results': results,
              'assumptions': ['symmetric constant-length flat double support',
                              'original sagittal nonlinear equations; ideal actuator torque',
                              'gain scheduled at supplied length; zero speed/pitch references',
                              'no calibrated horizontal COM offset, friction or gas spring error',
                              'position clip removed only for solving required nominal equilibrium'],
              'interpretation': 'Checks mathematical reference consistency only; not evidence that the physical 7deg calibration is wrong or that the limit cycle has one unique cause.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
