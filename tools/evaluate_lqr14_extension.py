#!/usr/bin/env python3
"""Offline-only extrapolation study; never updates runtime gains or configuration."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

import derive_lqr14 as model
import design_lqr14 as design


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-length-m', type=float, default=.34,
                        help='analysis endpoint, not a mechanical limit')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    source = model.ROOT / 'tools/data/leg_mass_properties0820.csv'
    samples = model.legacy.read_leg_samples(source)
    lengths, _ = model.validate_samples(samples)
    endpoint = args.max_length_m
    if not np.isfinite(endpoint) or not lengths[-1] < endpoint <= .36:
        parser.error('analysis endpoint must exceed CSV maximum and be <= 0.36 m')
    parameters = model.read_parameters(model.ROOT / 'tools/data/lqr14_development.json')
    parameters['calibration_status'] = 'OFFLINE ONLY: extrapolated geometry, not CAD validated'
    grid = np.unique(np.r_[np.arange(lengths[-1] + .005, endpoint, .005), endpoint])
    variants, diagnostics = {}, {}
    for count in (2, 3, 4):
        extended = [dict(sample) for sample in samples]
        fits = {key: np.polyfit(lengths[-count:], [s[key] for s in samples[-count:]], 1)
                for key in ('leg_com_from_wheel_m', 'single_leg_inertia_kg_m2')}
        # Anchor at the last CAD sample to keep parameter values continuous.
        for length in grid:
            extended.append({'leg_length_m': float(length), **{
                key: float(samples[-1][key] + fit[0] * (length - lengths[-1]))
                for key, fit in fits.items()}})
        nodes, _ = model.validate_samples(extended)
        results = [design.synthesize(model.derive_model(extended, float(x), parameters))
                   for x in nodes]
        gains = np.array([result['gain'] for result in results])
        worst = model.check_schedule(extended, parameters, nodes, gains, 401)
        variants[count] = (extended, nodes, gains)
        baseline = gains[len(samples)-1]
        diagnostics[str(count)] = {
            'fit_last_samples': count, 'anchored_at_last_CAD_sample': True,
            'extrapolated_samples': extended[len(samples):],
            'dense_schedule_max_spectral_radius': worst,
            'max_relative_dare_residual': max(r['relative_dare_residual'] for r in results),
            'endpoint_gain_frobenius_relative_change': float(
                np.linalg.norm(gains[-1]-baseline)/np.linalg.norm(baseline)),
            'max_gain_element_change_from_boundary': float(np.max(np.abs(gains[-1]-baseline)))}
    # Model uncertainty exercise: schedule from one fit applied to another fit.
    cross = []
    for controller, (_, nodes, gains) in variants.items():
        for plant, (plant_samples, _, _) in variants.items():
            worst = 0.
            for length in np.linspace(lengths[-1], endpoint, 101):
                data = design.synthesize(model.derive_model(plant_samples, float(length), parameters))
                gain = np.array([[np.interp(length, nodes, gains[:, i, j])
                                  for j in range(14)] for i in range(6)])
                radius = float(np.max(np.abs(np.linalg.eigvals(
                    np.array(data['discrete_a']) - np.array(data['discrete_b']) @ gain))))
                worst = max(worst, radius)
            cross.append({'controller_fit_samples': controller, 'plant_fit_samples': plant,
                          'max_spectral_radius': worst, 'stable': worst < 1.})
    report = {'offline_only': True, 'runtime_modified': False,
              'source_csv_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'CAD_max_length_m': float(lengths[-1]), 'analysis_max_length_m': endpoint,
              'variants': diagnostics, 'cross_fit_checks': cross,
              'limitations': ['340 mm is an analysis endpoint, not a verified mechanical limit',
                              'linear double-support model; no saturation, impact, slip or gas spring parameter uncertainty',
                              'cross-fit checks do not bound actual CAD or hardware error',
                              'COM derivative may jump at CAD/extrapolation boundary; dense checks include both sides',
                              'runtime gains, height validity gates and motor output remain unchanged']}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / 'evaluation.json'
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
