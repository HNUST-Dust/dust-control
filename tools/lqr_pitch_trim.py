"""Empirical sagittal trim model, separate from the recovered historical model.

A user-selected lean at one leg length fits an EFFECTIVE horizontal COM offset.
This is a declared calibration assumption, not a CAD measurement or diagnosis.
"""
import numpy as np

import lqr_robot_dynamics as dynamics


def forward_offset(samples, parameters):
    trim = parameters.get('pitch_trim')
    if trim is None:
        return 0.0
    if trim.get('method') != 'effective_com_from_empirical_lean':
        raise ValueError('unsupported pitch trim calibration')
    theta = np.deg2rad(float(trim['reference_theta_deg']))
    length = float(trim['reference_length_m'])
    lengths = np.array([sample['leg_length_m'] for sample in samples])
    if not np.isfinite([theta, length]).all() or abs(theta) > np.deg2rad(10) or not lengths[0] <= length <= lengths[-1]:
        raise ValueError('trim must be within CAD range and +/-10 degrees')
    if not isinstance(trim.get('source'), str) or not trim['source'].strip():
        raise ValueError('empirical trim source is required')
    sample = dynamics.interpolate_sample(samples, lengths, length)
    robot = dynamics.RobotParameters()
    return float(-(length + 2*robot.leg_mass_each_kg*sample['leg_com_from_wheel_m']/robot.body_mass_kg)*np.sin(theta))


def derivative(robot, sample, state, control, com_x):
    if com_x == 0.0:
        return dynamics.planar_state_derivative(robot, sample, state, control)
    mw, iw = 2*robot.wheel_mass_each_kg, 2*robot.wheel_inertia_each_kg_m2
    mp, ip = 2*robot.leg_mass_each_kg, 2*sample['single_leg_inertia_kg_m2']
    mb, ib, d, g = robot.body_mass_kg, robot.body_pitch_inertia_kg_m2, robot.body_com_offset_m, robot.gravity_m_s2
    radius, length, c = robot.wheel_radius_m, sample['leg_length_m'], sample['leg_com_from_wheel_m']
    theta, theta_rate, _, speed, phi, phi_rate = state
    tw, tp = control
    st, ct, sp, cp = np.sin(theta), np.cos(theta), np.sin(phi), np.cos(phi)
    # COM relative to hip: x=a*cos(phi)-d*sin(phi), z=-a*sin(phi)-d*cos(phi).
    dx = -com_x*sp-d*cp
    dz = -com_x*cp+d*sp
    ddx = -com_x*cp+d*sp
    ddz = com_x*sp+d*cp
    mass, rhs = np.zeros((7, 7)), np.zeros(7)
    mass[0,0], mass[0,3], rhs[0] = iw/radius+mw*radius, radius, tw
    mass[1,0], mass[1,1], mass[1,3], mass[1,5] = -mp, -mp*c*ct, 1., -1.
    rhs[1] = -mp*c*theta_rate**2*st
    mass[2,1], mass[2,4], mass[2,6] = mp*c*st, 1., -1.
    rhs[2] = mp*g-mp*c*theta_rate**2*ct
    mass[3,1], mass[3,3], mass[3,4] = ip, c*ct, -c*st
    mass[3,5], mass[3,6], rhs[3] = (length-c)*ct, -(length-c)*st, -tw+tp
    mass[4,0], mass[4,1], mass[4,2], mass[4,5] = -mb, -mb*length*ct, -mb*dx, 1.
    rhs[4] = mb*(-length*theta_rate**2*st+ddx*phi_rate**2)
    mass[5,1], mass[5,2], mass[5,6] = mb*length*st, -mb*dz, 1.
    rhs[5] = mb*(g-length*theta_rate**2*ct+ddz*phi_rate**2)
    mass[6,2], mass[6,5], mass[6,6], rhs[6] = ib, dx, -dz, tp
    wheel_accel, theta_accel, phi_accel = np.linalg.solve(mass, rhs)[:3]
    body_accel = wheel_accel+length*theta_accel*ct-length*theta_rate**2*st
    return np.array([theta_rate, theta_accel, speed, body_accel, phi_rate, phi_accel])


def operating_point(robot, sample, com_x):
    moment_arm = robot.body_mass_kg*sample['leg_length_m'] + 2*robot.leg_mass_each_kg*sample['leg_com_from_wheel_m']
    sine = -robot.body_mass_kg*com_x/moment_arm
    if not np.isfinite(sine) or abs(sine) >= 1:
        raise ValueError('no upright static trim at this leg length')
    theta = float(np.arcsin(sine))
    x = np.array([theta, 0., 0., 0., 0., 0.])
    u = np.array([0., robot.body_mass_kg*robot.gravity_m_s2*com_x])
    residual = derivative(robot, sample, x, u, com_x)
    if np.linalg.norm(residual) > 1e-8:
        raise ValueError('nonzero trim dynamics residual')
    return x, u, residual


def linearize(robot, sample, state, control, com_x):
    step = 1e-6
    a = np.column_stack([(derivative(robot, sample, state+np.eye(6)[j]*step, control, com_x)-
                          derivative(robot, sample, state-np.eye(6)[j]*step, control, com_x))/(2*step) for j in range(6)])
    b = np.column_stack([(derivative(robot, sample, state, control+np.eye(2)[j]*step, com_x)-
                          derivative(robot, sample, state, control-np.eye(2)[j]*step, com_x))/(2*step) for j in range(2)])
    return a, b


def full_matrices(robot, sample, com_x):
    state, control, residual = operating_point(robot, sample, com_x)
    a6, b6 = linearize(robot, sample, state, control, com_x)
    ay, by = dynamics.yaw_matrices(robot, sample)
    a, b = np.zeros((10,10)), np.zeros((10,4))
    a[:6,:6], a[6:,6:], b[:6,:2], b[6:,2:] = a6, ay, b6, by
    transform, inputs = dynamics.coordinate_transforms()
    inverse = np.linalg.inv(transform)
    return inverse@a@transform, inverse@b@inputs, state, control, residual
