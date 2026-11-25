"""Hardware frame math: gravity-aligned Z-up world / IMU -> local FRD / body FRD."""
import math


def unit(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('Invalid quaternion')
    norm = math.hypot(*q)
    if not .5 <= norm <= 1.5:
        raise ValueError('Quaternion is not near unit length')
    result = tuple(v/norm for v in q)
    return tuple(-v for v in result) if result[0] < 0 else result


def conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def mul(a, b):
    w,x,y,z = a
    W,X,Y,Z = b
    return (w*W-x*X-y*Y-z*Z, w*X+x*W+y*Z-z*Y,
            w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W)


def rotate(q, v):
    return mul(mul(q, (0., *v)), conj(q))[1:]


def cross(a,b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def frd(v):
    return (v[0], -v[1], -v[2])


def body_odometry(position, q_wi, velocity_i, omega_i, q_bi, p_bi):
    values = (*position, *velocity_i, *omega_i, *p_bi)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite odometry/mount')
    q_wi, q_bi = unit(q_wi), unit(q_bi)
    q_wb = unit(mul(q_wi, conj(q_bi)))
    lever = rotate(q_wb, p_bi)
    p_wb = tuple(p-l for p,l in zip(position, lever))
    omega_b = rotate(q_bi, omega_i)
    lever_velocity = rotate(q_wb, cross(omega_b, p_bi))
    v_wb = tuple(v-l for v,l in zip(rotate(q_wi, velocity_i), lever_velocity))
    # Rx(pi) on world Z-up and body FLU: local FRD has arbitrary heading, NOT true North.
    flip = (0., 1., 0., 0.)
    return frd(p_wb), unit(mul(mul(flip, q_wb), conj(flip))), frd(v_wb), frd(omega_b)
