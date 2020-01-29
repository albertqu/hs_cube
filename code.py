import numpy as np
import scipy
from scipy import interpolate

def parse_serial_input():
    """ Given serial input, return initialization variables:
    
    Returns:
        u: float
            displacement
        du: float
            velocity
        ddu: float
            acceleration
        dt: float
            analysis sample interval
        ug: np.ndarray(float)
            ground displacement across time (excitation displacement)
        dug: np.ndarray (float)
            ground movement velocity
        dtg: float
            interval of ground motion measurement
        m: float
            mass
        m_table: float
            table mass
        gamma, c: variables
        k, n: variables
    """
    u, du, ddu, ug, dug, m, m_table = None, None, None, None, None, None, None
    gamma, c = None, None
    k, n = None
    dt, dtg = None, None
    T = dtg * len(ug)
    # if sample rate mismatch, interpolate
    if dt != dtg:
        xg = np.linspace(dtg, T, len(ug))
        xa = np.linspace(dt, T, T//dt)
        ugp = interpolate.interp1d(xg, ug)(xa)
        dugp = interpolate.interp1d(xg, dug)(xa)
        ug, dug = ugp, dugp
    return u, du, ddu, dt, ug, dug, dtg, m , m_table, gamma, c, k, n

def apply_command(u, ug):
    # do something with u+ug
    """Interface with actuator using A/D output"""
    pass

def get_feedback(u=None, k = None, fake=False):
    # retrieve feedback from actuator, wait until there is feedback
    """interface with actuator using A/D input"""
    fi=None
    if fake:
        fi = u * 2 * k
    return fi

def main():
    u, du, ddu, dt, ug, dug, dtg, m, m_table, gamma, c, k, n = parse_serial_input()
    N = len(ug)
    i = 1
    u = u + dt * du + ddu * dt ** 2 / 2
    m_eff = m + dt * gamma * c
    while i <= N:
        # TODO: clock align this step
        apply_command(u, ug)
        fi = get_feedback()
        du_hat = du + dt * (1-gamma) * ddu
        p_eff = - m * dug[i-1] - k * u + n * fi - c * du_hat
        ddu  - p_eff / (m_eff + n * m_table)
        du = du_hat + dt * gamma * ddu
        u = u + dt * du + ddu * dt ** 2 / 2
    pass