use nalgebra::{vector, Const, OVector};
use ode_solvers::{Rk4, System};

#[derive(Debug, Copy, Clone)]
pub struct DoublePendulumState {
    // Top pendulum angle
    pub θ1: f64,
    // Lower pendulum angle
    pub θ2: f64,
    // Top pendulum angle change
    pub ω1: f64,
    // Lower pendulum angle change
    pub ω2: f64,
}

impl DoublePendulumState {
    pub fn new(θ1: f64, θ2: f64, ω1: f64, ω2: f64) -> Self {
        Self { θ1, θ2, ω1, ω2 }
    }

    fn as_mat(self) -> OVector<f64, Const<4>> {
        vector![self.θ1, self.θ2, self.ω1, self.ω2]
    }
}

pub struct DoublePendulumSystem {
    // Gravity
    pub g: f64,
    // Weight of inner pendulum
    pub m1: f64,
    /// Weight of outer pendulum
    pub m2: f64,
    /// Length of inner pendulum
    pub l1: f64,
    /// Length of outer pendulum
    pub l2: f64,
}

impl DoublePendulumSystem {
    pub fn step(&self, state: DoublePendulumState, delta: f64) -> DoublePendulumState {
        let mut solver = Rk4::new(self, 0., state.as_mat(), delta, delta);
        solver.integrate().unwrap();
        let out = solver.y_out();
        let out = &out[out.len() - 1];
        DoublePendulumState {
            θ1: out.x,
            θ2: out.y,
            ω1: out.z,
            ω2: out.w,
        }
    }
}

const G_EARTH: f64 = 9.80665;

impl Default for DoublePendulumSystem {
    fn default() -> Self {
        DoublePendulumSystem {
            g: G_EARTH,
            m1: 1.,
            m2: 1.,
            l1: 1.,
            l2: 1.,
        }
    }
}

impl<'a> System<f64, OVector<f64, Const<4>>> for &'a DoublePendulumSystem {
    fn system(&self, _t: f64, y: &OVector<f64, Const<4>>, dy: &mut OVector<f64, Const<4>>) {
        let (θ1, θ2, ω1, ω2) = (y.x, y.y, y.z, y.w);
        let (θ1, θ2, ω1, ω2) = deriv(θ1, θ2, ω1, ω2, self.g, self.m1, self.m2, self.l1, self.l2);
        dy.x = θ1;
        dy.y = θ2;
        dy.z = ω1;
        dy.w = ω2;
    }
}

/// Derivative for a pendulum system
///
/// Params:
///  - θ1: angle of top pendulum,
///  - θ2: angle of bottom pendulum,
///  - ω1: velocity of angle of top pendulum,
///  - ω2: velocity of angle of bottom pendulum,
///  - g: strength of gravity,
///  - m1: mass of top pendulum,
///  - m2: mass of bottom pendulum,
///  - l1: length of top pendulum,
///  - l2: length of bottom pendulum,
///
/// output: θ'1, θ'2, ω'1, ω'2
fn deriv(
    θ1: f64,
    θ2: f64,
    ω1: f64,
    ω2: f64,
    g: f64,
    m1: f64,
    m2: f64,
    l1: f64,
    l2: f64,
) -> (f64, f64, f64, f64) {
    let dc = (θ1 - θ2).cos();
    let ds = (θ1 - θ2).sin();
    let tdc = (2. * (θ1 - θ2)).cos();
    let num = -g * (2. * m1 + m2) * θ1.sin()
        - m2 * g * (θ1 - 2. * θ2).sin()
        - 2. * ds * m2 * (ω2 * ω2 * l2 + ω1 * ω1 * l1 * dc);
    let denom = l1 * (2. * m1 + m2 - m2 * tdc);
    let ωp1 = num / denom;
    let num =
        2. * ds * (ω1 * ω1 * l1 * (m1 + m2) + g * (m1 + m2) * θ1.cos() + ω2 * ω2 * l2 * m2 * dc);
    let denom = l2 * (2. * m1 + m2 - m2 * tdc);
    let ωp2 = num / denom;
    (ω1, ω2, ωp1, ωp2)
}
