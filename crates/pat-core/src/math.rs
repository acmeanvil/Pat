//! Minimal 3D math: vectors, rotation matrices and rigid transforms.
//!
//! Kept in-house (instead of pulling in a linear algebra crate) because the
//! kinematics only needs a handful of operations and we want the numerics to
//! be easy to read and audit.

use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Neg, Sub};

pub const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(from = "[f64; 3]", into = "[f64; 3]")]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<[f64; 3]> for Vec3 {
    fn from(a: [f64; 3]) -> Self {
        Vec3::new(a[0], a[1], a[2])
    }
}

impl From<Vec3> for [f64; 3] {
    fn from(v: Vec3) -> Self {
        [v.x, v.y, v.z]
    }
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    pub const X: Vec3 = Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    pub const Y: Vec3 = Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    pub const Z: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }

    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Unit vector in the same direction, or `None` for a (near) zero vector.
    pub fn normalized(self) -> Option<Vec3> {
        let n = self.norm();
        (n > EPS).then(|| self * (1.0 / n))
    }

    /// Angle between two vectors in radians (robust near 0 and pi).
    pub fn angle_to(self, o: Vec3) -> f64 {
        self.cross(o).norm().atan2(self.dot(o))
    }

    pub fn distance(self, o: Vec3) -> f64 {
        (self - o).norm()
    }

    pub fn lerp(self, o: Vec3, t: f64) -> Vec3 {
        self + (o - self) * t
    }

    pub fn get(self, i: usize) -> f64 {
        match i {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

/// Spherical linear interpolation between two unit vectors.
pub fn slerp(a: Vec3, b: Vec3, t: f64) -> Vec3 {
    let theta = a.angle_to(b);
    if theta < 1e-12 {
        return a.lerp(b, t).normalized().unwrap_or(a);
    }
    if (std::f64::consts::PI - theta).abs() < 1e-9 {
        // Antiparallel: the great circle is not unique; fall back to lerp.
        return a.lerp(b, t).normalized().unwrap_or(a);
    }
    let s = theta.sin();
    a * (((1.0 - t) * theta).sin() / s) + b * ((t * theta).sin() / s)
}

/// Row-major 3x3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3(pub [[f64; 3]; 3]);

impl Mat3 {
    pub const IDENTITY: Mat3 = Mat3([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    /// Rotation by `angle` radians about unit axis `k` (Rodrigues' formula).
    pub fn rotation(k: Vec3, angle: f64) -> Mat3 {
        let (s, c) = angle.sin_cos();
        let t = 1.0 - c;
        let Vec3 { x, y, z } = k;
        Mat3([
            [c + x * x * t, x * y * t - z * s, x * z * t + y * s],
            [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
            [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
        ])
    }

    /// Matrix whose columns are the given vectors.
    pub fn from_cols(a: Vec3, b: Vec3, c: Vec3) -> Mat3 {
        Mat3([[a.x, b.x, c.x], [a.y, b.y, c.y], [a.z, b.z, c.z]])
    }

    pub fn transpose(&self) -> Mat3 {
        let m = &self.0;
        Mat3([
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ])
    }

    pub fn mul_vec(&self, v: Vec3) -> Vec3 {
        let m = &self.0;
        Vec3::new(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
        )
    }

    pub fn mul_mat(&self, o: &Mat3) -> Mat3 {
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (0..3).map(|k| self.0[i][k] * o.0[k][j]).sum();
            }
        }
        Mat3(r)
    }

    pub fn determinant(&self) -> f64 {
        let m = &self.0;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Solve `self * x = b`, or `None` when the matrix is (near) singular.
    pub fn solve(&self, b: Vec3) -> Option<Vec3> {
        let det = self.determinant();
        if det.abs() < 1e-12 {
            return None;
        }
        let m = &self.0;
        let col = |j: usize| Vec3::new(m[0][j], m[1][j], m[2][j]);
        let (c0, c1, c2) = (col(0), col(1), col(2));
        Some(Vec3::new(
            Mat3::from_cols(b, c1, c2).determinant() / det,
            Mat3::from_cols(c0, b, c2).determinant() / det,
            Mat3::from_cols(c0, c1, b).determinant() / det,
        ))
    }
}

/// Rigid transform: `p -> r * p + t`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Iso {
    pub r: Mat3,
    pub t: Vec3,
}

impl Iso {
    pub const IDENTITY: Iso = Iso {
        r: Mat3::IDENTITY,
        t: Vec3::ZERO,
    };

    pub fn translation(t: Vec3) -> Iso {
        Iso {
            r: Mat3::IDENTITY,
            t,
        }
    }

    /// Rotation by `angle` radians about the line through `point` along unit `axis`.
    pub fn rotation_about(axis: Vec3, point: Vec3, angle: f64) -> Iso {
        let r = Mat3::rotation(axis, angle);
        Iso {
            r,
            t: point - r.mul_vec(point),
        }
    }

    pub fn apply_point(&self, p: Vec3) -> Vec3 {
        self.r.mul_vec(p) + self.t
    }

    pub fn apply_vec(&self, v: Vec3) -> Vec3 {
        self.r.mul_vec(v)
    }

    /// `self ∘ o` (apply `o` first).
    pub fn then(&self, o: &Iso) -> Iso {
        Iso {
            r: self.r.mul_mat(&o.r),
            t: self.r.mul_vec(o.t) + self.t,
        }
    }

    pub fn inverse(&self) -> Iso {
        let rt = self.r.transpose();
        Iso {
            r: rt,
            t: -rt.mul_vec(self.t),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3) -> bool {
        a.distance(b) < 1e-9
    }

    #[test]
    fn rotation_right_hand_rule() {
        let r = Mat3::rotation(Vec3::Z, std::f64::consts::FRAC_PI_2);
        assert!(close(r.mul_vec(Vec3::X), Vec3::Y));
    }

    #[test]
    fn iso_inverse_roundtrip() {
        let a = Iso::rotation_about(Vec3::X, Vec3::new(1.0, 2.0, 3.0), 0.7);
        let p = Vec3::new(4.0, -5.0, 6.0);
        assert!(close(a.inverse().apply_point(a.apply_point(p)), p));
    }

    #[test]
    fn solve_3x3() {
        let m = Mat3([[2.0, 1.0, 0.0], [0.0, 3.0, 1.0], [1.0, 0.0, 4.0]]);
        let x = Vec3::new(1.0, -2.0, 0.5);
        let b = m.mul_vec(x);
        assert!(close(m.solve(b).unwrap(), x));
    }

    #[test]
    fn slerp_midpoint() {
        let m = slerp(Vec3::X, Vec3::Y, 0.5);
        let h = std::f64::consts::FRAC_1_SQRT_2;
        assert!(close(m, Vec3::new(h, h, 0.0)));
    }
}
