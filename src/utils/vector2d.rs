use std::iter::Sum;
use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector2D {
    pub x: f64,
    pub y: f64,
}

impl Vector2D {
    pub fn new(x: f64, y: f64) -> Self {
        Vector2D { x, y }
    }

    pub fn magnitude(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag == 0.0 {
            *self
        } else {
            Vector2D::new(self.x / mag, self.y / mag)
        }
    }

    pub fn dot(&self, other: &Vector2D) -> f64 {
        self.x * other.x + self.y * other.y
    }

    pub fn angle(&self) -> f64 {
        self.y.atan2(self.x)
    }
}

impl Sum for Vector2D {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Vector2D::new(0.0, 0.0), |a, b| a + b)
    }
}

impl Add for Vector2D {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Vector2D::new(self.x + other.x, self.y + other.y)
    }
}

impl Sub for Vector2D {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Vector2D::new(self.x - other.x, self.y - other.y)
    }
}

impl Mul<f64> for Vector2D {
    type Output = Self;

    fn mul(self, scalar: f64) -> Self {
        Vector2D::new(self.x * scalar, self.y * scalar)
    }
}

impl Mul<Vector2D> for f64 {
    type Output = Vector2D;

    fn mul(self, vector: Vector2D) -> Vector2D {
        Vector2D::new(self * vector.x, self * vector.y)
    }
}

impl Div<f64> for Vector2D {
    type Output = Self;

    fn div(self, scalar: f64) -> Self {
        Vector2D::new(self.x / scalar, self.y / scalar)
    }
}

impl Neg for Vector2D {
    type Output = Self;

    fn neg(self) -> Self {
        Vector2D::new(-self.x, -self.y)
    }
}
