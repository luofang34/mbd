//! Dense LU factorisation with partial pivoting for the small iteration
//! matrices of implicit steps.

use alloc::vec::Vec;

/// Row-major `n × n` LU factors with their row permutation.
#[derive(Debug, Clone, PartialEq, Default)]
pub(crate) struct DenseLu {
    pub(crate) n: usize,
    pub(crate) a: Vec<f64>,
    pub(crate) perm: Vec<usize>,
    pub(crate) y: Vec<f64>,
}

impl DenseLu {
    /// Factors `I − shift · jac` for the row-major `n × n` matrix `jac`.
    /// Returns `false` when a pivot is zero or not finite.
    pub(crate) fn factor_shifted(&mut self, jac: &[f64], n: usize, shift: f64) -> bool {
        self.n = n;
        self.a.clear();
        self.a.extend(jac.iter().map(|v| -shift * v));
        self.a.resize(n * n, 0.0);
        for d in 0..n {
            if let Some(slot) = self.a.get_mut(d * n + d) {
                *slot += 1.0;
            }
        }
        self.perm.clear();
        self.perm.extend(0..n);
        self.y.resize(n, 0.0);
        (0..n).all(|k| self.eliminate(k))
    }

    /// Pivots column `k` and eliminates it from the rows below.
    fn eliminate(&mut self, k: usize) -> bool {
        let n = self.n;
        let column = self.a.iter().skip(k).step_by(n.max(1)).skip(k);
        let mut best = (k, 0.0_f64);
        for (row, value) in (k..n).zip(column) {
            if value.abs() > best.1 {
                best = (row, value.abs());
            }
        }
        if !(best.1 > 0.0 && best.1.is_finite()) {
            return false;
        }
        if best.0 != k {
            for j in 0..n {
                self.a.swap(k * n + j, best.0 * n + j);
            }
            self.perm.swap(k, best.0);
        }
        let Some((top, below)) = self.a.split_at_mut_checked((k + 1) * n) else {
            return false;
        };
        let Some(pivot_row) = top.get(k * n..) else {
            return false;
        };
        let Some(&pivot) = pivot_row.get(k) else {
            return false;
        };
        for row in below.chunks_exact_mut(n) {
            let Some(lead) = row.get_mut(k) else {
                continue;
            };
            let factor = *lead / pivot;
            *lead = factor;
            if factor != 0.0 {
                for (r, p) in row.iter_mut().zip(pivot_row).skip(k + 1) {
                    *r -= factor * p;
                }
            }
        }
        true
    }

    /// Solves the factored system in place: `b ← (I − shift · jac)⁻¹ b`.
    pub(crate) fn solve(&mut self, b: &mut [f64]) {
        let n = self.n;
        for (y, &p) in self.y.iter_mut().zip(&self.perm) {
            *y = b.get(p).copied().unwrap_or(0.0);
        }
        for i in 0..n {
            let Some((done, rest)) = self.y.split_at_mut_checked(i) else {
                return;
            };
            let row = self.a.get(i * n..i * n + i).unwrap_or(&[]);
            let sum: f64 = row.iter().zip(done.iter()).map(|(l, y)| l * y).sum();
            if let Some(y) = rest.first_mut() {
                *y -= sum;
            }
        }
        for i in (0..n).rev() {
            let Some((head, tail)) = self.y.split_at_mut_checked(i + 1) else {
                return;
            };
            let row = self.a.get(i * n..(i + 1) * n).unwrap_or(&[]);
            let upper = row.iter().skip(i + 1);
            let sum: f64 = upper.zip(tail.iter()).map(|(u, y)| u * y).sum();
            let diag = row.get(i).copied().unwrap_or(1.0);
            if let Some(y) = head.last_mut() {
                *y = (*y - sum) / diag;
            }
        }
        for (out, y) in b.iter_mut().zip(&self.y) {
            *out = *y;
        }
    }
}
