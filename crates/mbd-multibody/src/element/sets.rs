//! Element collections: a slice, an option, a box, a reference and tuples
//! are elements themselves, so a host composes its element set with plain
//! data structures and keeps typed access to every member. Blocks of
//! continuous state and of modes follow member order.

use alloc::boxed::Box;
use alloc::vec::Vec;

use super::{ElementContext, ForceElement};
use crate::MultibodyError;
use crate::model::{Loads, Resolution};

fn short(expected: usize, actual: usize) -> MultibodyError {
    MultibodyError::StateLengthMismatch { expected, actual }
}

/// Splits the first `n` values off the front of `cursor`.
pub(crate) fn take<'a>(cursor: &mut &'a [f64], n: usize) -> Result<&'a [f64], MultibodyError> {
    let (head, tail) = cursor
        .split_at_checked(n)
        .ok_or_else(|| short(n, cursor.len()))?;
    *cursor = tail;
    Ok(head)
}

/// Splits the first `n` values off the front of a mutable `cursor`.
pub(crate) fn take_mut<'a>(
    cursor: &mut &'a mut [f64],
    n: usize,
) -> Result<&'a mut [f64], MultibodyError> {
    let whole = core::mem::take(cursor);
    let len = whole.len();
    let (head, tail) = whole.split_at_mut_checked(n).ok_or_else(|| short(n, len))?;
    *cursor = tail;
    Ok(head)
}

/// Splits the first `n` values off a mutable `cursor`, leaving shorter
/// buffers untouched: sizing callbacks cannot fail.
fn take_lenient<'a>(cursor: &mut &'a mut [f64], n: usize) -> &'a mut [f64] {
    take_mut(cursor, n).unwrap_or(&mut [])
}

impl<I: ?Sized, T: ForceElement<I>> ForceElement<I> for [T] {
    fn state_len(&self) -> usize {
        self.iter().map(T::state_len).sum()
    }

    fn mode_len(&self) -> usize {
        self.iter().map(T::mode_len).sum()
    }

    fn has_transitions(&self) -> bool {
        self.iter().any(T::has_transitions)
    }

    fn initial_state(&self, mut state: &mut [f64], mut modes: &mut [f64]) {
        for e in self {
            let (s, m) = (
                take_lenient(&mut state, e.state_len()),
                take_lenient(&mut modes, e.mode_len()),
            );
            e.initial_state(s, m);
        }
    }

    fn state_scale(&self, mut scale: &mut [f64]) {
        for e in self {
            e.state_scale(take_lenient(&mut scale, e.state_len()));
        }
    }

    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        mut state: &[f64],
        mut modes: &[f64],
        loads: &mut Loads<'_>,
        mut state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        for e in self {
            let n = e.state_len();
            let (s, m) = (take(&mut state, n)?, take(&mut modes, e.mode_len())?);
            e.evaluate(ctx, s, m, loads, take_mut(&mut state_dot, n)?)?;
        }
        Ok(())
    }

    fn has_resolution(&self) -> bool {
        self.iter().any(T::has_resolution)
    }

    fn resolve(
        &self,
        ctx: &ElementContext<'_, I>,
        mut state: &[f64],
        mut modes: &[f64],
        resolution: &mut Resolution<'_>,
        loads: &mut Loads<'_>,
    ) -> Result<(), MultibodyError> {
        for e in self {
            let s = take(&mut state, e.state_len())?;
            let m = take(&mut modes, e.mode_len())?;
            e.resolve(ctx, s, m, resolution, loads)?;
        }
        Ok(())
    }

    fn transition(
        &self,
        ctx: &ElementContext<'_, I>,
        mut state: &mut [f64],
        mut modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let mut changed = false;
        for e in self {
            let s = take_mut(&mut state, e.state_len())?;
            let m = take_mut(&mut modes, e.mode_len())?;
            changed |= e.transition(ctx, s, m)?;
        }
        Ok(changed)
    }
}

/// Forwards every method of the contract to `$target`, a place expression
/// over `self` naming another element.
macro_rules! forward_element {
    ($self:ident => $target:expr) => {
        fn state_len(&$self) -> usize {
            $target.state_len()
        }

        fn mode_len(&$self) -> usize {
            $target.mode_len()
        }

        fn has_transitions(&$self) -> bool {
            $target.has_transitions()
        }

        fn initial_state(&$self, state: &mut [f64], modes: &mut [f64]) {
            $target.initial_state(state, modes);
        }

        fn state_scale(&$self, scale: &mut [f64]) {
            $target.state_scale(scale);
        }

        fn evaluate(
            &$self,
            ctx: &ElementContext<'_, I>,
            state: &[f64],
            modes: &[f64],
            loads: &mut Loads<'_>,
            state_dot: &mut [f64],
        ) -> Result<(), MultibodyError> {
            $target.evaluate(ctx, state, modes, loads, state_dot)
        }

        fn has_resolution(&$self) -> bool {
            $target.has_resolution()
        }

        fn resolve(
            &$self,
            ctx: &ElementContext<'_, I>,
            state: &[f64],
            modes: &[f64],
            resolution: &mut Resolution<'_>,
            loads: &mut Loads<'_>,
        ) -> Result<(), MultibodyError> {
            $target.resolve(ctx, state, modes, resolution, loads)
        }

        fn transition(
            &$self,
            ctx: &ElementContext<'_, I>,
            state: &mut [f64],
            modes: &mut [f64],
        ) -> Result<bool, MultibodyError> {
            $target.transition(ctx, state, modes)
        }
    };
}

impl<I: ?Sized, T: ForceElement<I>> ForceElement<I> for Vec<T> {
    forward_element!(self => self.as_slice());
}

impl<I: ?Sized, T: ForceElement<I>, const N: usize> ForceElement<I> for [T; N] {
    forward_element!(self => self.as_slice());
}

impl<I: ?Sized, T: ForceElement<I>> ForceElement<I> for Option<T> {
    forward_element!(self => self.as_slice());
}

impl<I: ?Sized, T: ForceElement<I> + ?Sized> ForceElement<I> for Box<T> {
    forward_element!(self => (**self));
}

impl<I: ?Sized, T: ForceElement<I> + ?Sized> ForceElement<I> for &T {
    forward_element!(self => (**self));
}

/// The empty element set.
impl<I: ?Sized> ForceElement<I> for () {
    fn evaluate(
        &self,
        _ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        _loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        Ok(())
    }
}

macro_rules! tuple_element {
    ($($name:ident : $idx:tt),+) => {
        impl<I: ?Sized, $($name: ForceElement<I>),+> ForceElement<I> for ($($name,)+) {
            fn state_len(&self) -> usize {
                0 $(+ self.$idx.state_len())+
            }

            fn mode_len(&self) -> usize {
                0 $(+ self.$idx.mode_len())+
            }

            fn has_transitions(&self) -> bool {
                false $(|| self.$idx.has_transitions())+
            }

            fn initial_state(&self, mut state: &mut [f64], mut modes: &mut [f64]) {
                $(
                    let s = take_lenient(&mut state, self.$idx.state_len());
                    let m = take_lenient(&mut modes, self.$idx.mode_len());
                    self.$idx.initial_state(s, m);
                )+
            }

            fn state_scale(&self, mut scale: &mut [f64]) {
                $(self.$idx.state_scale(take_lenient(&mut scale, self.$idx.state_len()));)+
            }

            fn evaluate(
                &self,
                ctx: &ElementContext<'_, I>,
                mut state: &[f64],
                mut modes: &[f64],
                loads: &mut Loads<'_>,
                mut state_dot: &mut [f64],
            ) -> Result<(), MultibodyError> {
                $(
                    let n = self.$idx.state_len();
                    let s = take(&mut state, n)?;
                    let m = take(&mut modes, self.$idx.mode_len())?;
                    self.$idx.evaluate(ctx, s, m, loads, take_mut(&mut state_dot, n)?)?;
                )+
                Ok(())
            }

            fn has_resolution(&self) -> bool {
                false $(|| self.$idx.has_resolution())+
            }

            fn resolve(
                &self,
                ctx: &ElementContext<'_, I>,
                mut state: &[f64],
                mut modes: &[f64],
                resolution: &mut Resolution<'_>,
                loads: &mut Loads<'_>,
            ) -> Result<(), MultibodyError> {
                $(
                    let s = take(&mut state, self.$idx.state_len())?;
                    let m = take(&mut modes, self.$idx.mode_len())?;
                    self.$idx.resolve(ctx, s, m, resolution, loads)?;
                )+
                Ok(())
            }

            fn transition(
                &self,
                ctx: &ElementContext<'_, I>,
                mut state: &mut [f64],
                mut modes: &mut [f64],
            ) -> Result<bool, MultibodyError> {
                let mut changed = false;
                $(
                    let s = take_mut(&mut state, self.$idx.state_len())?;
                    let m = take_mut(&mut modes, self.$idx.mode_len())?;
                    changed |= self.$idx.transition(ctx, s, m)?;
                )+
                Ok(changed)
            }
        }
    };
}

tuple_element!(A: 0);
tuple_element!(A: 0, B: 1);
tuple_element!(A: 0, B: 1, C: 2);
tuple_element!(A: 0, B: 1, C: 2, D: 3);
tuple_element!(A: 0, B: 1, C: 2, D: 3, E: 4);
tuple_element!(A: 0, B: 1, C: 2, D: 3, E: 4, F: 5);
