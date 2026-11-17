//! Walls that touch along a line (the retired tangency-faults plan), each family in a
//! random rigid pose from `prop::tangent`: a cylinder tangent inside a
//! larger one, a bar with a boss tangent to each end, and a column tangent
//! to a bore along four lines. Every boolean of the family either builds a
//! body clean at `Full` whose volume is the closed form, with its
//! provenance audited, or is refused by a modelled reason: never
//! `Internal`, never a body the checker refuses. At least half the draws
//! of each family must build. A failure prints the shrunk case and the
//! seed and becomes a fixture under `tests/fixtures/regression/`
//! (`tests/fixtures/README.md` §Property-test failures).

use arris_check::{Level, check};
use arris_debug::prop::{self, tangent::TangentInside};
use arris_debug::testing::{REL, close_to, fail, fitted_rel};
use arris_debug::unmetered::{common, cut, fuse, mass_properties};
use arris_ops::OpError;
use arris_topo::provenance::audit;
use arris_topo::{Body, Model, Provenance};
use proptest::prelude::*;
use std::cell::Cell;

/// What an operation returns.
type Made = Result<(Body, Provenance), OpError>;

/// A refusal the kernel owes a random input: not a fault, not an input
/// that failed the checker.
fn is_refusal(e: &OpError) -> bool {
    !matches!(e, OpError::Internal(_) | OpError::InvalidInput { .. })
}

/// How many operations of a family were asked and how many built.
#[derive(Default)]
struct Tally {
    asked: Cell<usize>,
    built: Cell<usize>,
}

impl Tally {
    /// `result`, which must be a named refusal or a body clean at `Full`
    /// with nothing unchecked, its provenance audited against `inputs` and
    /// its volume `volume`. Whether it built.
    fn settle(
        &self,
        m: &Model,
        name: &str,
        inputs: &[Body],
        result: Made,
        volume: f64,
    ) -> Result<bool, TestCaseError> {
        self.asked.set(self.asked.get() + 1);
        let (body, provenance) = match result {
            Ok(made) => made,
            Err(e) if is_refusal(&e) => return Ok(false),
            Err(e) => return Err(fail(format!("{name}: {e}"))),
        };
        let report = check(m, body, Level::Full);
        if !report.is_ok() || !report.unchecked().is_empty() {
            return Err(fail(format!("{name}: not clean at Full\n{report}")));
        }
        audit(m, inputs, body, &provenance)
            .map_err(|e| fail(format!("{name}: provenance: {e}")))?;
        let p = mass_properties(m, body).map_err(|e| fail(format!("{name}: measure: {e}")))?;
        prop_assert!(
            close_to(p.volume, volume, volume, fitted_rel(m, &p).max(REL)),
            "{}: volume {} against the closed form {}",
            name,
            p.volume,
            volume
        );
        self.built.set(self.built.get() + 1);
        Ok(true)
    }

    /// At least half of what was asked built.
    fn enough(&self, family: &str) {
        assert!(
            2 * self.built.get() >= self.asked.get(),
            "{family}: {} of {} operations built",
            self.built.get(),
            self.asked.get()
        );
    }
}

/// The named exclusion of the first property: the touching ruling within
/// 0.01 rad of the seams but not on them. Its faults are fixed
/// ([`a_ruling_a_hair_off_the_seams_matches_its_closed_forms`]), and it
/// keeps out two findings: where the inner seam crosses a cap one to about
/// sixty tolerances inside the rim, at angles up to about 0.009 as the
/// radii come together, the crescent's polygons cross near the touch and
/// `Full` refuses L5 (`regression/fuse-cylinder-tangent-inside-seam-cusp`,
/// docs/BACKLOG.md, a loop with a cusp at a coarse chord); and from 1e-12
/// to 1e-7 the touch merged into the seam's vertex faults
/// (`regression/fuse-cylinder-tangent-inside-seam-hair`).
fn near_the_seam(case: &TangentInside) -> bool {
    let turn = case.angle.rem_euclid(core::f64::consts::TAU);
    let off = turn.min(core::f64::consts::TAU - turn);
    off > 0.0 && off < 0.01
}

/// A cylinder inside a larger one, touching its wall along a ruling:
/// `fuse`, `common` and both cuts are the closed forms of the two
/// volumes and what they share.
#[test]
fn a_cylinder_tangent_inside_another_matches_its_closed_forms() {
    let tally = Tally::default();
    prop::check(prop::tangent::tangent_inside(), |case| {
        prop_assume!(!near_the_seam(&case));
        let mut m = Model::default();
        let (outer, inner) = case.build(&mut m).map_err(fail)?;
        let (vo, vi, shared) = case.volumes();
        let both = [outer, inner];
        let r = fuse(&mut m, outer, inner);
        tally.settle(&m, "fuse(outer, inner)", &both, r, vo + vi - shared)?;
        let r = common(&mut m, outer, inner);
        tally.settle(&m, "common(outer, inner)", &both, r, shared)?;
        let r = cut(&mut m, outer, inner);
        tally.settle(&m, "cut(outer, inner)", &both, r, vo - shared)?;
        let r = cut(&mut m, inner, outer);
        tally.settle(&m, "cut(inner, outer)", &both, r, vi - shared)?;
        Ok(())
    });
    tally.enough("tangent inside");
}

/// The fixture's cylinders, radius 0.3 tangent inside radius 1.5 and
/// standing out of both its ends, with the touching ruling a hair off both
/// seams on either side, where it faulted (the retired tangency-faults plan):
/// the inner seam's crossing of each cap on the rim within its tolerance
/// (to about 5e-4 rad) or inside it past the polygons' band (from about
/// 2e-3). `fuse`, `common` and both cuts are the closed forms or refused by
/// a modelled reason.
#[test]
fn a_ruling_a_hair_off_the_seams_matches_its_closed_forms() {
    let tally = Tally::default();
    for angle in [1e-6, 1e-5, 1e-4, 3e-4, 5e-4, 2e-3, 3e-3, 5e-3] {
        for angle in [angle, -angle] {
            let case = TangentInside {
                outer: 1.5,
                inner: 0.3,
                angle,
                outer_z: (0.0, 1.0),
                inner_z: (-1.0, 2.0),
                pose: arris_math::Isometry::identity(),
            };
            let mut m = Model::default();
            let (outer, inner) = case.build(&mut m).expect("two cylinders");
            let (vo, vi, shared) = case.volumes();
            let both = [outer, inner];
            let at = |op: &str| format!("{op} at {angle:e} rad");
            let r = fuse(&mut m, outer, inner);
            let built = tally.settle(&m, &at("fuse"), &both, r, vo + vi - shared);
            assert!(
                built.expect("closed form or refusal"),
                "{} refused",
                at("fuse")
            );
            let r = common(&mut m, outer, inner);
            tally
                .settle(&m, &at("common"), &both, r, shared)
                .expect("closed form or refusal");
            let r = cut(&mut m, outer, inner);
            tally
                .settle(&m, &at("cut(outer, inner)"), &both, r, vo - shared)
                .expect("closed form or refusal");
            let r = cut(&mut m, inner, outer);
            tally
                .settle(&m, &at("cut(inner, outer)"), &both, r, vi - shared)
                .expect("closed form or refusal");
        }
    }
    tally.enough("a hair off the seams");
}

/// A bar with a boss tangent to an end, or to each end: the fuse, the chain
/// of two fuses, `common` and both cuts of the first boss are the closed
/// forms.
#[test]
fn a_bar_with_bosses_tangent_to_its_ends_matches_its_closed_forms() {
    let tally = Tally::default();
    prop::check(prop::tangent::bar_bosses(), |case| {
        let mut m = Model::default();
        let (bar, bosses) = case.build(&mut m).map_err(fail)?;
        let specs = case.bosses();
        let (vb, (v1, shared)) = (case.bar_volume(), case.boss_volumes(&specs[0]));
        let first = [bar, bosses[0]];
        let r = fuse(&mut m, bar, bosses[0]);
        let one = match r {
            Ok((body, _)) => Some(body),
            Err(_) => None,
        };
        let r_again = fuse(&mut m, bar, bosses[0]);
        tally.settle(&m, "fuse(bar, boss)", &first, r_again, vb + v1 - shared)?;
        let r = common(&mut m, bar, bosses[0]);
        tally.settle(&m, "common(bar, boss)", &first, r, shared)?;
        let r = cut(&mut m, bar, bosses[0]);
        tally.settle(&m, "cut(bar, boss)", &first, r, vb - shared)?;
        let r = cut(&mut m, bosses[0], bar);
        tally.settle(&m, "cut(boss, bar)", &first, r, v1 - shared)?;
        if let (Some(one), Some(spec)) = (one, specs.get(1)) {
            let (v2, shared2) = case.boss_volumes(spec);
            let r = fuse(&mut m, one, bosses[1]);
            let inputs = [one, bosses[1]];
            tally.settle(
                &m,
                "fuse(fuse(bar, first), second)",
                &inputs,
                r,
                vb + v1 - shared + v2 - shared2,
            )?;
        }
        Ok(())
    });
    tally.enough("bar bosses");
}

/// A box bored through, and a column tangent to the bore along four lines:
/// the bored box's cut by the column and fuse with it, the common part and
/// the column's cut by the bored box are the closed forms.
#[test]
fn a_column_tangent_to_a_bore_matches_its_closed_forms() {
    let tally = Tally::default();
    prop::check(prop::tangent::bore_column(), |case| {
        let mut m = Model::default();
        let (cuboid, bore, column) = case.build(&mut m).map_err(fail)?;
        let pi = core::f64::consts::PI;
        let (a, b, c) = case.half;
        let (r, h) = (case.radius, case.column_half);
        let vbox = 8.0 * a * b * c;
        let bored = cut(&mut m, cuboid, bore);
        let bored_volume = vbox - 2.0 * pi * r * r * c;
        if !tally.settle(
            &m,
            "cut(box, bore)",
            &[cuboid, bore],
            bored.clone(),
            bored_volume,
        )? {
            return Ok(());
        }
        let bored = bored.map_err(fail)?.0;
        let pair = [bored, column];
        let in_box = 8.0 * r * r * c;
        let r1 = cut(&mut m, bored, column);
        tally.settle(&m, "cut(bored, column)", &pair, r1, vbox - in_box)?;
        let r1 = fuse(&mut m, bored, column);
        tally.settle(
            &m,
            "fuse(bored, column)",
            &pair,
            r1,
            vbox + 8.0 * r * r * (h - c),
        )?;
        let r1 = common(&mut m, bored, column);
        tally.settle(
            &m,
            "common(bored, column)",
            &pair,
            r1,
            (4.0 - pi) * r * r * 2.0 * c,
        )?;
        let r1 = cut(&mut m, column, bored);
        tally.settle(
            &m,
            "cut(column, bored)",
            &pair,
            r1,
            8.0 * r * r * h - (4.0 - pi) * r * r * 2.0 * c,
        )?;
        Ok(())
    });
    tally.enough("bore column");
}
