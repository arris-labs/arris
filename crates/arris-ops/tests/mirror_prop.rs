//! The mirror's algebra (ADR-0031) at random poses: a box and a cylinder
//! that overlap (`prop::body::overlapping_pair`) mirrored in a random
//! plane through the workspace. The image of each operand is clean at
//! `Full`, keeps its volume and area and has the reflected centroid;
//! `mirror ∘ mirror` is the original's measures; and a boolean of the
//! mirrors is the mirror of the boolean — `fuse`, `common` and `cut` —
//! in volume, area and centroid.

use arris_check::{Level, check};
use arris_debug::testing::{REL, close_to, fail};
use arris_debug::unmetered::{common, cut, fuse, mass_properties, mirror};
use arris_debug::{prop, prop_shards};
use arris_math::{Point3, Reflection, UnitVec3};
use arris_ops::OpError;
use arris_ops::measure::MassProperties;
use arris_topo::{Body, Model, Provenance};
use proptest::prelude::*;

/// A random plane: a point of the workspace and a unit normal.
fn plane() -> impl Strategy<Value = (Point3, UnitVec3)> {
    (prop::point_in_box(prop::DEFAULT_SCALE), prop::unit_vec3())
}

type Boolean = fn(&mut Model, Body, Body) -> Result<(Body, Provenance), OpError>;

/// `body` mirrored, clean at `Full` with nothing unchecked.
fn image(m: &mut Model, name: &str, body: Body, r: &Reflection) -> Result<Body, TestCaseError> {
    let (out, _) = mirror(m, body, r).map_err(|e| fail(format!("{name}: {e}")))?;
    let report = check(m, out, Level::Full);
    if !report.is_ok() || !report.unchecked().is_empty() {
        return Err(fail(format!("{name}: {report}")));
    }
    Ok(out)
}

/// `a` and `b` agree in volume, area and centroid, `b` being `a`'s
/// reflection in `r` when `reflected`.
fn same(
    name: &str,
    a: &MassProperties,
    b: &MassProperties,
    r: Option<&Reflection>,
) -> Result<(), TestCaseError> {
    let floor = a.volume.abs().max(1.0);
    let centre = r.map_or(a.centroid, |r| r.apply(a.centroid));
    let ok = close_to(a.volume, b.volume, floor, REL)
        && close_to(a.area, b.area, floor, REL)
        && (centre - b.centroid).norm() <= 1e-9 * (centre.coords.abs().max() + floor.cbrt());
    if ok {
        Ok(())
    } else {
        Err(fail(format!("{name}: {a:?} against {b:?}")))
    }
}

prop_shards! {
    /// Each operand's image keeps its measures, reflected, and mirrors
    /// back to them.
    a_mirror_reflects_and_mirrors_back
        [shard_0 shard_1 shard_2 shard_3]
        ((pair, (o, n))) = (prop::body::overlapping_pair(), plane()) => {
            let r = Reflection::new(o, n.into_inner()).map_err(fail)?;
            let mut m = Model::default();
            let (a, b) = pair.build(&mut m).map_err(fail)?;
            for (name, body) in [("box", a), ("cylinder", b)] {
                let before = mass_properties(&m, body).map_err(fail)?;
                let img = image(&mut m, name, body, &r)?;
                let after = mass_properties(&m, img).map_err(fail)?;
                prop_assert!(after.volume > 0.0, "{}: inside out", name);
                same(name, &before, &after, Some(&r))?;
                let back = image(&mut m, name, img, &r)?;
                let again = mass_properties(&m, back).map_err(fail)?;
                same(name, &before, &again, None)?;
            }
            Ok(())
        }
}

prop_shards! {
    /// `mirror(op(a, b))` and `op(mirror(a), mirror(b))` are one solid
    /// for `fuse`, `common` and `cut`.
    a_mirror_of_a_boolean_is_the_boolean_of_the_mirrors
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        ((pair, (o, n))) = (prop::body::overlapping_pair(), plane()) => {
            let r = Reflection::new(o, n.into_inner()).map_err(fail)?;
            let mut m = Model::default();
            let (a, b) = pair.build(&mut m).map_err(fail)?;
            let (ia, ib) = (image(&mut m, "a", a, &r)?, image(&mut m, "b", b, &r)?);
            let ops: [(&str, Boolean); 3] = [("fuse", fuse), ("common", common), ("cut", cut)];
            for (name, op) in ops {
                let (whole, _) = op(&mut m, a, b).map_err(|e| fail(format!("{name}(a, b): {e}")))?;
                let (of_images, _) =
                    op(&mut m, ia, ib).map_err(|e| fail(format!("{name}(ia, ib): {e}")))?;
                let (mirrored, _) = mirror(&mut m, whole, &r).map_err(|e| fail(format!("{name}: {e}")))?;
                let report = check(&m, of_images, Level::Full);
                prop_assert!(report.is_ok(), "{}(ia, ib): {}", name, report);
                let want = mass_properties(&m, mirrored).map_err(fail)?;
                let got = mass_properties(&m, of_images).map_err(fail)?;
                same(name, &want, &got, None)?;
            }
            Ok(())
        }
}
