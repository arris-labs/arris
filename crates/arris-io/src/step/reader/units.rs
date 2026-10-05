//! A representation context's units, and the conversion of every length
//! and angle it holds to the caller's (ADR-0025 §5).
//!
//! The length unit is an `SI_UNIT` of `METRE` with its prefix, or a
//! `CONVERSION_BASED_UNIT` (the inch, the foot) whose factor is a measure
//! of another unit, followed to an `SI_UNIT`. The plane-angle unit is the
//! radian or a conversion-based unit of it (the degree); a context that
//! declares none is in radians, the SI unit. Every length converts to
//! [`ReadOptions::length_unit`](super::ReadOptions) and every angle to
//! radians. The only angle a geometric entity of the subset carries is a
//! cone's semi-angle: a `TRIMMED_CURVE`'s parameters would be another,
//! but the reader takes its basis and never its trim (ADR-0025 §1).
//!
//! `UNCERTAINTY_MEASURE_WITH_UNIT` is the file's *claim*, kept beside the
//! result in the caller's unit; it is never an entity's tolerance
//! (ADR-0025 §4).

use arris_math::{Frame, FrameError, Isometry, Point3, Vec3};

use super::entities::{Args, Entities, describe, malformed, number};
use super::{LengthUnit, Refusal};
use crate::step::part21::Param;

/// How deep a conversion-based unit may nest before the reader stops
/// following it: a real file nests one or two (the inch of millimetres),
/// and a cycle of references must end.
const UNIT_DEPTH: u8 = 8;

/// A unit as `mantissa · 10^exponent` of its SI base unit: an SI prefix
/// is an exponent, so a ratio of two units divides only the mantissas and
/// is exact wherever the ratio is representable.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Scale {
    pub(crate) mantissa: f64,
    pub(crate) exponent: i32,
}

impl Scale {
    const ONE: Scale = Scale {
        mantissa: 1.0,
        exponent: 0,
    };

    /// How many of `other` one of `self` is.
    pub(crate) fn ratio(self, other: Scale) -> f64 {
        let r = self.mantissa / other.mantissa;
        let e = self.exponent - other.exponent;
        // `10^k` is exact for `|k| ≤ 22`; dividing by it, rather than
        // multiplying by its inexact reciprocal, rounds once.
        if e >= 0 {
            r * 10f64.powi(e)
        } else {
            r / 10f64.powi(-e)
        }
    }
}

/// What a named unit measures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dimension {
    Length,
    PlaneAngle,
    Other,
}

/// The conversions of one representation context.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Units {
    /// One file length in the caller's unit.
    pub(crate) length: f64,
    /// One file angle in radians.
    pub(crate) angle: f64,
    /// The smallest length uncertainty the context claims, in the
    /// caller's unit.
    pub(crate) uncertainty: Option<f64>,
    /// The placement an assembly puts the representation at, applied to
    /// every point and direction after the conversion: `None` for a solid
    /// read where it stands (ADR-0025 §5).
    pub(crate) motion: Option<Isometry>,
}

impl Units {
    /// The units of the representation context `context`, converting to
    /// `target`. Errors: [`Refusal::NoLengthUnit`] for a context that
    /// assigns no length unit; [`Refusal::Malformed`] for a unit or a
    /// measure that is not one.
    pub(crate) fn of_context(
        entities: &Entities<'_>,
        context: u64,
        target: LengthUnit,
    ) -> Result<Units, Refusal> {
        let instance = entities.get(context, context)?;
        let no_unit = || Refusal::NoLengthUnit { context };
        let Some(assigned) = instance.record("GLOBAL_UNIT_ASSIGNED_CONTEXT") else {
            return Err(no_unit());
        };
        let assigned = Args {
            id: context,
            record: assigned,
        };
        let mut length = None;
        let mut angle = None;
        for id in assigned.references(0)? {
            let (dimension, scale) = unit(entities, context, id, 0)?;
            match dimension {
                Dimension::Length if length.is_none() => length = Some(scale),
                Dimension::PlaneAngle if angle.is_none() => angle = Some(scale),
                _ => {}
            }
        }
        let length = length.ok_or_else(no_unit)?.ratio(target.scale());
        let angle = angle.unwrap_or(Scale::ONE).ratio(Scale::ONE);
        let mut uncertainty: Option<f64> = None;
        if let Some(claims) = instance.record("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT") {
            let claims = Args {
                id: context,
                record: claims,
            };
            for id in claims.references(0)? {
                let (value, unit_id) = measure(entities, context, id)?;
                let (dimension, scale) = unit(entities, id, unit_id, 0)?;
                if dimension == Dimension::Length {
                    let claimed = value * scale.ratio(target.scale());
                    uncertainty = Some(uncertainty.map_or(claimed, |u| u.min(claimed)));
                }
            }
        }
        Ok(Units {
            length,
            angle,
            uncertainty,
            motion: None,
        })
    }

    /// These units with every point and direction moved by `motion`
    /// after the conversion.
    pub(crate) fn placed(self, motion: Option<Isometry>) -> Units {
        Units { motion, ..self }
    }

    /// A length in the file's unit, in the caller's.
    pub(crate) fn length(&self, x: f64) -> f64 {
        x * self.length
    }

    /// An angle in the file's unit, in radians.
    pub(crate) fn angle(&self, x: f64) -> f64 {
        x * self.angle
    }

    /// A `CARTESIAN_POINT` of three coordinates, in the caller's unit.
    pub(crate) fn point(
        &self,
        entities: &Entities<'_>,
        from: u64,
        id: u64,
    ) -> Result<Point3, Refusal> {
        let args = entities.record(from, id, "CARTESIAN_POINT")?;
        let xs = args.reals(1)?;
        match xs[..] {
            [x, y, z] => {
                let p = Point3::new(self.length(x), self.length(y), self.length(z));
                Ok(self.motion.map_or(p, |m| m.apply(p)))
            }
            _ => Err(args.malformed(format!(
                "a point of {} coordinates in a 3D context",
                xs.len()
            ))),
        }
    }

    /// A `DIRECTION` of three ratios: not a length, so not converted, and
    /// not normalised.
    pub(crate) fn direction(
        &self,
        entities: &Entities<'_>,
        from: u64,
        id: u64,
    ) -> Result<Vec3, Refusal> {
        let args = entities.record(from, id, "DIRECTION")?;
        let xs = args.reals(1)?;
        match xs[..] {
            [x, y, z] => {
                let v = Vec3::new(x, y, z);
                Ok(self.motion.map_or(v, |m| m.apply_vec(v)))
            }
            _ => Err(args.malformed(format!(
                "a direction of {} ratios in a 3D context",
                xs.len()
            ))),
        }
    }

    /// An `AXIS2_PLACEMENT_3D` as a frame: its location converted, its
    /// axis `Z` (world `Z` when unset) and its reference direction's
    /// component perpendicular to it `X`. An unset reference direction is
    /// world `X`, or world `Y` when the axis lies along world `X` — ISO
    /// 10303-42's `first_proj_axis`, which takes world `Y` for world `X`.
    /// "Along" is to rounding, the test [`Frame::new`] makes of any hint:
    /// an exporter writes the axis `(-1, -6.1e-17, 0)` for `-X`, and the
    /// perpendicular part of world `X` is then rounding, not a direction.
    /// Errors: a zero axis or a written reference direction along the
    /// axis, as [`Refusal::Malformed`] naming the placement.
    pub(crate) fn placement(
        &self,
        entities: &Entities<'_>,
        from: u64,
        id: u64,
    ) -> Result<Frame, Refusal> {
        let args = entities.record(from, id, "AXIS2_PLACEMENT_3D")?;
        let origin = self.point(entities, id, args.reference(1)?)?;
        let z = match args.optional_reference(2)? {
            Some(axis) => self.direction(entities, id, axis)?,
            None => Vec3::z(),
        };
        let frame = match args.optional_reference(3)? {
            Some(reference) => Frame::new(origin, z, self.direction(entities, id, reference)?),
            None => match Frame::new(origin, z, Vec3::x()) {
                Err(FrameError::DegenerateHint) => Frame::new(origin, z, Vec3::y()),
                built => built,
            },
        };
        frame.map_err(|e| args.malformed(format!("no frame: {e}")))
    }
}

/// A named unit's dimension and scale, following conversion-based units
/// to an SI one. `from` is what a dangling reference is named against.
fn unit(
    entities: &Entities<'_>,
    from: u64,
    id: u64,
    depth: u8,
) -> Result<(Dimension, Scale), Refusal> {
    if depth >= UNIT_DEPTH {
        return Err(malformed(
            id,
            "a unit converted from units nested too deep to follow",
        ));
    }
    let instance = entities.get(from, id)?;
    let declared = if instance.record("LENGTH_UNIT").is_some() {
        Some(Dimension::Length)
    } else if instance.record("PLANE_ANGLE_UNIT").is_some() {
        Some(Dimension::PlaneAngle)
    } else {
        None
    };
    if let Some(si) = instance.record("SI_UNIT") {
        // `SI_UNIT(prefix, name)` as a partial entity, or with the
        // dimensions before them as a simple instance: the last two.
        let args = Args { id, record: si };
        let n = args.len();
        if n < 2 {
            return Err(args.malformed("SI_UNIT has no prefix and name"));
        }
        let prefix = args.optional_enumeration(n - 2)?;
        let name = args.enumeration(n - 1)?;
        let exponent = match prefix {
            None => 0,
            Some(p) => si_prefix(p)
                .ok_or_else(|| args.malformed(format!("the SI prefix .{p}. is not one")))?,
        };
        let dimension = match name {
            "METRE" => Dimension::Length,
            "RADIAN" => Dimension::PlaneAngle,
            _ => declared.unwrap_or(Dimension::Other),
        };
        return Ok((
            dimension,
            Scale {
                mantissa: 1.0,
                exponent,
            },
        ));
    }
    if let Some(converted) = instance.record("CONVERSION_BASED_UNIT") {
        // `CONVERSION_BASED_UNIT(name, factor)`, or with the dimensions
        // first as a simple instance: the factor is the last.
        let args = Args {
            id,
            record: converted,
        };
        let n = args.len();
        if n < 1 {
            return Err(args.malformed("CONVERSION_BASED_UNIT has no factor"));
        }
        let (value, base) = measure(entities, id, args.reference(n - 1)?)?;
        let (base_dimension, base_scale) = unit(entities, id, base, depth + 1)?;
        if !(value.is_finite() && value > 0.0) {
            return Err(args.malformed(format!("a conversion factor of {value}")));
        }
        let scale = Scale {
            mantissa: value * base_scale.mantissa,
            exponent: base_scale.exponent,
        };
        return Ok((declared.unwrap_or(base_dimension), scale));
    }
    match declared {
        // A unit of a known dimension that is neither SI nor converted is
        // a context-dependent one, which has no size.
        Some(_) => Err(malformed(
            id,
            format!("{} is a unit with no size", describe(instance)),
        )),
        None => Ok((Dimension::Other, Scale::ONE)),
    }
}

/// A measure with unit's value and its unit's id: `MEASURE_WITH_UNIT` or
/// any of its subtypes, simple or as a partial entity.
fn measure(entities: &Entities<'_>, from: u64, id: u64) -> Result<(f64, u64), Refusal> {
    let instance = entities.get(from, id)?;
    let record = instance
        .records()
        .iter()
        .find(|r| r.name.ends_with("MEASURE_WITH_UNIT") && r.params.len() >= 2)
        .ok_or_else(|| {
            malformed(
                id,
                format!(
                    "is {}, where a measure with unit belongs",
                    describe(instance)
                ),
            )
        })?;
    let args = Args { id, record };
    let value = number(&record.params[0])
        .ok_or_else(|| args.malformed("a measure whose value is not a number"))?;
    let unit = match &record.params[1] {
        Param::Ref(u) => *u,
        _ => return Err(args.malformed("a measure whose unit is not a reference")),
    };
    Ok((value, unit))
}

/// The power of ten of an SI prefix.
fn si_prefix(p: &str) -> Option<i32> {
    Some(match p {
        "EXA" => 18,
        "PETA" => 15,
        "TERA" => 12,
        "GIGA" => 9,
        "MEGA" => 6,
        "KILO" => 3,
        "HECTO" => 2,
        "DECA" => 1,
        "DECI" => -1,
        "CENTI" => -2,
        "MILLI" => -3,
        "MICRO" => -6,
        "NANO" => -9,
        "PICO" => -12,
        "FEMTO" => -15,
        "ATTO" => -18,
        _ => return None,
    })
}

/// The context of the first instance holding a record named
/// `representation`, for the tests' one-block files.
#[cfg(test)]
pub(crate) fn context_of(
    instances: &std::collections::BTreeMap<u64, crate::step::part21::Instance>,
    representation: &str,
) -> Option<u64> {
    instances.values().find_map(|i| {
        let r = i.record(representation)?;
        match r.params.get(2) {
            Some(Param::Ref(c)) => Some(*c),
            _ => None,
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::step::part21;
    use arris_geom::Surface;
    use arris_math::is_negligible;
    use core::f64::consts::PI;

    /// A one-block file: `units` are the instances `#10` onwards that the
    /// context `#9` assigns as `assigned`, and `#1` to `#8` a cone placed
    /// off the origin, of radius `radius` and semi-angle `angle` in the
    /// file's units.
    fn file(units: &str, assigned: &str, coordinates: [f64; 3], radius: f64, angle: f64) -> String {
        let [x, y, z] = coordinates;
        format!(
            "ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#5),#9);
#2=CARTESIAN_POINT('',({x:?},{y:?},{z:?}));
#3=DIRECTION('',(0.,0.6,0.8));
#4=DIRECTION('',(1.,0.,0.));
#5=CONICAL_SURFACE('',#6,{radius:?},{angle:?});
#6=AXIS2_PLACEMENT_3D('',#2,#3,#4);
#9=( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#20)) GLOBAL_UNIT_ASSIGNED_CONTEXT(({assigned})) REPRESENTATION_CONTEXT('','') );
{units}
ENDSEC;
END-ISO-10303-21;
"
        )
    }

    /// The cone of [`file`], read as `target`.
    fn cone(text: &str, target: LengthUnit) -> Result<(Surface, Units), Refusal> {
        let x = part21::parse(text).unwrap();
        let entities = Entities::new(&x.instances);
        let context = context_of(&x.instances, "ADVANCED_BREP_SHAPE_REPRESENTATION").unwrap();
        let units = Units::of_context(&entities, context, target)?;
        let args = entities.record(1, 5, "CONICAL_SURFACE")?;
        let frame = units.placement(&entities, 5, args.reference(1)?)?;
        let surface = Surface::Cone {
            frame,
            radius: units.length(args.real(2)?),
            half_angle: units.angle(args.real(3)?),
        };
        Ok((surface, units))
    }

    const RADIAN: &str = "#11=( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );";
    const DEGREE: &str =
        "#11=( CONVERSION_BASED_UNIT('DEGREE',#13) NAMED_UNIT(#14) PLANE_ANGLE_UNIT() );
#12=( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );
#13=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.017453292519943295),#12);
#14=DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);";

    fn close(a: f64, b: f64) -> bool {
        is_negligible(a - b, a.abs().max(b.abs()))
    }

    fn same_cone(a: &Surface, b: &Surface) {
        let (
            Surface::Cone {
                frame: fa,
                radius: ra,
                half_angle: ha,
            },
            Surface::Cone {
                frame: fb,
                radius: rb,
                half_angle: hb,
            },
        ) = (a, b)
        else {
            panic!("{a:?} {b:?}");
        };
        let o = (fa.origin() - fb.origin()).norm();
        assert!(is_negligible(o, fa.origin().coords.norm()), "{fa:?} {fb:?}");
        assert!((fa.z().into_inner() - fb.z().into_inner()).norm() == 0.0);
        assert!((fa.x().into_inner() - fb.x().into_inner()).norm() == 0.0);
        assert!(close(*ra, *rb), "{ra} {rb}");
        assert!(close(*ha, *hb), "{ha} {hb}");
    }

    #[test]
    fn millimetres_metres_and_inches_read_to_the_same_geometry() {
        let mm = file(
            &format!(
                "#10=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n{RADIAN}\n#20=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.001),#10,'distance_accuracy_value','');"
            ),
            "#10,#11",
            [25.4, -50.8, 254.0],
            12.7,
            PI / 6.0,
        );
        let metres = file(
            &format!(
                "#10=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) );\n{RADIAN}\n#20=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.000001),#10,'distance_accuracy_value','');"
            ),
            "#10,#11",
            [0.0254, -0.0508, 0.254],
            0.0127,
            PI / 6.0,
        );
        // The inch as a conversion of the millimetre, as writers put it,
        // and its uncertainty given in a unit of its own.
        let inches = file(
            &format!(
                "#10=( CONVERSION_BASED_UNIT('INCH',#15) LENGTH_UNIT() NAMED_UNIT(#16) );\n{RADIAN}\n#15=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#17);\n#16=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);\n#17=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n#20=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.001),#17,'distance_accuracy_value','');"
            ),
            "#10,#11",
            [1.0, -2.0, 10.0],
            0.5,
            PI / 6.0,
        );
        let (reference, units) = cone(&mm, LengthUnit::Millimetre).unwrap();
        assert_eq!(
            units.length, 1.0,
            "millimetres to millimetres is exactly one"
        );
        assert_eq!(units.uncertainty, Some(0.001));
        for text in [&metres, &inches] {
            let (read, units) = cone(text, LengthUnit::Millimetre).unwrap();
            same_cone(&read, &reference);
            assert!(close(units.uncertainty.unwrap(), 0.001), "{units:?}");
        }
        let (_, units) = cone(&inches, LengthUnit::Millimetre).unwrap();
        assert_eq!(units.length, 25.4, "inches to millimetres is exactly 25.4");
        // The same file read in inches is the inch file read as itself.
        let (in_inches, units) = cone(&mm, LengthUnit::Inch).unwrap();
        let (native, _) = cone(&inches, LengthUnit::Inch).unwrap();
        same_cone(&in_inches, &native);
        assert!(close(units.length, 1.0 / 25.4));
        // A foot as a conversion of the inch: units nest.
        let feet = file(
            &format!(
                "#10=( CONVERSION_BASED_UNIT('FOOT',#15) LENGTH_UNIT() NAMED_UNIT(#16) );\n{RADIAN}\n#15=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(12.),#18);\n#16=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);\n#17=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n#18=( CONVERSION_BASED_UNIT('INCH',#19) LENGTH_UNIT() NAMED_UNIT(#16) );\n#19=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#17);\n#20=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.001),#17,'','');"
            ),
            "#10,#11",
            [1.0 / 12.0, -2.0 / 12.0, 10.0 / 12.0],
            0.5 / 12.0,
            PI / 6.0,
        );
        let (read, _) = cone(&feet, LengthUnit::Millimetre).unwrap();
        same_cone(&read, &reference);
    }

    #[test]
    fn a_semi_angle_in_degrees_reads_in_radians() {
        let mm = "#10=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n#20=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.001),#10,'','');";
        let radians = file(
            &format!("{mm}\n{RADIAN}"),
            "#10,#11",
            [1.0, 2.0, 3.0],
            4.0,
            PI / 6.0,
        );
        let degrees = file(
            &format!("{mm}\n{DEGREE}"),
            "#11,#10",
            [1.0, 2.0, 3.0],
            4.0,
            30.0,
        );
        let (a, _) = cone(&radians, LengthUnit::Millimetre).unwrap();
        let (b, units) = cone(&degrees, LengthUnit::Millimetre).unwrap();
        same_cone(&a, &b);
        assert_eq!(units.length, 1.0);
        // No angle unit at all is radians.
        let bare = file(mm, "#10", [1.0, 2.0, 3.0], 4.0, PI / 6.0);
        let (c, _) = cone(&bare, LengthUnit::Millimetre).unwrap();
        same_cone(&a, &c);
    }

    #[test]
    fn a_context_with_no_length_unit_is_refused() {
        let no_length = file(
            &format!(
                "{RADIAN}\n#20=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.001),#11,'','');"
            ),
            "#11",
            [0.0; 3],
            1.0,
            0.5,
        );
        assert_eq!(
            cone(&no_length, LengthUnit::Millimetre).unwrap_err(),
            Refusal::NoLengthUnit { context: 9 }
        );
        // A context that assigns no units at all.
        let text = file(RADIAN, "#11", [0.0; 3], 1.0, 0.5)
            .replace("GLOBAL_UNIT_ASSIGNED_CONTEXT((#11)) ", "");
        assert_eq!(
            cone(&text, LengthUnit::Millimetre).unwrap_err(),
            Refusal::NoLengthUnit { context: 9 }
        );
    }

    #[test]
    fn a_malformed_unit_is_refused_naming_it() {
        let mm = "#10=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLY.,.METRE.) );\n#20=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.001),#10,'','');";
        let e = cone(&file(mm, "#10", [0.0; 3], 1.0, 0.5), LengthUnit::Millimetre).unwrap_err();
        assert!(matches!(e, Refusal::Malformed { entity: 10, .. }), "{e}");
        // A conversion that refers to itself stops.
        let cycle = "#10=( CONVERSION_BASED_UNIT('LOOP',#15) LENGTH_UNIT() NAMED_UNIT(*) );\n#15=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(2.),#10);\n#20=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.001),#10,'','');";
        let e = cone(
            &file(cycle, "#10", [0.0; 3], 1.0, 0.5),
            LengthUnit::Millimetre,
        )
        .unwrap_err();
        assert!(matches!(e, Refusal::Malformed { entity: 10, .. }), "{e}");
        // A unit the context names that the file does not define.
        let e = cone(&file("", "#10", [0.0; 3], 1.0, 0.5), LengthUnit::Millimetre).unwrap_err();
        assert!(matches!(e, Refusal::Malformed { entity: 9, .. }), "{e}");
    }

    #[test]
    fn placements_default_their_axes_as_the_standard_builds_them() {
        let text = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('','',(''),(''),'','','');\nFILE_SCHEMA(('X'));\nENDSEC;\nDATA;\n#1=CARTESIAN_POINT('',(1.,2.,3.));\n#2=AXIS2_PLACEMENT_3D('',#1,$,$);\n#3=DIRECTION('',(2.,0.,0.));\n#4=AXIS2_PLACEMENT_3D('',#1,#3,$);\n#5=AXIS2_PLACEMENT_3D('',#1,#3,#3);\n#6=CARTESIAN_POINT('',(1.,2.));\n#7=AXIS2_PLACEMENT_3D('',#6,$,$);\n#8=DIRECTION('',(-1.,-6.12323399574E-17,-0.));\n#9=AXIS2_PLACEMENT_3D('',#1,#8,$);\nENDSEC;\nEND-ISO-10303-21;\n";
        let x = part21::parse(text).unwrap();
        let e = Entities::new(&x.instances);
        let units = Units {
            length: 10.0,
            angle: 1.0,
            uncertainty: None,
            motion: None,
        };
        let f = units.placement(&e, 2, 2).unwrap();
        assert_eq!(f.origin(), Point3::new(10.0, 20.0, 30.0));
        assert_eq!(
            (f.z().into_inner(), f.x().into_inner()),
            (Vec3::z(), Vec3::x())
        );
        let f = units.placement(&e, 4, 4).unwrap();
        assert_eq!(
            (f.z().into_inner(), f.x().into_inner()),
            (Vec3::x(), Vec3::y())
        );
        // An axis along -X to rounding, as NIST's CTC-04 writes it.
        let f = units.placement(&e, 9, 9).unwrap();
        assert!(
            (f.x().into_inner() - Vec3::y()).norm() < 1e-15,
            "{:?}",
            f.x()
        );
        assert!(matches!(
            units.placement(&e, 5, 5),
            Err(Refusal::Malformed { entity: 5, .. })
        ));
        assert!(matches!(
            units.placement(&e, 7, 7),
            Err(Refusal::Malformed { entity: 6, .. })
        ));
    }
}
