//! The isolation goldens: what the isolations and the arithmetic of this
//! module return on seeded systems, bit for bit, held to committed
//! digests (recorded while the two Bernstein modules became one,
//! `plans/one-bernstein`). A digest is FNV-1a over every word of the
//! result — each zero's `at`, `lo` and `hi` as bits, `certified`, the
//! depth, the boxes looked at and a [`Continuum`]'s front — so a golden
//! that moves names the family and the halving, not the zero.

use arris_math::Meter;

use super::tests::{Seeded, floor, sphere, through, weight};
use super::{Continuum, Gate, Halving, Isolation, PolyN, Wrap, Zero, common_zeros, merge};
use crate::bernstein::Binomials;
use crate::halt::Halt;

/// The 2-D isolations, each family under both halvings.
const GOLDENS_2: &[(&str, u64)] = &[
    ("2/circles/scored", 0xed2f2a7420429ba4),
    ("2/circles/look_ahead", 0xdee30db8725994a7),
    ("2/weighted/scored", 0xa42ef53b7296974b),
    ("2/weighted/look_ahead", 0xcfccbee95ace4599),
    ("2/turning/scored", 0x01fa36b9391182ee),
    ("2/turning/look_ahead", 0xc4bafedbab247481),
    ("2/products/scored", 0x54d0ff03a72725bb),
    ("2/products/look_ahead", 0x2f2fb08bd26dc1a9),
    ("2/gated/scored", 0x86e1b4f0115294ed),
    ("2/gated/look_ahead", 0x59a33f621c0c320c),
    ("2/one_sided/scored", 0x9fbc60794a669b3b),
    ("2/one_sided/look_ahead", 0x7bffc3fb8c366dcb),
    ("2/continuum/scored", 0x264e4d06ac43517d),
    ("2/continuum/look_ahead", 0x264e4d06ac43517d),
    ("2/merged", 0x9bd0b38237975021),
];

/// The 2-D arithmetic: the product, the partials, the parts, the values
/// and the 2-D extras.
const GOLDENS_ARITHMETIC: &[(&str, u64)] = &[
    ("2/arithmetic/mul", 0x17a91eef0d2a63cc),
    ("2/arithmetic/partial", 0xf7d13a9f6f1c2a69),
    ("2/arithmetic/split", 0xdbe269b9134ac7f2),
    ("2/arithmetic/eval", 0x52fd89bd4c5d93d0),
    ("2/arithmetic/lines", 0x09012f990aa0648d),
    ("2/arithmetic/over_linear", 0xc8d0076d0966d208),
    ("2/arithmetic/transposed", 0xac6aaf461b1462d4),
    ("2/arithmetic/restricted", 0x43f11aafd856ef60),
];

/// The 3- and 4-D isolations and merges.
const GOLDENS_N: &[(&str, u64)] = &[
    ("3/posed", 0xc5e8a989141fffad),
    ("3/gated", 0x41200c3dd391e0ab),
    ("3/merged", 0x0c3ae12937aec94a),
    ("3/posed_scored", 0xe4218eadb49b355d),
    ("3/gated_scored", 0x0795af685bc850e2),
    ("4/posed", 0x004b0d439b6470db),
    ("4/gated", 0xa4377bf9b29ebb03),
    ("4/merged", 0xfbb429551d3403de),
    ("4/posed_scored", 0xa68427e30eff5f1f),
    ("4/gated_scored", 0x35a45b1d326a9678),
];

/// FNV-1a over 64-bit words.
struct Digest(u64);

impl Digest {
    fn new() -> Self {
        Digest(0xcbf2_9ce4_8422_2325)
    }

    fn word(&mut self, w: u64) {
        for byte in w.to_le_bytes() {
            self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    fn reals(&mut self, xs: &[f64]) {
        self.word(xs.len() as u64);
        for x in xs {
            self.word(x.to_bits());
        }
    }

    fn poly<const N: usize>(&mut self, p: &PolyN<N>) {
        for d in p.degree() {
            self.word(d as u64);
        }
        self.reals(p.coefficients());
    }

    fn zeros<const N: usize>(&mut self, zeros: &[Zero<N>]) {
        self.word(zeros.len() as u64);
        for z in zeros {
            self.reals(&z.at);
            self.reals(&z.lo);
            self.reals(&z.hi);
            self.word(u64::from(z.certified));
        }
    }

    fn isolation<const N: usize>(&mut self, found: &Result<Isolation<N>, Halt<Continuum<N>>>) {
        match found {
            Ok(found) => {
                self.word(0);
                self.word(found.depth as u64);
                self.word(found.boxes as u64);
                self.zeros(&found.zeros);
            }
            Err(Halt::Fault(continuum)) => {
                self.word(1);
                self.word(continuum.depth as u64);
                self.zeros(&continuum.found);
                self.zeros(&continuum.front);
            }
            Err(Halt::Stopped(_)) => self.word(2),
        }
    }
}

fn floor2(p: &PolyN<2>) -> f64 {
    floor(p)
}

/// `a₀ + aₛ·s + aₜ·t`.
fn linear(a0: f64, a_s: f64, a_t: f64) -> PolyN<2> {
    PolyN::new([1, 1], vec![a0, a0 + a_t, a0 + a_s, a0 + a_s + a_t])
}

/// A random circle meeting the unit square.
fn circle(s: &mut Seeded) -> PolyN<2> {
    let c: [f64; 2] = s.point(0.1, 0.9);
    sphere(c, 0.1 + 0.3 * s.next())
}

/// A random weight `(1 + a·s)(1 + b·t)`.
fn weight2(s: &mut Seeded) -> PolyN<2> {
    weight(s.point(-0.5, 1.0))
}

/// Two circles through two random points, their common zeros.
fn circles(s: &mut Seeded) -> Option<[PolyN<2>; 2]> {
    let (p, q): ([f64; 2], [f64; 2]) = (s.point(0.05, 0.95), s.point(0.05, 0.95));
    let spread: [[f64; 2]; 2] = core::array::from_fn(|_| s.point(-0.4, 0.4));
    let d = [q[0] - p[0], q[1] - p[1]];
    (d[0] * d[0] + d[1] * d[1] >= 0.04).then(|| through(p, q, &spread))
}

/// The 2-D systems of one family, with their floors and gates' parts:
/// `(system, floors, gates as (poly, floor, one_sided))`.
type System2 = ([PolyN<2>; 2], [f64; 2], Vec<(PolyN<2>, f64, bool)>);

fn family2(name: &str, s: &mut Seeded, b: &Binomials) -> Vec<System2> {
    let plain = |system: [PolyN<2>; 2]| {
        let floors = system.each_ref().map(floor2);
        (system, floors, Vec::new())
    };
    let mut systems = Vec::new();
    for _ in 0..12 {
        let system = match name {
            "circles" => circles(s).map(plain),
            "weighted" => circles(s).map(|c| plain(c.map(|f| f.mul(&weight2(s), b)))),
            // The torus tracer's turning points: a curve and its `t` partial.
            "turning" => {
                let f = circle(s).mul(&weight2(s), b);
                let f_t = f.partial(1);
                Some(plain([f, f_t]))
            }
            "products" => {
                let f = circle(s).mul(&circle(s), b);
                let [a, c]: [f64; 2] = s.point(-1.0, 1.0);
                Some(plain([f, linear(-0.5 * (a + c) - 0.25, a, c)]))
            }
            "gated" | "one_sided" => {
                let g = PolyN::combine(&[
                    (1.0, &circle(s).mul(&circle(s), b)),
                    (0.01 * s.next(), &PolyN::outer([&[1.0; 5], &[1.0; 5]])),
                ]);
                let near = 0.02 * s.next();
                let (gs, gt) = (g.partial(0), g.partial(1));
                let floors = [floor2(&gs), floor2(&gt)];
                Some(([gs, gt], floors, vec![(g, near, name == "one_sided")]))
            }
            // `(s − a)(1 + t²)` and its `t` partial share the line `s = a`.
            "continuum" => {
                let t = linear(0.0, 0.0, 1.0);
                let one = PolyN::outer([&[1.0; 3], &[1.0; 3]]);
                let bowl = PolyN::combine(&[(1.0, &one), (1.0, &t.mul(&t, b))]);
                let f = linear(-0.2 - 0.6 * s.next(), 1.0, 0.0).mul(&bowl, b);
                let f_t = f.partial(1);
                // Floors near `f64`'s own, where a curve of zeros keeps
                // its boxes alive.
                let floors = [4.0 * f64::EPSILON; 2];
                (systems.len() < 2).then(|| ([f, f_t], floors, Vec::new()))
            }
            _ => unreachable!("no family {name}"),
        };
        systems.extend(system);
    }
    systems
}

const FAMILIES_2: [&str; 7] = [
    "circles",
    "weighted",
    "turning",
    "products",
    "gated",
    "one_sided",
    "continuum",
];

/// What this module isolates on every 2-D family under both halvings.
fn goldens2() -> Vec<(String, u64)> {
    let b = Binomials::new(16);
    let mut out = Vec::new();
    for (k, name) in FAMILIES_2.into_iter().enumerate() {
        for (halving, label) in [
            (Halving::Scored, "scored"),
            (Halving::LookAhead, "look_ahead"),
        ] {
            let mut s = Seeded(0x0b22 + k as u64);
            let mut digest = Digest::new();
            for (system, floors, gates) in family2(name, &mut s, &b) {
                let gates: Vec<Gate<'_, 2>> = gates
                    .iter()
                    .map(|(poly, floor, one_sided)| Gate {
                        poly,
                        floor: *floor,
                        one_sided: *one_sided,
                    })
                    .collect();
                let found = common_zeros(
                    system.each_ref(),
                    floors,
                    &gates,
                    halving,
                    &mut Meter::default(),
                );
                digest.isolation(&found);
            }
            out.push((format!("2/{name}/{label}"), digest.0));
        }
    }
    out.push(("2/merged".to_owned(), merged2()));
    out
}

/// Random boxes, some certified, a period apart or nearly.
fn boxes<const N: usize>(s: &mut Seeded, count: usize) -> Vec<Zero<N>> {
    (0..count)
        .map(|_| {
            let at: [f64; N] = s.point(-1.0, 7.0);
            let half: [f64; N] = s.point(0.001, 0.3);
            Zero {
                at,
                lo: core::array::from_fn(|k| at[k] - half[k]),
                hi: core::array::from_fn(|k| at[k] + half[k]),
                certified: s.next() < 0.5,
            }
        })
        .collect()
}

fn random_wrap<const N: usize>(s: &mut Seeded) -> Wrap<N> {
    Wrap {
        period: core::array::from_fn(|_| (s.next() < 0.7).then_some(6.0)),
        origin: core::array::from_fn(|_| if s.next() < 0.5 { 0.0 } else { -3.0 }),
        touch: 8.0 * f64::EPSILON,
    }
}

/// The merges at two, wrapped per direction and in both.
fn merged2() -> u64 {
    let mut s = Seeded(0x0e26);
    let mut digest = Digest::new();
    for _ in 0..24 {
        let zeros = boxes::<2>(&mut s, 24);
        let wrap = random_wrap::<2>(&mut s);
        digest.zeros(&merge(zeros.clone(), &wrap));
        let both = Wrap {
            period: [Some(6.0); 2],
            origin: [0.0; 2],
            touch: wrap.touch,
        };
        digest.zeros(&merge(zeros, &both));
    }
    digest.0
}

fn random_poly2(s: &mut Seeded) -> PolyN<2> {
    let degree = [0, 1].map(|_| (5.0 * s.next()) as usize);
    let c = (0..(degree[0] + 1) * (degree[1] + 1))
        .map(|_| 2.0 * s.next() - 1.0)
        .collect();
    PolyN::new(degree, c)
}

/// The 2-D arithmetic on random polynomials of degree up to four.
fn goldens_arithmetic() -> Vec<(String, u64)> {
    let b = Binomials::new(16);
    let mut s = Seeded(0x0a21);
    let names = [
        "mul",
        "partial",
        "split",
        "eval",
        "lines",
        "over_linear",
        "transposed",
        "restricted",
    ];
    let mut digests = names.map(|_| Digest::new());
    for _ in 0..64 {
        let (p, q) = (random_poly2(&mut s), random_poly2(&mut s));
        let [
            mul,
            partial,
            split,
            eval,
            lines,
            over,
            transposed,
            restricted,
        ] = &mut digests;
        mul.poly(&p.mul(&q, &b));
        partial.poly(&p.partial(0));
        partial.poly(&p.partial(1));
        let at = 2.0 * s.next() - 0.5;
        for (lo, hi) in [p.split(0, at), p.split(1, at)] {
            split.poly(&lo);
            split.poly(&hi);
        }
        let x: [f64; 2] = s.point(-0.2, 1.2);
        eval.word(p.eval(x).to_bits());
        lines.reals(&p.across(x[0]));
        lines.reals(&p.along(x[1]));
        let root = 3.0 * s.next() - 1.0;
        let (quotient, rounding) = p.over_linear(root, floor2(&p), &b);
        over.poly(&quotient);
        over.word(rounding.to_bits());
        transposed.poly(&p.transposed());
        let ([s0, s1], [t0, t1]): ([f64; 2], [f64; 2]) = (s.point(0.0, 1.0), s.point(0.0, 1.0));
        restricted.poly(&p.restricted([s0.min(s1), s0.max(s1)], [t0.min(t1), t0.max(t1)]));
    }
    names
        .into_iter()
        .zip(digests)
        .map(|(name, d)| (format!("2/arithmetic/{name}"), d.0))
        .collect()
}

/// `N` spheres through two random points, each times a random weight.
fn spheres<const N: usize>(s: &mut Seeded, b: &Binomials) -> Option<[PolyN<N>; N]> {
    let (p, q): ([f64; N], [f64; N]) = (s.point(0.05, 0.95), s.point(0.05, 0.95));
    let spread: [[f64; N]; N] = core::array::from_fn(|_| s.point(-0.4, 0.4));
    let d2: f64 = (0..N).map(|k| (q[k] - p[k]) * (q[k] - p[k])).sum();
    (d2 >= 0.04).then(|| through(p, q, &spread).map(|f| f.mul(&weight(s.point(-0.5, 1.0)), b)))
}

/// This module's isolations at `N`: posed systems, the same gated to the
/// inside of a random sphere, each under both halvings, and the merges.
fn goldens_n<const N: usize>(cases: usize) -> Vec<(String, u64)> {
    let b = Binomials::new(16);
    let mut s = Seeded(0x0b00 + N as u64);
    let mut digests = [(); 4].map(|()| Digest::new());
    for _ in 0..cases {
        let Some(system) = spheres::<N>(&mut s, &b) else {
            continue;
        };
        let floors = system.each_ref().map(floor);
        let inside = sphere(s.point(0.2, 0.8), 0.3 + 0.3 * s.next());
        let gate = || Gate {
            poly: &inside,
            floor: 0.0,
            one_sided: true,
        };
        for (halving, at) in [(Halving::LookAhead, 0), (Halving::Scored, 2)] {
            let isolate = |gates: &[Gate<'_, N>]| {
                common_zeros(
                    system.each_ref(),
                    floors,
                    gates,
                    halving,
                    &mut Meter::default(),
                )
            };
            digests[at].isolation(&isolate(&[]));
            digests[at + 1].isolation(&isolate(&[gate()]));
        }
    }
    let mut merged = Digest::new();
    for _ in 0..16 {
        let zeros = boxes::<N>(&mut s, 24);
        merged.zeros(&merge(zeros, &random_wrap(&mut s)));
    }
    let [posed, gated, posed_scored, gated_scored] = digests;
    vec![
        (format!("{N}/posed"), posed.0),
        (format!("{N}/gated"), gated.0),
        (format!("{N}/merged"), merged.0),
        (format!("{N}/posed_scored"), posed_scored.0),
        (format!("{N}/gated_scored"), gated_scored.0),
    ]
}

/// Every golden equal to its committed value; a failure prints the
/// table as it stands now, in the source's form.
fn check(found: &[(String, u64)], committed: &[(&str, u64)]) {
    let table: String = found
        .iter()
        .map(|(name, d)| format!("    (\"{name}\", 0x{d:016x}),\n"))
        .collect();
    let same = found.len() == committed.len()
        && found
            .iter()
            .zip(committed)
            .all(|((a, x), (b, y))| a == b && x == y);
    assert!(same, "goldens moved; as they stand:\n{table}");
}

#[test]
fn bernstein_n_goldens_of_the_2d_isolations_hold() {
    check(&goldens2(), GOLDENS_2);
}

#[test]
fn bernstein_n_goldens_of_the_2d_arithmetic_hold() {
    check(&goldens_arithmetic(), GOLDENS_ARITHMETIC);
}

#[test]
fn bernstein_n_goldens_of_the_3d_and_4d_isolations_hold() {
    let mut found = goldens_n::<3>(8);
    found.extend(goldens_n::<4>(4));
    check(&found, GOLDENS_N);
}
