//! Closed roads for vehicle scenes: a rational centreline, its arc-length table, wrapped sampling,
//! nearest-point projection and static obstacles. Tensor owns rational spline evaluation.
//!
//! A road is built from plain data ([`TrackSpec`]); reading one out of HyMeKo IR is
//! transformation's `hymeko_formats::environment::track`. Moved from transformation's
//! `hymeko_geometry::track` (2026-11-01) with the source handles as a parameter, as for the
//! kinematic model.

use hymeko_kinematics::handles::Handles;
use hymeko_spline::nurbs::NurbsCurve;

/// A control point of the centreline, in planar world coordinates, with its rational weight.
#[derive(Clone, Debug)]
pub struct TrackControl<H: Handles> {
    pub id: H::Id,
    pub point: [f64; 2],
    pub weight: f64,
}

/// An obstacle as authored: where along the road, how far to the left, how large.
#[derive(Clone, Debug)]
pub struct ObstacleSpec<H: Handles> {
    pub id: H::Id,
    /// Position along the road as a fraction of its length, in `[0, 1)`.
    pub fraction: f64,
    /// Lateral offset in metres; positive is left.
    pub offset: f64,
    pub radius: f64,
    pub height: f64,
}

/// Static cylindrical obstacle in road coordinates; positive offset is left.
#[derive(Clone, Debug)]
pub struct TrackObstacle<H: Handles> {
    pub id: H::Id,
    pub station: f64,
    pub offset: f64,
    pub radius: f64,
    pub height: f64,
    pub position: [f64; 2],
}

/// Road frame in metres; curvature is signed and measured in inverse metres.
#[derive(Clone, Copy, Debug)]
pub struct TrackSample {
    pub position: [f64; 2],
    pub tangent: [f64; 2],
    pub curvature: f64,
    pub station: f64,
    pub parameter: f64,
}

impl TrackSample {
    /// Point offset to the left of the centreline, in metres.
    pub fn offset(self, lateral: f64) -> [f64; 2] {
        [
            self.position[0] - lateral * self.tangent[1],
            self.position[1] + lateral * self.tangent[0],
        ]
    }
}

/// Nearest road position; station wraps at the closed seam.
#[derive(Clone, Copy, Debug)]
pub struct TrackProjection {
    pub station: f64,
    pub lateral: f64,
    pub separation: f64,
}

/// Everything a road is built from.
#[derive(Clone, Debug)]
pub struct TrackSpec<H: Handles> {
    pub id: H::Id,
    /// Road width in metres.
    pub width: f64,
    /// Arc intervals of the lookup table.
    pub samples: usize,
    pub degree: usize,
    pub knots: Vec<f64>,
    pub controls: Vec<TrackControl<H>>,
    pub obstacles: Vec<ObstacleSpec<H>>,
}

/// Immutable bounded road. The lookup table is constructed once, never per tick.
#[derive(Clone, Debug)]
pub struct Track<H: Handles> {
    pub id: H::Id,
    pub width: f64,
    pub controls: Vec<TrackControl<H>>,
    pub obstacles: Vec<TrackObstacle<H>>,
    curve: NurbsCurve<2>,
    samples: Vec<TrackSample>,
    length: f64,
}

impl<H: Handles> Track<H> {
    /// Build a closed rational road and place its obstacles.
    /// # Preconditions
    /// Degree 1..5, 2..256 ordered controls, finite positive weights, at most 262 sorted knots.
    /// Road width 4..40 m; 256..8192 arc intervals; at most 32 obstacles.
    /// # Postconditions
    /// Curve positions/tangents close, every table frame is regular, original IDs survive.
    /// # Errors
    /// Invalid fields, degenerate/open curves, excessive size or invalid obstacles.
    pub fn new(spec: TrackSpec<H>) -> Result<Self, String> {
        let TrackSpec {
            id,
            width,
            samples: count,
            degree,
            knots,
            controls,
            obstacles,
        } = spec;
        if !(4.0..=40.0).contains(&width) {
            return Err("road width must be 4..40 metres".into());
        }
        if !(256..=8192).contains(&count) {
            return Err("track samples must be 256..8192".into());
        }
        if !(1..=5).contains(&degree) {
            return Err("track degree must be 1..5".into());
        }
        if controls.len() > 256 {
            return Err("more than 256 control points".into());
        }
        if controls
            .iter()
            .any(|c| c.point.iter().any(|v| !v.is_finite()) || !c.weight.is_finite() || c.weight <= 0.0)
        {
            return Err("control points must be finite with positive weights".into());
        }
        if knots.len() > 262 {
            return Err("more than 262 knots".into());
        }
        if knots.iter().any(|k| !k.is_finite()) {
            return Err("knots must be finite".into());
        }
        let points: Vec<_> = controls.iter().map(|c| c.point).collect();
        let weights: Vec<_> = controls.iter().map(|c| c.weight).collect();
        let curve = NurbsCurve::new(degree, &points, &weights, &knots).map_err(|e| e.to_string())?;
        let samples = Self::table(&curve, count)?;
        let length = validate_closed(width, &samples)?;
        let mut track = Self {
            id,
            width,
            controls,
            obstacles: Vec::new(),
            curve,
            samples,
            length,
        };
        track.place_obstacles(obstacles)?;
        Ok(track)
    }

    fn place_obstacles(&mut self, specs: Vec<ObstacleSpec<H>>) -> Result<(), String> {
        if specs.len() > 32 {
            return Err("more than 32 obstacles".into());
        }
        for spec in specs {
            if !(0.0..1.0).contains(&spec.fraction)
                || !spec.offset.is_finite()
                || spec.offset.abs() > self.width * 0.5
                || !spec.radius.is_finite()
                || spec.radius <= 0.0
                || spec.radius > 40.0
                || !spec.height.is_finite()
                || spec.height <= 0.0
                || spec.height > 10.0
            {
                return Err("invalid obstacle road coordinates or dimensions".into());
            }
            let station = spec.fraction * self.length;
            let position = self.sample(station)?.offset(spec.offset);
            self.obstacles.push(TrackObstacle {
                id: spec.id,
                station,
                offset: spec.offset,
                radius: spec.radius,
                height: spec.height,
                position,
            });
        }
        Ok(())
    }

    fn frame(curve: &NurbsCurve<2>, parameter: f64, station: f64) -> Result<TrackSample, String> {
        let s = curve.sample(parameter).map_err(|e| e.to_string())?;
        let speed = norm(s.first);
        if !speed.is_finite() || speed < 1e-9 {
            return Err("singular track tangent".into());
        }
        let curvature = (s.first[0] * s.second[1] - s.first[1] * s.second[0]) / speed.powi(3);
        if !curvature.is_finite() {
            return Err("nonfinite track curvature".into());
        }
        Ok(TrackSample {
            position: s.position,
            tangent: [s.first[0] / speed, s.first[1] / speed],
            curvature,
            station,
            parameter,
        })
    }

    fn table(curve: &NurbsCurve<2>, count: usize) -> Result<Vec<TrackSample>, String> {
        let [lo, hi] = curve.domain();
        let mut samples: Vec<TrackSample> = Vec::with_capacity(count + 1);
        for i in 0..=count {
            let u = if i == count {
                hi
            } else {
                lo + (hi - lo) * (i as f64 / count as f64)
            };
            let mut s = Self::frame(curve, u, 0.0)?;
            if let Some(previous) = samples.last() {
                let ds = norm(sub(s.position, previous.position));
                if !ds.is_finite() || ds < 1e-9 {
                    return Err("degenerate arc interval".into());
                }
                s.station = previous.station + ds;
            }
            samples.push(s);
        }
        Ok(samples)
    }

    /// Centreline arc length estimated by the validated polyline table, in metres.
    pub fn length(&self) -> f64 {
        self.length
    }

    /// Cached frames including the duplicated seam, for one-time mesh construction.
    pub fn samples(&self) -> &[TrackSample] {
        &self.samples
    }

    /// Evaluate the rational curve at a wrapped arc distance in O(log N), allocation-free.
    /// # Errors
    /// Nonfinite station or an ill-conditioned spline sample.
    pub fn sample(&self, station: f64) -> Result<TrackSample, String> {
        if !station.is_finite() {
            return Err("station must be finite".into());
        }
        let station = station.rem_euclid(self.length);
        let i = self
            .samples
            .partition_point(|s| s.station <= station)
            .clamp(1, self.samples.len() - 1);
        let a = self.samples[i - 1];
        let b = self.samples[i];
        let fraction = (station - a.station) / (b.station - a.station);
        Self::frame(
            &self.curve,
            a.parameter + fraction * (b.parameter - a.parameter),
            station,
        )
    }

    /// Project onto the cached centreline. An optional prior station and search radius
    /// restrict the search to a local arc window; None performs global initialization.
    /// Returns None for invalid inputs or an empty search window.
    pub fn project(&self, point: [f64; 2], window: Option<(f64, f64)>) -> Option<TrackProjection> {
        if point.iter().any(|v| !v.is_finite())
            || window.is_some_and(|(s, r)| !s.is_finite() || !r.is_finite() || r <= 0.0)
        {
            return None;
        }
        let mut best = None;
        let mut best_distance = f64::INFINITY;
        // Binary search selects bounded windows, avoiding a full-track scan per tick.
        let (start, end) = window.map_or((0, self.samples.len() - 1), |(s, r)| {
            let s = s.rem_euclid(self.length);
            if r >= self.length * 0.5 || s - r < 0.0 || s + r >= self.length {
                (0, self.samples.len() - 1)
            } else {
                (
                    self.samples
                        .partition_point(|x| x.station < s - r)
                        .saturating_sub(1),
                    self.samples
                        .partition_point(|x| x.station <= s + r)
                        .min(self.samples.len() - 1),
                )
            }
        });
        for pair in self.samples[start..=end].windows(2) {
            let a = pair[0];
            let b = pair[1];
            if let Some((s, r)) = window {
                let delta =
                    (a.station - s + self.length * 0.5).rem_euclid(self.length) - self.length * 0.5;
                if delta.abs() > r + (b.station - a.station) {
                    continue;
                }
            }
            let d = sub(b.position, a.position);
            let q = sub(point, a.position);
            let t = ((q[0] * d[0] + q[1] * d[1]) / (d[0] * d[0] + d[1] * d[1])).clamp(0.0, 1.0);
            let residual = [q[0] - t * d[0], q[1] - t * d[1]];
            let separation = norm(residual);
            if separation < best_distance {
                best_distance = separation;
                best = Some(TrackProjection {
                    station: (a.station + t * (b.station - a.station)).rem_euclid(self.length),
                    lateral: (d[0] * residual[1] - d[1] * residual[0]) / norm(d),
                    separation,
                });
            }
        }
        best
    }
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn norm(v: [f64; 2]) -> f64 {
    v[0].hypot(v[1])
}

fn validate_closed(width: f64, samples: &[TrackSample]) -> Result<f64, String> {
    let last = *samples.last().ok_or("track has no samples")?;
    let first = samples[0];
    let length = last.station;
    if !(100.0..=100_000.0).contains(&length)
        || norm(sub(first.position, last.position)) > 1e-6
        || norm(sub(first.tangent, last.tangent)) > 1e-6
    {
        return Err("track must close in position and tangent, length 100..100000 metres".into());
    }
    if samples
        .iter()
        .any(|s| s.curvature.abs() * width * 0.5 >= 0.8)
    {
        return Err("track turn radius too small for its road width".into());
    }
    Ok(length)
}
