//! Kawakern: physics over kinematic models, without IR.
//!
//! Articulated-body dynamics, contact and its replayable solvers, the stepping [`Engine`] with
//! rigid attachments, an energy audit and hydraulic muscles, force fields, vehicles and the closed
//! roads they drive on ([`track`]). The models come from `hymeko_kinematics` (geometry
//! repository); reading a mechanism or a road out of HyMeKo IR is transformation's
//! `hymeko_mechanism` and `hymeko_formats::environment`. Moved out of transformation's former
//! `hymeko_geometry` unchanged apart from the handle parameter.

pub mod articulated;
pub mod contact;
pub mod engine;
pub mod force_field;
pub mod track;
pub mod vehicle;

pub use articulated::{AbaPlan, AbaScratch};
pub use contact::{
    Constraint, ConstraintSolver, ContactParameters, GroundContact, Limit, ground_constraints,
    ground_contacts,
};
pub use engine::{Engine, FreeFlight};
