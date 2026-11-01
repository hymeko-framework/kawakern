//! Kawakern: physics over kinematic models, without IR.
//!
//! Articulated-body dynamics, contact and its replayable solvers, the stepping [`Engine`] with
//! rigid attachments, an energy audit and hydraulic muscles, force fields and vehicles. The
//! models come from `hymeko_kinematics` (geometry repository); reading them out of a HyMeKo IR
//! is transformation's `hymeko_geometry`, which re-exports this crate with Core declaration
//! handles. Moved out of that crate unchanged apart from the handle parameter.

pub mod articulated;
pub mod contact;
pub mod engine;
pub mod force_field;
pub mod vehicle;

pub use articulated::{AbaPlan, AbaScratch};
pub use contact::{
    Constraint, ConstraintSolver, ContactParameters, GroundContact, Limit, ground_constraints,
    ground_contacts,
};
pub use engine::{Engine, FreeFlight};
