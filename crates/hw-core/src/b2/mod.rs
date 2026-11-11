//! Box2DFlashAS3 2.0.2 port. One module per AS3 class family; functions are named after the AS3
//! methods (snake_case) and take `ctx: &mut Ctx<R>` plus the object handle in place of `this`.
pub mod math;
pub mod settings;
pub mod state;
// Ported class families (filled in by the port):
pub mod shapes;      // b2Shape, b2CircleShape, b2PolygonShape, b2ShapeDef/b2CircleDef/b2PolygonDef, b2MassData
pub mod collision;   // b2Collision, b2Distance, b2TimeOfImpact, b2AABB, b2OBB, b2Segment(unused)
pub mod broad_phase; // b2BroadPhase, b2PairManager, b2Bound, b2Proxy, b2Pair, b2BufferedPair
pub mod contacts;    // b2Contact, b2CircleContact, b2PolyAndCircleContact, b2PolygonContact, b2ContactManager, b2NullContact
pub mod body;        // b2Body, b2BodyDef
pub mod joints;      // b2Joint(+Def), b2RevoluteJoint, b2PrismaticJoint, b2DistanceJoint, b2Jacobian
pub mod island;      // b2Island, b2ContactSolver, b2ContactConstraint(Point)
pub mod world;       // b2World, b2TimeStep

#[cfg(test)]
mod geometry_fixture;
