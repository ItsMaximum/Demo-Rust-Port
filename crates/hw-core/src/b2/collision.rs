//! b2Collision / b2Distance / b2TimeOfImpact / b2AABB / b2OBB / ClipVertex / b2ContactID
//! (Box2DFlashAS3 2.0.2, `Collision/*.as`), ported statement by statement.
//!
//! Shapes are passed as `&Shape<R>` (read-only records) and transforms as `&XForm<R>`; manifolds
//! are mutated in place. All static scratch objects (`b2CollidePolyTempVec`, `b2Distance.s_p1s /
//! s_p2s / s_points / gPoint`, `b2TimeOfImpact.s_p1 / s_p2 / s_xf1 / s_xf2`) are locals: every use
//! fully overwrites them before reading and nothing outside keeps a reference.
//!
//! `b2Segment` and the `TestSegment` family are unused by the game and are not ported.
//! `b2Distance.g_GJK_Iterations` (debug counter) is not kept: nothing in the game reads it.
use hw_real::as3::{b2_max, math_abs, math_max};
use hw_real::Real;

/// AS3 Number literal as an `R` (bit-exact double).

use super::math::{b2_cross_vf, b2_dot, b2_mul_mv, b2_mul_x, b2_mul_xt, subtract_vv, Mat22, Sweep, Vec2, XForm};
use super::settings::{b2_toi_slop, max_value, min_value, B2_MAX_MANIFOLD_POINTS};
use super::shapes;
use super::state::{Manifold, Obb, Shape, E_CIRCLE_SHAPE, E_POLYGON_SHAPE};

/// b2Collision.b2_nullFeature
pub const B2_NULL_FEATURE: u32 = 255;

// ---- b2ContactID / Features (packed key) -------------------------------------------------------
// The AS3 Features setters write one byte of the owning b2ContactID._key each; with the key held
// as a plain u32 every setter becomes a masked update of that word.

/// `features.referenceEdge = v`: `_key = _key & 4294967040 | v & 0xFF`
#[inline(always)]
pub fn id_set_reference_edge(key: u32, v: i32) -> u32 {
    (key & 0xFFFF_FF00) | ((v as u32) & 0xFF)
}
/// `features.incidentEdge = v`: `_key = _key & 4294902015 | v << 8 & 0xFF00`
#[inline(always)]
pub fn id_set_incident_edge(key: u32, v: i32) -> u32 {
    (key & 0xFFFF_00FF) | (((v as u32).wrapping_shl(8)) & 0xFF00)
}
/// `features.incidentVertex = v`: `_key = _key & 4278255615 | v << 16 & 0xFF0000`
#[inline(always)]
pub fn id_set_incident_vertex(key: u32, v: i32) -> u32 {
    (key & 0xFF00_FFFF) | (((v as u32).wrapping_shl(16)) & 0x00FF_0000)
}
/// `features.flip = v`: `_key = _key & 0xFFFFFF | v << 24 & 4278190080`
#[inline(always)]
pub fn id_set_flip(key: u32, v: i32) -> u32 {
    (key & 0x00FF_FFFF) | (((v as u32).wrapping_shl(24)) & 0xFF00_0000)
}
#[inline(always)]
pub fn id_reference_edge(key: u32) -> i32 {
    (key & 0xFF) as i32
}
#[inline(always)]
pub fn id_incident_edge(key: u32) -> i32 {
    (((key & 0xFF00) >> 8) & 0xFF) as i32
}
#[inline(always)]
pub fn id_incident_vertex(key: u32) -> i32 {
    (((key & 0x00FF_0000) >> 16) & 0xFF) as i32
}
#[inline(always)]
pub fn id_flip(key: u32) -> i32 {
    (((key & 0xFF00_0000) >> 24) & 0xFF) as i32
}

/// ClipVertex: `v` plus the b2ContactID key.
#[derive(Clone, Copy, Debug)]
pub struct ClipVertex<R> {
    pub v: Vec2<R>,
    pub id: u32,
}
impl<R: Real> ClipVertex<R> {
    /// `new ClipVertex()`
    #[inline(always)]
    pub fn new() -> Self {
        ClipVertex { v: Vec2::zero(), id: 0 }
    }
}

// ---- b2AABB / b2OBB ----------------------------------------------------------------------------

/// b2AABB.IsValid() on a (lowerBound, upperBound) pair.
pub fn aabb_is_valid<R: Real>(lower: Vec2<R>, upper: Vec2<R>) -> bool {
    let dx = upper.x - lower.x;
    let dy = upper.y - lower.y;
    let valid = dx.ge(R::zero()) && dy.ge(R::zero());
    valid && lower.is_valid() && upper.is_valid()
}

/// `new b2OBB()`: R = new b2Mat22() (rotation of angle 0 via cos/sin), center/extents zero.
pub fn obb_new<R: Real>() -> Obb<R> {
    Obb { r: Mat22::from_angle(R::zero()), center: Vec2::zero(), extents: Vec2::zero() }
}

// ---- b2Collision -------------------------------------------------------------------------------

/// b2Collision.EdgeSeparation(poly1, xf1, edge1, poly2, xf2)
pub fn edge_separation<R: Real>(poly1: &Shape<R>, xf1: &XForm<R>, edge1: i32, poly2: &Shape<R>, xf2: &XForm<R>) -> R {
    let _count1 = poly1.vertex_count;
    let vert1s = &poly1.vertices;
    let normals1 = &poly1.normals;
    let count2 = poly2.vertex_count;
    let vert2s = &poly2.vertices;
    let mut t_mat = xf1.r;
    let mut t_vec = normals1[edge1 as usize];
    let normal1_world_x = t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y;
    let normal1_world_y = t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y;
    t_mat = xf2.r;
    let normal1_x = t_mat.col1.x * normal1_world_x + t_mat.col1.y * normal1_world_y;
    let normal1_y = t_mat.col2.x * normal1_world_x + t_mat.col2.y * normal1_world_y;
    let mut index: i32 = 0;
    let mut min_dot = max_value::<R>();
    let mut i: i32 = 0;
    while i < count2 {
        t_vec = vert2s[i as usize];
        let dot = t_vec.x * normal1_x + t_vec.y * normal1_y;
        if dot.lt(min_dot) {
            min_dot = dot;
            index = i;
        }
        i += 1;
    }
    t_vec = vert1s[edge1 as usize];
    t_mat = xf1.r;
    let v1x = xf1.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    let v1y = xf1.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    t_vec = vert2s[index as usize];
    t_mat = xf2.r;
    let mut v2x = xf2.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    let mut v2y = xf2.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    v2x = v2x - v1x;
    v2y = v2y - v1y;
    v2x * normal1_world_x + v2y * normal1_world_y
}

/// b2Collision.b2TestOverlap(a, b) on (lower, upper) pairs.
pub fn test_overlap<R: Real>(a_lower: Vec2<R>, a_upper: Vec2<R>, b_lower: Vec2<R>, b_upper: Vec2<R>) -> bool {
    let mut t1 = b_lower;
    let mut t2 = a_upper;
    let d1x = t1.x - t2.x;
    let d1y = t1.y - t2.y;
    t1 = a_lower;
    t2 = b_upper;
    let d2x = t1.x - t2.x;
    let d2y = t1.y - t2.y;
    if d1x.gt(R::zero()) || d1y.gt(R::zero()) {
        return false;
    }
    if d2x.gt(R::zero()) || d2y.gt(R::zero()) {
        return false;
    }
    true
}

/// b2Collision.FindIncidentEdge(c, poly1, xf1, edge1, poly2, xf2)
pub fn find_incident_edge<R: Real>(
    c: &mut [ClipVertex<R>; 2],
    poly1: &Shape<R>,
    xf1: &XForm<R>,
    edge1: i32,
    poly2: &Shape<R>,
    xf2: &XForm<R>,
) {
    let _count1 = poly1.vertex_count;
    let normals1 = &poly1.normals;
    let count2 = poly2.vertex_count;
    let vert2s = &poly2.vertices;
    let normals2 = &poly2.normals;
    let mut t_mat = xf1.r;
    let mut t_vec = normals1[edge1 as usize];
    let mut normal1_x = t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y;
    let mut normal1_y = t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y;
    t_mat = xf2.r;
    let t_x = t_mat.col1.x * normal1_x + t_mat.col1.y * normal1_y;
    normal1_y = t_mat.col2.x * normal1_x + t_mat.col2.y * normal1_y;
    normal1_x = t_x;
    let mut index: i32 = 0;
    let mut min_dot = max_value::<R>();
    let mut i: i32 = 0;
    while i < count2 {
        t_vec = normals2[i as usize];
        let dot = normal1_x * t_vec.x + normal1_y * t_vec.y;
        if dot.lt(min_dot) {
            min_dot = dot;
            index = i;
        }
        i += 1;
    }
    let i1 = index;
    let i2 = if i1 + 1 < count2 { i1 + 1 } else { 0 };
    let t_clip = &mut c[0];
    t_vec = vert2s[i1 as usize];
    t_mat = xf2.r;
    t_clip.v.x = xf2.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    t_clip.v.y = xf2.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    t_clip.id = id_set_reference_edge(t_clip.id, edge1);
    t_clip.id = id_set_incident_edge(t_clip.id, i1);
    t_clip.id = id_set_incident_vertex(t_clip.id, 0);
    let t_clip = &mut c[1];
    t_vec = vert2s[i2 as usize];
    t_mat = xf2.r;
    t_clip.v.x = xf2.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    t_clip.v.y = xf2.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    t_clip.id = id_set_reference_edge(t_clip.id, edge1);
    t_clip.id = id_set_incident_edge(t_clip.id, i2);
    t_clip.id = id_set_incident_vertex(t_clip.id, 1);
}

/// b2Collision.FindMaxSeparation(edgeIndex[], poly1, xf1, poly2, xf2) -> (separation, edgeIndex).
/// The AS3 writes `edgeIndex[0]` only on the non-positive-separation paths; the caller's local starts
/// at 0, so the returned edge is 0 whenever the AS3 leaves it unwritten.
pub fn find_max_separation<R: Real>(poly1: &Shape<R>, xf1: &XForm<R>, poly2: &Shape<R>, xf2: &XForm<R>) -> (R, i32) {
    let count1 = poly1.vertex_count;
    let normals1 = &poly1.normals;
    let mut t_mat = xf2.r;
    let mut t_vec = poly2.centroid;
    let mut dx = xf2.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    let mut dy = xf2.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    t_mat = xf1.r;
    t_vec = poly1.centroid;
    dx = dx - (xf1.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y));
    dy = dy - (xf1.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y));
    let d_local1_x = dx * xf1.r.col1.x + dy * xf1.r.col1.y;
    let d_local1_y = dx * xf1.r.col2.x + dy * xf1.r.col2.y;
    let mut edge: i32 = 0;
    let mut max_dot = -max_value::<R>();
    let mut i: i32 = 0;
    while i < count1 {
        t_vec = normals1[i as usize];
        let dot = t_vec.x * d_local1_x + t_vec.y * d_local1_y;
        if dot.gt(max_dot) {
            max_dot = dot;
            edge = i;
        }
        i += 1;
    }
    let mut s = edge_separation(poly1, xf1, edge, poly2, xf2);
    if s.gt(R::zero()) {
        return (s, 0);
    }
    let prev_edge = if edge - 1 >= 0 { edge - 1 } else { count1 - 1 };
    let s_prev = edge_separation(poly1, xf1, prev_edge, poly2, xf2);
    if s_prev.gt(R::zero()) {
        return (s_prev, 0);
    }
    let next_edge = if edge + 1 < count1 { edge + 1 } else { 0 };
    let s_next = edge_separation(poly1, xf1, next_edge, poly2, xf2);
    if s_next.gt(R::zero()) {
        return (s_next, 0);
    }
    let increment: i32;
    let mut best_edge: i32;
    let mut best_separation: R;
    if s_prev.gt(s) && s_prev.gt(s_next) {
        increment = -1;
        best_edge = prev_edge;
        best_separation = s_prev;
    } else {
        if s_next.le(s) {
            return (s, edge);
        }
        increment = 1;
        best_edge = next_edge;
        best_separation = s_next;
    }
    // while(true): with NaN separations this loop never terminates, exactly as the AS3 would.
    loop {
        if increment == -1 {
            edge = if best_edge - 1 >= 0 { best_edge - 1 } else { count1 - 1 };
        } else {
            edge = if best_edge + 1 < count1 { best_edge + 1 } else { 0 };
        }
        s = edge_separation(poly1, xf1, edge, poly2, xf2);
        if s.gt(R::zero()) {
            break;
        }
        if s.le(best_separation) {
            return (best_separation, best_edge);
        }
        best_edge = edge;
        best_separation = s;
    }
    (s, 0)
}

/// b2Collision.ClipSegmentToLine(vOut, vIn, normal, offset) -> count. AS3 copies object references
/// into `vOut`; nothing mutates the shared objects afterwards, so value copies are equivalent.
pub fn clip_segment_to_line<R: Real>(v_out: &mut [ClipVertex<R>; 2], v_in: &[ClipVertex<R>; 2], normal: Vec2<R>, offset: R) -> i32 {
    let mut num_out: i32 = 0;
    let v_in0 = v_in[0].v;
    let v_in1 = v_in[1].v;
    let distance0 = b2_dot(normal, v_in0) - offset;
    let distance1 = b2_dot(normal, v_in1) - offset;
    if distance0.le(R::zero()) {
        v_out[num_out as usize] = v_in[0];
        num_out += 1;
    }
    if distance1.le(R::zero()) {
        v_out[num_out as usize] = v_in[1];
        num_out += 1;
    }
    if (distance0 * distance1).lt(R::zero()) {
        // At most one of the two distances is <= 0 here, so num_out <= 1.
        debug_assert!(num_out < 2);
        let interp = distance0 / (distance0 - distance1);
        let t_vec = &mut v_out[num_out as usize].v;
        t_vec.x = v_in0.x + interp * (v_in1.x - v_in0.x);
        t_vec.y = v_in0.y + interp * (v_in1.y - v_in0.y);
        let t_clip = &mut v_out[num_out as usize];
        if distance0.gt(R::zero()) {
            t_clip.id = v_in[0].id;
        } else {
            t_clip.id = v_in[1].id;
        }
        num_out += 1;
    }
    num_out
}

/// b2Collision.b2CollidePolygons(manifold, polyA, xfA, polyB, xfB)
pub fn collide_polygons<R: Real>(manifold: &mut Manifold<R>, poly_a: &Shape<R>, xf_a: &XForm<R>, poly_b: &Shape<R>, xf_b: &XForm<R>) {
    manifold.point_count = 0;
    // var edgeA:int = 0; var edgeAO:Array = [edgeA]; ... edgeA = int(edgeAO[0])
    let (separation_a, edge_a) = find_max_separation(poly_a, xf_a, poly_b, xf_b);
    if separation_a.gt(R::zero()) {
        return;
    }
    let (separation_b, edge_b) = find_max_separation(poly_b, xf_b, poly_a, xf_a);
    if separation_b.gt(R::zero()) {
        return;
    }
    let poly1: &Shape<R>;
    let poly2: &Shape<R>;
    // var xf1:b2XForm = new b2XForm(); var xf2:b2XForm = new b2XForm(); (fully overwritten by Set)
    let xf1: XForm<R>;
    let xf2: XForm<R>;
    let edge1: i32;
    let flip: u32;
    let k_relative_tol: R = hw_real::rl!(R; 0.98);
    let k_absolute_tol: R = hw_real::rl!(R; 0.001);
    if separation_b.gt(k_relative_tol * separation_a + k_absolute_tol) {
        poly1 = poly_b;
        poly2 = poly_a;
        xf1 = *xf_b;
        xf2 = *xf_a;
        edge1 = edge_b;
        flip = 1;
    } else {
        poly1 = poly_a;
        poly2 = poly_b;
        xf1 = *xf_a;
        xf2 = *xf_b;
        edge1 = edge_a;
        flip = 0;
    }
    let mut incident_edge = [ClipVertex::new(), ClipVertex::new()];
    find_incident_edge(&mut incident_edge, poly1, &xf1, edge1, poly2, &xf2);
    let count1 = poly1.vertex_count;
    let vert1s = &poly1.vertices;
    let mut t_vec = vert1s[edge1 as usize];
    let mut v11 = t_vec; // .Copy()
    let mut v12: Vec2<R>;
    if edge1 + 1 < count1 {
        t_vec = vert1s[(edge1 + 1) as usize];
        v12 = t_vec;
    } else {
        t_vec = vert1s[0];
        v12 = t_vec;
    }
    let _dv = subtract_vv(v12, v11); // computed and unused in AS3
    let mut side_normal = b2_mul_mv(xf1.r, subtract_vv(v12, v11));
    side_normal.normalize();
    let front_normal = b2_cross_vf(side_normal, R::one());
    v11 = b2_mul_x(xf1, v11);
    v12 = b2_mul_x(xf1, v12);
    let front_offset = b2_dot(front_normal, v11);
    let side_offset1 = -b2_dot(side_normal, v11);
    let side_offset2 = b2_dot(side_normal, v12);
    let mut clip_points1 = [ClipVertex::new(), ClipVertex::new()];
    let mut clip_points2 = [ClipVertex::new(), ClipVertex::new()];
    let mut np = clip_segment_to_line(&mut clip_points1, &incident_edge, side_normal.negative(), side_offset1);
    if np < 2 {
        return;
    }
    np = clip_segment_to_line(&mut clip_points2, &clip_points1, side_normal, side_offset2);
    if np < 2 {
        return;
    }
    manifold.normal = if flip != 0 { front_normal.negative() } else { front_normal };
    let mut point_count: i32 = 0;
    let mut i: i32 = 0;
    while i < B2_MAX_MANIFOLD_POINTS as i32 {
        let cv = clip_points2[i as usize];
        let separation = b2_dot(front_normal, cv.v) - front_offset;
        if separation.le(R::zero()) {
            let cp = &mut manifold.points[point_count as usize];
            cp.separation = separation;
            cp.local_point1 = b2_mul_xt(*xf_a, cv.v);
            cp.local_point2 = b2_mul_xt(*xf_b, cv.v);
            // cp.id.key = cv.id._key; cp.id.features.flip = flip;
            cp.id = cv.id;
            cp.id = id_set_flip(cp.id, flip as i32);
            point_count += 1;
        }
        i += 1;
    }
    manifold.point_count = point_count;
}

/// b2Collision.b2CollideCircles(manifold, circle1, xf1, circle2, xf2)
pub fn collide_circles<R: Real>(manifold: &mut Manifold<R>, circle1: &Shape<R>, xf1: &XForm<R>, circle2: &Shape<R>, xf2: &XForm<R>) {
    manifold.point_count = 0;
    let mut t_mat = xf1.r;
    let mut t_vec = circle1.local_position;
    let mut p1x = xf1.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    let mut p1y = xf1.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    t_mat = xf2.r;
    t_vec = circle2.local_position;
    let mut p2x = xf2.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    let mut p2y = xf2.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    let dx = p2x - p1x;
    let dy = p2y - p1y;
    let dist_sqr = dx * dx + dy * dy;
    let r1 = circle1.radius;
    let r2 = circle2.radius;
    let radius_sum = r1 + r2;
    if dist_sqr.gt(radius_sum * radius_sum) {
        return;
    }
    let separation: R;
    if dist_sqr.lt(min_value::<R>()) {
        separation = -radius_sum;
        manifold.normal.set(R::zero(), R::one());
    } else {
        let dist = dist_sqr.sqrt();
        separation = dist - radius_sum;
        let a = R::one() / dist;
        manifold.normal.x = a * dx;
        manifold.normal.y = a * dy;
    }
    manifold.point_count = 1;
    let normal = manifold.normal;
    let t_point = &mut manifold.points[0];
    t_point.id = 0; // id.key = 0
    t_point.separation = separation;
    p1x = p1x + r1 * normal.x;
    p1y = p1y + r1 * normal.y;
    p2x = p2x - r2 * normal.x;
    p2y = p2y - r2 * normal.y;
    let px = hw_real::rl!(R; 0.5) * (p1x + p2x);
    let py = hw_real::rl!(R; 0.5) * (p1y + p2y);
    let mut t_x = px - xf1.position.x;
    let mut t_y = py - xf1.position.y;
    t_point.local_point1.x = t_x * xf1.r.col1.x + t_y * xf1.r.col1.y;
    t_point.local_point1.y = t_x * xf1.r.col2.x + t_y * xf1.r.col2.y;
    t_x = px - xf2.position.x;
    t_y = py - xf2.position.y;
    t_point.local_point2.x = t_x * xf2.r.col1.x + t_y * xf2.r.col1.y;
    t_point.local_point2.y = t_x * xf2.r.col2.x + t_y * xf2.r.col2.y;
}

/// b2Collision.b2CollidePolygonAndCircle(manifold, polygon, xf1, circle, xf2)
pub fn collide_polygon_and_circle<R: Real>(manifold: &mut Manifold<R>, polygon: &Shape<R>, xf1: &XForm<R>, circle: &Shape<R>, xf2: &XForm<R>) {
    manifold.point_count = 0;
    let mut t_mat = xf2.r;
    let mut t_vec = circle.local_position;
    let cx = xf2.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    let cy = xf2.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    let mut dx = cx - xf1.position.x;
    let mut dy = cy - xf1.position.y;
    t_mat = xf1.r;
    let c_local_x = dx * t_mat.col1.x + dy * t_mat.col1.y;
    let c_local_y = dx * t_mat.col2.x + dy * t_mat.col2.y;
    let mut normal_index: i32 = 0;
    let mut separation = -max_value::<R>();
    let radius = circle.radius;
    let vertex_count = polygon.vertex_count;
    let vertices = &polygon.vertices;
    let normals = &polygon.normals;
    let mut i: i32 = 0;
    while i < vertex_count {
        t_vec = vertices[i as usize];
        dx = c_local_x - t_vec.x;
        dy = c_local_y - t_vec.y;
        t_vec = normals[i as usize];
        let s = t_vec.x * dx + t_vec.y * dy;
        if s.gt(radius) {
            return;
        }
        if s.gt(separation) {
            separation = s;
            normal_index = i;
        }
        i += 1;
    }
    if separation.lt(min_value::<R>()) {
        manifold.point_count = 1;
        t_vec = normals[normal_index as usize];
        t_mat = xf1.r;
        manifold.normal.x = t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y;
        manifold.normal.y = t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y;
        let normal = manifold.normal;
        let t_point = &mut manifold.points[0];
        t_point.id = id_set_incident_edge(t_point.id, normal_index);
        t_point.id = id_set_incident_vertex(t_point.id, B2_NULL_FEATURE as i32);
        t_point.id = id_set_reference_edge(t_point.id, 0);
        t_point.id = id_set_flip(t_point.id, 0);
        let position_x = cx - radius * normal.x;
        let position_y = cy - radius * normal.y;
        dx = position_x - xf1.position.x;
        dy = position_y - xf1.position.y;
        t_mat = xf1.r;
        t_point.local_point1.x = dx * t_mat.col1.x + dy * t_mat.col1.y;
        t_point.local_point1.y = dx * t_mat.col2.x + dy * t_mat.col2.y;
        dx = position_x - xf2.position.x;
        dy = position_y - xf2.position.y;
        t_mat = xf2.r;
        t_point.local_point2.x = dx * t_mat.col1.x + dy * t_mat.col1.y;
        t_point.local_point2.y = dx * t_mat.col2.x + dy * t_mat.col2.y;
        t_point.separation = separation - radius;
        return;
    }
    let vert_index1 = normal_index;
    let vert_index2 = if vert_index1 + 1 < vertex_count { vert_index1 + 1 } else { 0 };
    t_vec = vertices[vert_index1 as usize];
    let t_vec2 = vertices[vert_index2 as usize];
    let mut ex = t_vec2.x - t_vec.x;
    let mut ey = t_vec2.y - t_vec.y;
    let length = (ex * ex + ey * ey).sqrt();
    ex = ex / length;
    ey = ey / length;
    dx = c_local_x - t_vec.x;
    dy = c_local_y - t_vec.y;
    let u = dx * ex + dy * ey;
    let t_point = &mut manifold.points[0];
    let px: R;
    let py: R;
    if u.le(R::zero()) {
        px = t_vec.x;
        py = t_vec.y;
        t_point.id = id_set_incident_edge(t_point.id, B2_NULL_FEATURE as i32);
        t_point.id = id_set_incident_vertex(t_point.id, vert_index1);
    } else if u.ge(length) {
        px = t_vec2.x;
        py = t_vec2.y;
        t_point.id = id_set_incident_edge(t_point.id, B2_NULL_FEATURE as i32);
        t_point.id = id_set_incident_vertex(t_point.id, vert_index2);
    } else {
        px = ex * u + t_vec.x;
        py = ey * u + t_vec.y;
        t_point.id = id_set_incident_edge(t_point.id, normal_index);
        t_point.id = id_set_incident_vertex(t_point.id, 0);
    }
    dx = c_local_x - px;
    dy = c_local_y - py;
    let dist = (dx * dx + dy * dy).sqrt();
    dx = dx / dist;
    dy = dy / dist;
    if dist.gt(radius) {
        // Early exit leaves incidentEdge/incidentVertex already written into points[0].id
        // (referenceEdge/flip untouched) with pointCount == 0, as in AS3.
        return;
    }
    manifold.point_count = 1;
    t_mat = xf1.r;
    manifold.normal.x = t_mat.col1.x * dx + t_mat.col2.x * dy;
    manifold.normal.y = t_mat.col1.y * dx + t_mat.col2.y * dy;
    let position_x = cx - radius * manifold.normal.x;
    let position_y = cy - radius * manifold.normal.y;
    dx = position_x - xf1.position.x;
    dy = position_y - xf1.position.y;
    t_mat = xf1.r;
    t_point.local_point1.x = dx * t_mat.col1.x + dy * t_mat.col1.y;
    t_point.local_point1.y = dx * t_mat.col2.x + dy * t_mat.col2.y;
    dx = position_x - xf2.position.x;
    dy = position_y - xf2.position.y;
    t_mat = xf2.r;
    t_point.local_point2.x = dx * t_mat.col1.x + dy * t_mat.col1.y;
    t_point.local_point2.y = dx * t_mat.col2.x + dy * t_mat.col2.y;
    t_point.separation = dist - radius;
    t_point.id = id_set_reference_edge(t_point.id, 0);
    t_point.id = id_set_flip(t_point.id, 0);
}

// ---- b2Distance --------------------------------------------------------------------------------

/// The duck-typed `*` parameters of b2Distance.DistanceGeneric: a b2PolygonShape or a b2Point.
pub trait GjkShape<R: Real> {
    fn get_first_vertex(&self, xf: &XForm<R>) -> Vec2<R>;
    fn support(&self, xf: &XForm<R>, dx: R, dy: R) -> Vec2<R>;
}

impl<R: Real> GjkShape<R> for Shape<R> {
    #[inline(always)]
    fn get_first_vertex(&self, xf: &XForm<R>) -> Vec2<R> {
        shapes::get_first_vertex(self, xf)
    }
    #[inline(always)]
    fn support(&self, xf: &XForm<R>, dx: R, dy: R) -> Vec2<R> {
        shapes::support(self, xf, dx, dy)
    }
}

/// b2Point (the `gPoint` static of b2Distance): a single point acting as a shape.
#[derive(Clone, Copy, Debug)]
pub struct Point<R> {
    pub p: Vec2<R>,
}
impl<R: Real> GjkShape<R> for Point<R> {
    #[inline(always)]
    fn get_first_vertex(&self, _xf: &XForm<R>) -> Vec2<R> {
        self.p
    }
    #[inline(always)]
    fn support(&self, _xf: &XForm<R>, _dx: R, _dy: R) -> Vec2<R> {
        self.p
    }
}

/// b2Distance.InPoints(w, points, pointCount). Dead code in Box2D 2.0.2 (never called); ported for
/// completeness. Uses Math.abs / Math.max (not the b2Math ternaries).
pub fn in_points<R: Real>(w: Vec2<R>, points: &[Vec2<R>; 3], point_count: i32) -> bool {
    let k_tolerance = R::from_i32(100) * min_value::<R>();
    let mut i: i32 = 0;
    while i < point_count {
        let v = points[i as usize];
        let dx = math_abs(w.x - v.x);
        let dy = math_abs(w.y - v.y);
        let mx = math_max(math_abs(w.x), math_abs(v.x));
        let my = math_max(math_abs(w.y), math_abs(v.y));
        if dx.lt(k_tolerance * (mx + R::one())) && dy.lt(k_tolerance * (my + R::one())) {
            return true;
        }
        i += 1;
    }
    false
}

/// b2Distance.ProcessTwo(x1, x2, p1s, p2s, points)
pub fn process_two<R: Real>(
    x1: &mut Vec2<R>,
    x2: &mut Vec2<R>,
    p1s: &mut [Vec2<R>; 3],
    p2s: &mut [Vec2<R>; 3],
    points: &mut [Vec2<R>; 3],
) -> i32 {
    let r_x = -points[1].x;
    let r_y = -points[1].y;
    let mut dx = points[0].x - points[1].x;
    let mut dy = points[0].y - points[1].y;
    let length = (dx * dx + dy * dy).sqrt();
    dx = dx / length;
    dy = dy / length;
    let mut lambda = r_x * dx + r_y * dy;
    if lambda.le(R::zero()) || length.lt(min_value::<R>()) {
        x1.set_v(p1s[1]);
        x2.set_v(p2s[1]);
        let t = p1s[1];
        p1s[0].set_v(t);
        let t = p2s[1];
        p2s[0].set_v(t);
        let t = points[1];
        points[0].set_v(t);
        return 1;
    }
    lambda = lambda / length;
    x1.x = p1s[1].x + lambda * (p1s[0].x - p1s[1].x);
    x1.y = p1s[1].y + lambda * (p1s[0].y - p1s[1].y);
    x2.x = p2s[1].x + lambda * (p2s[0].x - p2s[1].x);
    x2.y = p2s[1].y + lambda * (p2s[0].y - p2s[1].y);
    2
}

/// b2Distance.ProcessThree(x1, x2, p1s, p2s, points)
pub fn process_three<R: Real>(
    x1: &mut Vec2<R>,
    x2: &mut Vec2<R>,
    p1s: &mut [Vec2<R>; 3],
    p2s: &mut [Vec2<R>; 3],
    points: &mut [Vec2<R>; 3],
) -> i32 {
    let p1x = points[0].x;
    let p1y = points[0].y;
    let p2x = points[1].x;
    let p2y = points[1].y;
    let p3x = points[2].x;
    let p3y = points[2].y;
    let e12x = p2x - p1x;
    let e12y = p2y - p1y;
    let e13x = p3x - p1x;
    let e13y = p3y - p1y;
    let e23x = p3x - p2x;
    let e23y = p3y - p2y;
    let d12_1 = -(p1x * e12x + p1y * e12y);
    let d12_2 = p2x * e12x + p2y * e12y;
    let d13_1 = -(p1x * e13x + p1y * e13y);
    let d13_2 = p3x * e13x + p3y * e13y;
    let d23_1 = -(p2x * e23x + p2y * e23y);
    let d23_2 = p3x * e23x + p3y * e23y;
    let _ = (d12_1, d12_2); // computed in AS3, never read
    if d13_2.le(R::zero()) && d23_2.le(R::zero()) {
        x1.set_v(p1s[2]);
        x2.set_v(p2s[2]);
        let t = p1s[2];
        p1s[0].set_v(t);
        let t = p2s[2];
        p2s[0].set_v(t);
        let t = points[2];
        points[0].set_v(t);
        return 1;
    }
    let n = e12x * e13y - e12y * e13x;
    let d123_3 = n * (p1x * p2y - p1y * p2x);
    let d123_1 = n * (p2x * p3y - p2y * p3x);
    if d123_1.le(R::zero()) && d23_1.ge(R::zero()) && d23_2.ge(R::zero()) && (d23_1 + d23_2).gt(R::zero()) {
        let lambda = d23_1 / (d23_1 + d23_2);
        x1.x = p1s[1].x + lambda * (p1s[2].x - p1s[1].x);
        x1.y = p1s[1].y + lambda * (p1s[2].y - p1s[1].y);
        x2.x = p2s[1].x + lambda * (p2s[2].x - p2s[1].x);
        x2.y = p2s[1].y + lambda * (p2s[2].y - p2s[1].y);
        let t = p1s[2];
        p1s[0].set_v(t);
        let t = p2s[2];
        p2s[0].set_v(t);
        let t = points[2];
        points[0].set_v(t);
        return 2;
    }
    let d123_2 = n * (p3x * p1y - p3y * p1x);
    if d123_2.le(R::zero()) && d13_1.ge(R::zero()) && d13_2.ge(R::zero()) && (d13_1 + d13_2).gt(R::zero()) {
        let lambda = d13_1 / (d13_1 + d13_2);
        x1.x = p1s[0].x + lambda * (p1s[2].x - p1s[0].x);
        x1.y = p1s[0].y + lambda * (p1s[2].y - p1s[0].y);
        x2.x = p2s[0].x + lambda * (p2s[2].x - p2s[0].x);
        x2.y = p2s[0].y + lambda * (p2s[2].y - p2s[0].y);
        let t = p1s[2];
        p1s[1].set_v(t);
        let t = p2s[2];
        p2s[1].set_v(t);
        let t = points[2];
        points[1].set_v(t);
        return 2;
    }
    let mut denom = d123_1 + d123_2 + d123_3;
    denom = R::one() / denom;
    let lambda1 = d123_1 * denom;
    let lambda2 = d123_2 * denom;
    let lambda3 = R::one() - lambda1 - lambda2;
    x1.x = lambda1 * p1s[0].x + lambda2 * p1s[1].x + lambda3 * p1s[2].x;
    x1.y = lambda1 * p1s[0].y + lambda2 * p1s[1].y + lambda3 * p1s[2].y;
    x2.x = lambda1 * p2s[0].x + lambda2 * p2s[1].x + lambda3 * p2s[2].x;
    x2.y = lambda1 * p2s[0].y + lambda2 * p2s[1].y + lambda3 * p2s[2].y;
    3
}

/// b2Distance.DistanceGeneric(x1, x2, shape1, xf1, shape2, xf2). The shared static simplex arrays
/// `s_p1s / s_p2s / s_points` are locals (fully rewritten before every read).
pub fn distance_generic<R: Real, A: GjkShape<R>, B: GjkShape<R>>(
    x1: &mut Vec2<R>,
    x2: &mut Vec2<R>,
    shape1: &A,
    xf1: &XForm<R>,
    shape2: &B,
    xf2: &XForm<R>,
) -> R {
    let mut p1s = [Vec2::zero(); 3];
    let mut p2s = [Vec2::zero(); 3];
    let mut points = [Vec2::zero(); 3];
    let mut point_count: i32 = 0;
    x1.set_v(shape1.get_first_vertex(xf1));
    x2.set_v(shape2.get_first_vertex(xf2));
    let mut v_sqr = R::zero();
    let max_iterations: i32 = 20;
    let mut iter: i32 = 0;
    while iter < max_iterations {
        let vx = x2.x - x1.x;
        let vy = x2.y - x1.y;
        let w1 = shape1.support(xf1, vx, vy);
        let w2 = shape2.support(xf2, -vx, -vy);
        v_sqr = vx * vx + vy * vy;
        let wx = w2.x - w1.x;
        let wy = w2.y - w1.y;
        let _vw = vx * wx + vy * wy; // computed, then the dot is recomputed inline in the test (AS3)
        if (v_sqr - (vx * wx + vy * wy)).le(hw_real::rl!(R; 0.01) * v_sqr) {
            if point_count == 0 {
                x1.set_v(w1);
                x2.set_v(w2);
            }
            // g_GJK_Iterations = iter
            return v_sqr.sqrt();
        }
        match point_count {
            0 => {
                p1s[0].set_v(w1);
                p2s[0].set_v(w2);
                points[0].x = wx;
                points[0].y = wy;
                x1.set_v(p1s[0]);
                x2.set_v(p2s[0]);
                point_count += 1;
            }
            1 => {
                p1s[1].set_v(w1);
                p2s[1].set_v(w2);
                points[1].x = wx;
                points[1].y = wy;
                point_count = process_two(x1, x2, &mut p1s, &mut p2s, &mut points);
            }
            2 => {
                p1s[2].set_v(w1);
                p2s[2].set_v(w2);
                points[2].x = wx;
                points[2].y = wy;
                point_count = process_three(x1, x2, &mut p1s, &mut p2s, &mut points);
            }
            _ => {}
        }
        if point_count == 3 {
            // g_GJK_Iterations = iter
            return R::zero();
        }
        let mut max_sqr = -max_value::<R>();
        let mut i: i32 = 0;
        while i < point_count {
            let t_vec = points[i as usize];
            max_sqr = b2_max(max_sqr, t_vec.x * t_vec.x + t_vec.y * t_vec.y);
            i += 1;
        }
        if point_count == 3 || v_sqr.le(R::from_i32(100) * min_value::<R>() * max_sqr) {
            // g_GJK_Iterations = iter
            let vx = x2.x - x1.x;
            let vy = x2.y - x1.y;
            v_sqr = vx * vx + vy * vy;
            return v_sqr.sqrt();
        }
        iter += 1;
    }
    // g_GJK_Iterations = maxIterations
    v_sqr.sqrt()
}

/// b2Distance.DistanceCC(x1, x2, circle1, xf1, circle2, xf2)
pub fn distance_cc<R: Real>(x1: &mut Vec2<R>, x2: &mut Vec2<R>, circle1: &Shape<R>, xf1: &XForm<R>, circle2: &Shape<R>, xf2: &XForm<R>) -> R {
    let mut t_mat = xf1.r;
    let mut t_vec = circle1.local_position;
    let p1x = xf1.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    let p1y = xf1.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    t_mat = xf2.r;
    t_vec = circle2.local_position;
    let p2x = xf2.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    let p2y = xf2.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    let mut dx = p2x - p1x;
    let mut dy = p2y - p1y;
    let dist_sqr = dx * dx + dy * dy;
    let r1 = circle1.radius - b2_toi_slop::<R>();
    let r2 = circle2.radius - b2_toi_slop::<R>();
    let r = r1 + r2;
    if dist_sqr.gt(r * r) {
        let dist = (dx * dx + dy * dy).sqrt();
        dx = dx / dist;
        dy = dy / dist;
        let separation = dist - r;
        x1.x = p1x + r1 * dx;
        x1.y = p1y + r1 * dy;
        x2.x = p2x - r2 * dx;
        x2.y = p2y - r2 * dy;
        return separation;
    }
    if dist_sqr.gt(min_value::<R>() * min_value::<R>()) {
        let dist = (dx * dx + dy * dy).sqrt();
        dx = dx / dist;
        dy = dy / dist;
        x1.x = p1x + r1 * dx;
        x1.y = p1y + r1 * dy;
        x2.x = x1.x;
        x2.y = x1.y;
        return R::zero();
    }
    x1.x = p1x;
    x1.y = p1y;
    x2.x = x1.x;
    x2.y = x1.y;
    R::zero()
}

/// b2Distance.DistancePC(x1, x2, polygon, xf1, circle, xf2). `gPoint` is a local.
pub fn distance_pc<R: Real>(x1: &mut Vec2<R>, x2: &mut Vec2<R>, polygon: &Shape<R>, xf1: &XForm<R>, circle: &Shape<R>, xf2: &XForm<R>) -> R {
    let mut point = Point { p: Vec2::zero() };
    let t_vec = circle.local_position;
    let t_mat = xf2.r;
    point.p.x = xf2.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    point.p.y = xf2.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    // b2Math.b2XForm_identity: position (0,0), R = exact identity columns (1,0),(0,1).
    let identity = XForm::identity();
    let mut distance = distance_generic(x1, x2, polygon, xf1, &point, &identity);
    let r = circle.radius - b2_toi_slop::<R>();
    if distance.gt(r) {
        distance = distance - r;
        let mut dx = x2.x - x1.x;
        let mut dy = x2.y - x1.y;
        let d = (dx * dx + dy * dy).sqrt();
        dx = dx / d;
        dy = dy / d;
        x2.x = x2.x - r * dx;
        x2.y = x2.y - r * dy;
    } else {
        distance = R::zero();
        x2.x = x1.x;
        x2.y = x1.y;
    }
    distance
}

/// b2Distance.Distance(x1, x2, shape1, xf1, shape2, xf2)
pub fn distance<R: Real>(x1: &mut Vec2<R>, x2: &mut Vec2<R>, shape1: &Shape<R>, xf1: &XForm<R>, shape2: &Shape<R>, xf2: &XForm<R>) -> R {
    let type1 = shape1.type_;
    let type2 = shape2.type_;
    if type1 == E_CIRCLE_SHAPE && type2 == E_CIRCLE_SHAPE {
        return distance_cc(x1, x2, shape1, xf1, shape2, xf2);
    }
    if type1 == E_POLYGON_SHAPE && type2 == E_CIRCLE_SHAPE {
        return distance_pc(x1, x2, shape1, xf1, shape2, xf2);
    }
    if type1 == E_CIRCLE_SHAPE && type2 == E_POLYGON_SHAPE {
        return distance_pc(x2, x1, shape2, xf2, shape1, xf1);
    }
    if type1 == E_POLYGON_SHAPE && type2 == E_POLYGON_SHAPE {
        return distance_generic(x1, x2, shape1, xf1, shape2, xf2);
    }
    R::zero()
}

// ---- b2TimeOfImpact ----------------------------------------------------------------------------

/// b2TimeOfImpact.TimeOfImpact(shape1, sweep1, shape2, sweep2). The statics `s_p1 / s_p2 / s_xf1 /
/// s_xf2` are locals (GetXForm and Distance overwrite them before any read).
pub fn time_of_impact<R: Real>(shape1: &Shape<R>, sweep1: &Sweep<R>, shape2: &Shape<R>, sweep2: &Sweep<R>) -> R {
    let r1 = shape1.sweep_radius;
    let r2 = shape2.sweep_radius;
    let t0 = sweep1.t0;
    let v1x = sweep1.c.x - sweep1.c0.x;
    let v1y = sweep1.c.y - sweep1.c0.y;
    let v2x = sweep2.c.x - sweep2.c0.x;
    let v2y = sweep2.c.y - sweep2.c0.y;
    let omega1 = sweep1.a - sweep1.a0;
    let omega2 = sweep2.a - sweep2.a0;
    let mut alpha = R::zero();
    let mut p1 = Vec2::zero();
    let mut p2 = Vec2::zero();
    let k_max_iterations: i32 = 20;
    let mut iter: i32 = 0;
    let mut normal_x: R;
    let mut normal_y: R;
    let mut distance_: R;
    let mut target_distance = R::zero();
    loop {
        let t = (R::one() - alpha) * t0 + alpha;
        let mut xf1 = XForm::new_default();
        let mut xf2 = XForm::new_default();
        sweep1.get_xform(&mut xf1, t);
        sweep2.get_xform(&mut xf2, t);
        distance_ = distance(&mut p1, &mut p2, shape1, &xf1, shape2, &xf2);
        if iter == 0 {
            if distance_.gt(R::from_i32(2) * b2_toi_slop::<R>()) {
                target_distance = hw_real::rl!(R; 1.5) * b2_toi_slop::<R>();
            } else {
                let a = hw_real::rl!(R; 0.05) * b2_toi_slop::<R>();
                let b = distance_ - hw_real::rl!(R; 0.5) * b2_toi_slop::<R>();
                target_distance = if a.gt(b) { a } else { b };
            }
        }
        if (distance_ - target_distance).lt(hw_real::rl!(R; 0.05) * b2_toi_slop::<R>()) || iter == k_max_iterations {
            break;
        }
        normal_x = p2.x - p1.x;
        normal_y = p2.y - p1.y;
        let n_length = (normal_x * normal_x + normal_y * normal_y).sqrt();
        normal_x = normal_x / n_length;
        normal_y = normal_y / n_length;
        let approach_velocity_bound = normal_x * (v1x - v2x)
            + normal_y * (v1y - v2y)
            + (if omega1.lt(R::zero()) { -omega1 } else { omega1 }) * r1
            + (if omega2.lt(R::zero()) { -omega2 } else { omega2 }) * r2;
        if approach_velocity_bound.eq_num(R::zero()) {
            alpha = R::one();
            break;
        }
        let d_alpha = (distance_ - target_distance) / approach_velocity_bound;
        let new_alpha = alpha + d_alpha;
        if new_alpha.lt(R::zero()) || R::one().lt(new_alpha) {
            alpha = R::one();
            break;
        }
        // (1 + 100 * Number.MIN_VALUE) evaluates to exactly 1.0; written literally on purpose.
        if new_alpha.lt((R::one() + R::from_i32(100) * min_value::<R>()) * alpha) {
            break;
        }
        alpha = new_alpha;
        iter += 1;
    }
    alpha
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::b2::state::{ShapeDef, WorldState};
    use crate::ctx::Ctx;
    use crate::b2::geometry_fixture as golden;

    /// `Ctx` is several MB (arena arrays); build it on a thread with a large stack.
    fn with_ctx(f: fn(&mut Ctx<f64>)) {
        std::thread::Builder::new()
            .stack_size(256 << 20)
            .spawn(move || {
                let mut c = Box::new(Ctx { world: WorldState::blank(), scratch: crate::b2::island::StepScratch::blank(), listener_mode: 0, level: crate::game::specials::LevelState::blank(), character: crate::game::actors::Actors::blank() });
                f(&mut c);
            })
            .unwrap()
            .join()
            .unwrap();
    }

    fn circle(c: &mut Ctx<f64>, r: f64) -> Shape<f64> {
        let mut d = ShapeDef::<f64>::circle();
        d.radius = r;
        let h = shapes::create(c, &d);
        *c.world.shape(h)
    }

    fn boxp(c: &mut Ctx<f64>, hx: f64, hy: f64) -> Shape<f64> {
        let mut d = ShapeDef::<f64>::polygon();
        d.set_as_box(hx, hy);
        let h = shapes::create(c, &d);
        *c.world.shape(h)
    }

    fn at(x: f64, y: f64) -> XForm<f64> {
        let mut xf = XForm::<f64>::identity();
        xf.position = Vec2::new(x, y);
        xf
    }

    #[test]
    fn contact_id_setters() {
        let mut k = 0u32;
        k = id_set_reference_edge(k, 3);
        k = id_set_incident_edge(k, 5);
        k = id_set_incident_vertex(k, B2_NULL_FEATURE as i32);
        k = id_set_flip(k, 1);
        assert_eq!(k, 0x01FF_0503);
        assert_eq!(id_reference_edge(k), 3);
        assert_eq!(id_incident_edge(k), 5);
        assert_eq!(id_incident_vertex(k), 255);
        assert_eq!(id_flip(k), 1);
        assert_eq!(id_set_flip(k, 0), 0x00FF_0503);
    }

    #[test]
    fn circle_circle_manifold() {
        with_ctx(|c| {
            let c1 = circle(c, 1.0);
            let c2 = circle(c, 1.0);
            let mut m = Manifold::<f64>::new();
            collide_circles(&mut m, &c1, &at(0.0, 0.0), &c2, &at(1.5, 0.0));
            golden::manifold("cc.overlap", &m);
            // apart
            collide_circles(&mut m, &c1, &at(0.0, 0.0), &c2, &at(3.0, 0.0));
            golden::manifold("cc.apart", &m);
            // coincident centres -> normal (0,1), separation -(r1+r2)
            collide_circles(&mut m, &c1, &at(0.0, 0.0), &c2, &at(0.0, 0.0));
            golden::manifold("cc.coincident", &m);
        });
    }

    #[test]
    fn polygon_polygon_manifold() {
        with_ctx(|c| {
            let a = boxp(c, 1.0, 1.0);
            let b = boxp(c, 1.0, 1.0);
            let mut m = Manifold::<f64>::new();
            collide_polygons(&mut m, &a, &at(0.0, 0.0), &b, &at(1.5, 0.0));
            golden::manifold("pp.overlap", &m);
            collide_polygons(&mut m, &a, &at(0.0, 0.0), &b, &at(2.5, 0.0));
            golden::manifold("pp.apart", &m);
        });
    }

    #[test]
    fn polygon_circle_manifold() {
        with_ctx(|c| {
            let a = boxp(c, 1.0, 1.0);
            let b = circle(c, 0.5);
            let mut m = Manifold::<f64>::new();
            // circle centre 1.25 right of the box: edge 1 (normal +x), separation 0.25 - 0.5
            collide_polygon_and_circle(&mut m, &a, &at(0.0, 0.0), &b, &at(1.25, 0.0));
            golden::manifold("pc.overlap", &m);
            collide_polygon_and_circle(&mut m, &a, &at(0.0, 0.0), &b, &at(2.0, 0.0));
            golden::manifold("pc.apart", &m);
        });
    }

    #[test]
    fn overlap_and_distance() {
        assert!(test_overlap(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(0.5, 0.5), Vec2::new(2.0, 2.0)));
        assert!(!test_overlap(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(1.5, 0.5), Vec2::new(2.0, 2.0)));
        assert!(aabb_is_valid(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)));
        assert!(!aabb_is_valid(Vec2::new(0.0, 0.0), Vec2::new(-1.0, 1.0)));
        with_ctx(|c| {
            let c1 = circle(c, 1.0);
            let c2 = circle(c, 1.0);
            let mut x1 = Vec2::zero();
            let mut x2 = Vec2::zero();
            let d = distance(&mut x1, &mut x2, &c1, &at(0.0, 0.0), &c2, &at(5.0, 0.0));
            golden::number("distance.cc.distance", d);
            golden::vector("distance.cc.x1", x1);
            golden::vector("distance.cc.x2", x2);
            // polygon vs polygon GJK
            let a = boxp(c, 1.0, 1.0);
            let b = boxp(c, 1.0, 1.0);
            let d = distance(&mut x1, &mut x2, &a, &at(0.0, 0.0), &b, &at(5.0, 0.0));
            golden::number("distance.pp.distance", d);
            golden::vector("distance.pp.x1", x1);
            golden::vector("distance.pp.x2", x2);
            // polygon vs circle
            let d = distance(&mut x1, &mut x2, &a, &at(0.0, 0.0), &c2, &at(5.0, 0.0));
            golden::number("distance.pc.distance", d);
            golden::vector("distance.pc.x1", x1);
            golden::vector("distance.pc.x2", x2);
        });
    }

    #[test]
    fn toi_moving_circles() {
        with_ctx(|c| {
            let c1 = circle(c, 1.0);
            let c2 = circle(c, 1.0);
            let mut s1 = Sweep::<f64>::new_default();
            s1.c0 = Vec2::new(0.0, 0.0);
            s1.c = Vec2::new(0.0, 0.0);
            s1.a0 = 0.0;
            s1.a = 0.0;
            s1.t0 = 0.0;
            let mut s2 = s1;
            s2.c0 = Vec2::new(10.0, 0.0);
            s2.c = Vec2::new(0.0, 0.0);
            let sh1 = { let mut s = c1; s.sweep_radius = 1.0 - 0.04; s };
            let sh2 = { let mut s = c2; s.sweep_radius = 1.0 - 0.04; s };
            let t = time_of_impact(&sh1, &s1, &sh2, &s2);
            golden::number("toi.circles", t);
        });
    }
}
