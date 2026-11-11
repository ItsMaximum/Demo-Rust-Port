//! Host-only diagnostic-v1 serializer, shared by the CLI and golden tests.
//! This allocates outside the simulation step; it is excluded from no_std builds.
//! Every Number is emitted as its raw bits, including NaN payloads and signed zero.
use crate::b2::state::E_CIRCLE_SHAPE;
use crate::{Ctx, NONE};
use hw_real::Real;
use std::fmt::Write;
#[allow(dead_code)]
pub fn broadphase_string<R: Real>(ctx: &Ctx<R>) -> String {
    let bp = &ctx.world.broad_phase;
    let mut s = format!(
        "{},{},{},{},{},{},{}",
        bp.proxy_count,
        bp.free_proxy,
        bp.time_stamp,
        bp.query_result_count,
        bp.pair_count,
        bp.pair_buffer_count,
        bp.free_pair
    );
    for bounds in &bp.bounds {
        for b in bounds {
            write!(s, "|b{},{},{}", b.value, b.proxy_id, b.stabbing_count).unwrap();
        }
    }
    for p in &bp.proxy_pool {
        write!(
            s,
            "|p{},{},{},{},{},{}",
            p.lower_bounds[0],
            p.lower_bounds[1],
            p.upper_bounds[0],
            p.upper_bounds[1],
            p.overlap_count,
            p.time_stamp
        )
        .unwrap();
    }
    for i in 0..bp.query_result_count as usize {
        write!(s, "|q{}", bp.query_results[i]).unwrap();
    }
    for (i, p) in bp.pairs.iter().enumerate() {
        write!(
            s,
            "|r{},{},{},{},{}",
            p.proxy_id1, p.proxy_id2, p.status, p.next, bp.hash_table[i]
        )
        .unwrap();
    }
    for i in 0..bp.pair_buffer_count as usize {
        let p = bp.pair_buffer[i];
        write!(s, "|u{},{}", p.proxy_id1, p.proxy_id2).unwrap();
    }
    s
}
pub fn state_string<R: Real>(ctx: &Ctx<R>) -> String {
    let w = &ctx.world;
    let mut s = format!("b{}", w.body_count);
    let mut h = w.body_list;
    while h != NONE {
        let b = w.body(h);
        s.push('|');
        let values = [
            b.sweep.c.x,
            b.sweep.c.y,
            b.sweep.a,
            b.sweep.c0.x,
            b.sweep.c0.y,
            b.sweep.a0,
            b.sweep.local_center.x,
            b.sweep.local_center.y,
            b.linear_velocity.x,
            b.linear_velocity.y,
            b.angular_velocity,
            b.force.x,
            b.force.y,
            b.torque,
            b.mass,
            b.i,
            b.inv_mass,
            b.inv_i,
            b.sleep_time,
        ];
        for v in values {
            write!(s, "{:016x},", v.to_bits()).unwrap();
        }
        write!(s, "{},{}", b.flags, b.shape_count).unwrap();
        let mut sh = b.shape_list;
        while sh != NONE {
            let shape = w.shape(sh);
            write!(
                s,
                "/{},{},{},{},{},{},{},{:016x}",
                shape.type_,
                shape.proxy_id,
                shape.filter.category_bits,
                shape.filter.mask_bits,
                shape.filter.group_index,
                shape.is_sensor.word(),
                shape.material,
                shape.sweep_radius.to_bits()
            )
            .unwrap();
            if shape.type_ == E_CIRCLE_SHAPE {
                for v in [shape.local_position.x, shape.local_position.y, shape.radius] {
                    write!(s, ",{:016x}", v.to_bits()).unwrap();
                }
            } else {
                write!(s, ",{}", shape.vertex_count).unwrap();
                for v in &shape.vertices[..shape.vertex_count as usize] {
                    write!(s, ",{:016x},{:016x}", v.x.to_bits(), v.y.to_bits()).unwrap();
                }
            }
            sh = shape.next;
        }
        h = b.next;
    }
    write!(s, "#j{}", w.joint_count).unwrap();
    let mut h = w.joint_list;
    while h != NONE {
        let j = w.joint(h);
        if j.type_ == 6 {
            s.push_str("|6");
            for v in [
                j.force,
                j.mass,
                j.ratio,
                j.constant,
                j.linear_jacobian.linear1.x,
                j.linear_jacobian.linear1.y,
                j.linear_jacobian.angular1,
                j.linear_jacobian.linear2.x,
                j.linear_jacobian.linear2.y,
                j.linear_jacobian.angular2,
            ] {
                write!(s, ",{:016x}", v.to_bits()).unwrap();
            }
            write!(s, ",{}", j.broken.word()).unwrap();
            h = j.next;
            continue;
        }
        if j.type_ == 3 {
            write!(
                s,
                "|3,{:016x},{:016x},{:016x},{:016x},{}",
                j.impulse.to_bits(),
                j.u.x.to_bits(),
                j.u.y.to_bits(),
                j.length.to_bits(),
                j.broken.word()
            )
            .unwrap();
            h = j.next;
            continue;
        }
        let values = if j.type_ == 1 {
            [
                j.pivot_force.x,
                j.pivot_force.y,
                j.motor_force,
                j.limit_force,
            ]
        } else {
            [j.force, j.torque, j.motor_force, j.limit_force]
        };
        assert!(j.type_ == 1 || j.type_ == 2);
        write!(s, "|{}", j.type_).unwrap();
        for v in values {
            write!(s, ",{:016x}", v.to_bits()).unwrap();
        }
        write!(
            s,
            ",{},{},{}",
            j.limit_state, j.enable_motor.word(), j.enable_limit.word()
        )
        .unwrap();
        let values = if j.type_ == 1 {
            [
                j.motor_speed,
                j.lower_angle,
                j.upper_angle,
                j.max_motor_torque,
            ]
        } else {
            [
                j.motor_speed,
                j.lower_translation,
                j.upper_translation,
                j.max_motor_force,
            ]
        };
        for v in values {
            write!(s, ",{:016x}", v.to_bits()).unwrap();
        }
        write!(s, ",{}", j.broken.word()).unwrap();
        h = j.next;
    }
    let mut acc = R::zero();
    let mut count = 0;
    let mut contact = w.contact_list;
    while contact != NONE {
        let c = w.contact(contact);
        for _ in 0..c.manifold_count {
            for p in &c.manifold.points[..c.manifold.point_count as usize] {
                acc = acc + p.normal_impulse;
            }
        }
        count += 1;
        contact = c.next;
    }
    write!(
        s,
        "#c{},{:016x},{},{},{},{}",
        count,
        acc.to_bits(),
        w.broad_phase.proxy_count,
        w.broad_phase.pair_count,
        w.broad_phase.pair_buffer_count,
        w.lock.word()
    )
    .unwrap();
    s
}

pub fn oaat(s: &str, seed: u32) -> u32 {
    let mut h = seed;
    for ch in s.bytes() {
        h = h.wrapping_add(ch as u32);
        h = h.wrapping_add(h << 10);
        h ^= h >> 6;
    }
    h = h.wrapping_add(h << 3);
    h ^= h >> 11;
    h.wrapping_add(h << 15)
}

trait RawBits: Real {
    fn to_bits(self) -> u64 {
        let [hi, lo] = self.to_words();
        ((hi as u64) << 32) | lo as u64
    }
}
impl<R: Real> RawBits for R {}

pub fn replay_string<R: Real>(ctx: &Ctx<R>, replay: &crate::game::session::ReplayState) -> String {
    let focus = crate::b2::body::get_world_center(ctx, ctx.character.camera_focus);
    format!(
        "{}#g{},{},{},{:016x},{:016x}",
        state_string(ctx),
        ctx.character.dead.word(),
        ctx.character.current_pose,
        replay.iteration,
        focus.x.to_bits(),
        focus.y.to_bits()
    )
}

pub fn hash(s: &str) -> String {
    format!("{:08x}{:08x}", oaat(s, 0), oaat(s, 2654435769))
}
