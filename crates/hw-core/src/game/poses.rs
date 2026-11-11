//! CharacterB2D motor poses and grip arming. Table order follows each original method.
use super::character::*;
use crate::b2::{body, joints, world};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_real::{as3, Real};

pub fn set_pose<R: Real>(ctx: &mut Ctx<R>, pose: i32) {
    if ctx.character.current_pose == pose {
        return;
    }
    ctx.character.current_pose = pose;
    if pose == 0 {
        let items = [
            NECK, SHOULDER1, SHOULDER2, ELBOW1, ELBOW2, HIP1, HIP2, KNEE1, KNEE2,
        ];
        for item_index in 0..items.len() {
            let index = items[item_index];
            let j = ctx.character.joints[index];
            if j != NONE && joints::is_motor_enabled(ctx, j) {
                joints::enable_motor(ctx, j, false);
            }
        }
    }
}
pub fn set_joint<R: Real>(ctx: &mut Ctx<R>, j: H, offset: R, k: R, max: R) -> Flow {
    if j == NONE {
        return Flow::Abort(AbortSite::GameNullDeref);
    }
    if !joints::is_motor_enabled(ctx, j) {
        joints::enable_motor(ctx, j, true);
    }
    let mut dir = R::from_i32(-1);
    let target = ctx.world.joint(j).lower_angle + offset;
    let mut d = joints::get_joint_angle(ctx, j) - target;
    if d.lt(R::zero()) {
        dir = R::one();
        d = as3::math_abs(d);
    }
    let speed = as3::math_min(k * (d * d), max) * dir;
    joints::set_motor_speed(ctx, j, speed);
    Flow::Continue
}
/// Base poses 1..4, WheelChairGuy poses 5..7.
pub fn wheelchair_pose<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let row = match ctx.character.current_pose {
        1 => [
            (hw_real::rl!(R; 0.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 2.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 2.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 1.0), hw_real::rl!(R; 15.0)),
            (hw_real::rl!(R; 1.0), hw_real::rl!(R; 15.0)),
        ],
        2 => [
            (hw_real::rl!(R; 0.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 2.6), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 2.6), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 1.7), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 1.7), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 2.5), hw_real::rl!(R; 15.0)),
            (hw_real::rl!(R; 2.5), hw_real::rl!(R; 15.0)),
        ],
        3 => [
            (hw_real::rl!(R; 0.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 2.6), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 2.6), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 2.5), hw_real::rl!(R; 15.0)),
            (hw_real::rl!(R; 2.5), hw_real::rl!(R; 15.0)),
        ],
        4 => [
            (hw_real::rl!(R; 0.69), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 2.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 2.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 1.5), hw_real::rl!(R; 15.0)),
            (hw_real::rl!(R; 1.5), hw_real::rl!(R; 15.0)),
        ],
        5 => [
            (hw_real::rl!(R; 0.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 0.5), hw_real::rl!(R; 5.0)),
            (hw_real::rl!(R; 0.5), hw_real::rl!(R; 5.0)),
            (hw_real::rl!(R; 1.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 1.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 1.4), hw_real::rl!(R; 15.0)),
            (hw_real::rl!(R; 1.4), hw_real::rl!(R; 15.0)),
        ],
        6 => [
            (hw_real::rl!(R; 0.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 3.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 1.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
            (-hw_real::rl!(R; 0.5), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 1.0), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 15.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 15.0)),
        ],
        7 => [
            (hw_real::rl!(R; 0.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 1.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 3.5), hw_real::rl!(R; 2.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
            (hw_real::rl!(R; 1.0), hw_real::rl!(R; 20.0)),
            (-hw_real::rl!(R; 0.5), hw_real::rl!(R; 20.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 15.0)),
            (hw_real::rl!(R; 3.0), hw_real::rl!(R; 15.0)),
        ],
        _ => return Flow::Continue,
    };
    let items = [
        NECK, HIP1, HIP2, KNEE1, KNEE2, SHOULDER1, SHOULDER2, ELBOW1, ELBOW2,
    ];
    for i in 0..items.len() {
        let j = items[i];
        flow!(set_joint(
            ctx,
            ctx.character.joints[j],
            row[i].0,
            row[i].1,
            hw_real::rl!(R; 10.0)
        ));
    }
    Flow::Continue
}
pub fn reset_joint_limits<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let degrees = hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI);
    let items = [
        (CHEST, HEAD, -hw_real::rl!(R; 20.0), hw_real::rl!(R; 20.0)),
        (CHEST, PELVIS, -hw_real::rl!(R; 5.0), hw_real::rl!(R; 5.0)),
        (CHEST, UPPER_ARM1, -hw_real::rl!(R; 180.0), hw_real::rl!(R; 60.0)),
        (CHEST, UPPER_ARM2, -hw_real::rl!(R; 180.0), hw_real::rl!(R; 60.0)),
        (UPPER_ARM1, LOWER_ARM1, -hw_real::rl!(R; 160.0), hw_real::rl!(R; 0.0)),
        (UPPER_ARM2, LOWER_ARM2, -hw_real::rl!(R; 160.0), hw_real::rl!(R; 0.0)),
        (PELVIS, UPPER_LEG1, -hw_real::rl!(R; 150.0), hw_real::rl!(R; 10.0)),
        (PELVIS, UPPER_LEG2, -hw_real::rl!(R; 150.0), hw_real::rl!(R; 10.0)),
        (UPPER_LEG1, LOWER_LEG1, hw_real::rl!(R; 0.0), hw_real::rl!(R; 150.0)),
        (UPPER_LEG2, LOWER_LEG2, hw_real::rl!(R; 0.0), hw_real::rl!(R; 150.0)),
    ];
    for index in 0..items.len() {
        let (a, b, lo, hi) = items[index];
        let j = ctx.character.joints[index];
        if j == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        let (a, b) = if index == SHOULDER1 || index == SHOULDER2 || index == HIP1 || index == HIP2 {
            (ctx.world.joint(j).body1, ctx.world.joint(j).body2)
        } else {
            (ctx.character.bodies[a], ctx.character.bodies[b])
        };
        let angle =
            body::get_angle(ctx, b) - body::get_angle(ctx, a) - joints::get_joint_angle(ctx, j);
        joints::set_limits(
            ctx,
            j,
            lo / degrees - angle,
            hi / degrees - angle,
        );
    }
    Flow::Continue
}
pub fn end_grab<R: Real>(ctx: &mut Ctx<R>) {
    let items = [LOWER_ARM1, LOWER_ARM2];
    for item_index in 0..items.len() {
        let part = items[item_index];
        ctx.world.shape_mut(ctx.character.shapes[part]).handlers[3] = 0;
    }
}
pub fn start_grab<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if !ctx.character.grabbing {
        ctx.character.grabbing = crate::Flag32::TRUE;
        let items = [
            (SHOULDER1, ELBOW1, LOWER_ARM1),
            (SHOULDER2, ELBOW2, LOWER_ARM2),
        ];
        for item_index in 0..items.len() {
            let (sh, el, part) = items[item_index];
            let sh = ctx.character.joints[sh];
            let el = ctx.character.joints[el];
            if sh == NONE {
                return Flow::Abort(AbortSite::GameNullDeref);
            }
            if !ctx.world.joint(sh).broken {
                if el == NONE {
                    return Flow::Abort(AbortSite::GameNullDeref);
                }
                if !ctx.world.joint(el).broken {
                    ctx.world.shape_mut(ctx.character.shapes[part]).handlers[3] =
                        handler(ctx, part as u32);
                }
            }
        }
    } else {
        end_grab(ctx);
    }
    Flow::Continue
}
pub fn release_grip<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if ctx.character.grabbing.get() {
        ctx.character.grabbing = crate::Flag32::FALSE;
        end_grab(ctx);
        for i in 0..2 {
            let j = ctx.character.grip_joints[i];
            if j != NONE {
                flow!(world::destroy_joint(ctx, j));
                ctx.world.joints.release_root(j);
                ctx.character.grip_joints[i] = NONE;
            }
        }
    }
    Flow::Continue
}
