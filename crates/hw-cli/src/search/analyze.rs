//! `hw-search analyze`: per-frame launch anatomy from the `lab` hooks.
use super::sim::{chest, Sim, State};
use hw_core::b2::body;
use hw_core::lab::{self, Event};
use hw_core::{H, NONE};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

/// Human-readable name of a shape: character part, level geometry (with its world centroid),
/// or another body.
pub fn shape_label(ctx: &State, s: H) -> String {
    let sh = ctx.world.shape(s);
    let b = sh.body;
    for (i, &cb) in ctx.character.bodies.iter().enumerate() {
        if cb == b && b != NONE {
            return format!("char:{i}");
        }
    }
    if b == ctx.character.heart && b != NONE {
        return "char:heart".into();
    }
    let xf = ctx.world.body(b).xf;
    let local = if sh.type_ == 0 { sh.local_position } else { sh.centroid };
    let x = xf.position.x + xf.r.col1.x * local.x + xf.r.col2.x * local.y;
    let y = xf.position.y + xf.r.col1.y * local.x + xf.r.col2.y * local.y;
    let kind = if b == ctx.level.level_body { "level" } else if body::is_static(ctx, b) { "static" } else { "body" };
    format!("{kind}#{s}@({x:.2},{y:.2})")
}

/// Level-border hits of the chest on the last frame of `masks` (its launch frame).
#[derive(Clone, Copy, Debug, Default)]
pub struct Route {
    pub top: bool,
    pub right: bool,
    pub left: bool,
    /// Ordered level hits (T top, R right, L left, o other level shape), repeats collapsed,
    /// first 8.
    pub seq: [u8; 8],
    pub seq_len: usize,
}

impl Route {
    /// The chest's first level hits are the top border, then the right wall.
    pub fn clean(&self) -> bool {
        self.seq_len >= 2 && self.seq[0] == b'T' && self.seq[1] == b'R'
    }
    pub fn label(&self) -> String {
        let seq: String = self.seq[..self.seq_len].iter().map(|&c| c as char).collect();
        format!("{} {}", self.label_sets(), seq)
    }
    fn label_sets(&self) -> String {
        let mut v = Vec::new();
        if self.top { v.push("top"); }
        if self.right { v.push("right"); }
        if self.left { v.push("left"); }
        if v.is_empty() { "none".into() } else { v.join("+") }
    }
}

/// Replays `masks` and records which world borders the chest's TOI chain hits on the last frame.
pub fn launch_route(template: &Sim, masks: &[u8]) -> Route {
    let mut sim = template.fork();
    sim.masks = masks.to_vec();
    let mut r = Route::default();
    for i in 0..masks.len() {
        let last = i + 1 == masks.len();
        if last {
            lab::arm(&watch_list(&sim.ctx));
        }
        sim.step(0);
        if last {
            lab::disarm();
            let chest_h = sim.ctx.character.bodies[hw_core::game::character::CHEST];
            for e in lab::take_events() {
                if let Event::Toi { shape1, shape2, body1, body2, .. } = e {
                    if body1 != chest_h && body2 != chest_h {
                        continue;
                    }
                    let other = if body1 == chest_h { shape2 } else { shape1 };
                    let sh = sim.ctx.world.shape(other);
                    if sh.body != sim.ctx.level.level_body {
                        continue;
                    }
                    let xf = sim.ctx.world.body(sh.body).xf;
                    let local = if sh.type_ == 0 { sh.local_position } else { sh.centroid };
                    let x = xf.position.x + xf.r.col1.x * local.x + xf.r.col2.x * local.y;
                    let y = xf.position.y + xf.r.col1.y * local.x + xf.r.col2.y * local.y;
                    // Borders: left wall centre x = -2, right wall x = 322, ceiling y = -82 (m).
                    let c = if y < -80.0 {
                        r.top = true;
                        b'T'
                    } else if x > 320.0 {
                        r.right = true;
                        b'R'
                    } else if x < 0.0 {
                        r.left = true;
                        b'L'
                    } else {
                        b'o'
                    };
                    if r.seq_len < 8 && (r.seq_len == 0 || r.seq[r.seq_len - 1] != c) {
                        r.seq[r.seq_len] = c;
                        r.seq_len += 1;
                    }
                }
            }
        }
    }
    r
}

pub fn watch_list(ctx: &State) -> Vec<H> {
    let mut w: Vec<H> = ctx.character.bodies.iter().copied().filter(|&b| b != NONE).collect();
    if ctx.character.heart != NONE {
        w.push(ctx.character.heart);
    }
    w
}

/// Events of one armed frame as JSON (chest-centric).
pub fn describe(ctx: &State, events: &[Event]) -> Value {
    let chest_h = ctx.character.bodies[hw_core::game::character::CHEST];
    let mut solve = Value::Null;
    let mut tois = Vec::new();
    let mut finish = Vec::new();
    let mut freezes = Vec::new();
    let mut smash = false;
    for e in events {
        match *e {
            Event::Solve { body, c0, c, v, w, injected } if body == chest_h => {
                solve = json!({"c0": c0, "c": c, "v": v, "w": w, "vdt": [v[0] / 30.0, v[1] / 30.0],
                    "corrected_move": ((c[0]-c0[0]).powi(2)+(c[1]-c0[1]).powi(2)).sqrt(), "injected": injected});
            }
            Event::Toi { toi, shape1, shape2, body1, body2 } if body1 == chest_h || body2 == chest_h => {
                let other = if body1 == chest_h { shape2 } else { shape1 };
                tois.push(json!({"toi": toi, "hit": shape_label(ctx, other)}));
            }
            Event::AfterToi { body, c, v, .. } if body == chest_h => {
                if let Some(last) = tois.last_mut() {
                    last["after_c"] = json!(c);
                    last["after_v"] = json!(v);
                }
            }
            Event::Finish { x, y, inside, dead, other } => finish.push(json!({"x": x, "y": y, "inside": inside, "dead": dead, "other": shape_label(ctx, other)})),
            Event::Freeze { body } => freezes.push(json!(body)),
            Event::ChestSmash => smash = true,
            _ => {}
        }
    }
    json!({"chest_solve": solve, "chest_tois": tois, "finish_checks": finish, "frozen": freezes, "chest_smash": smash})
}

pub fn run(level: u32, character: u32, masks: &[u8], from: u32, to: u32) -> Result<(), String> {
    // Exact TAS semantics: the string's own length ends input, as in `hw sim`.
    let mut sim = Sim::new(level, character)?;
    sim.replay = hw_core::game::session::ReplayState::new(masks.len() as u32);
    sim.masks = masks.to_vec();
    for frame in 1..=to {
        let armed = frame >= from;
        if armed {
            lab::arm(&watch_list(&sim.ctx));
        }
        let f = sim.step(0);
        if armed {
            lab::disarm();
            let events = lab::take_events();
            let mut line = describe(&sim.ctx, &events);
            line["frame"] = json!(frame);
            line["input"] = json!(super::sim::tas_text(&[f.mask]));
            line["site"] = json!(f.site);
            line["alive_before"] = json!(f.alive_before);
            line["dead_after"] = json!(!f.alive_after);
            line["finish"] = json!(sim.replay.finish_frame);
            line["chest_before"] = json!(f.chest_before);
            line["chest_after"] = json!(chest(&sim.ctx));
            line["displacement"] = json!(f.displacement);
            line["qe"] = json!(f.qe);
            line["qe_raw"] = json!(f.qe_raw);
            line["ejected"] = json!(f.ejected);
            line["grips"] = json!(f.grips);
            let held: Vec<_> = sim.ctx.character.grip_joints.iter().filter(|&&j| j != NONE && sim.ctx.world.joints.is_alive(j)).map(|&j| {
                let b = sim.ctx.world.joint(j).body1;
                let bd = sim.ctx.world.body(b);
                json!({"body": b, "pos": [bd.sweep.c.x, bd.sweep.c.y], "mass": bd.mass, "static": bd.inv_mass == 0.0})
            }).collect();
            line["held"] = json!(held);
            println!("{line}");
        }
    }
    Ok(())
}

/// JSON cannot represent NaN or infinity numerically. Keep their exact words,
/// and also the sign of zero, instead of normalizing diagnostic evidence.
fn raw_f64(value: f64) -> (Value, String) {
    (serde_json::Number::from_f64(value).map(Value::Number).unwrap_or(Value::Null),
        format!("{:016x}", value.to_bits()))
}

fn raw_pair(values: [f64; 2]) -> ([Value; 2], [String; 2]) {
    let (x, x_bits) = raw_f64(values[0]);
    let (y, y_bits) = raw_f64(values[1]);
    ([x, y], [x_bits, y_bits])
}

fn raw_damage_buffer(buffer: hw_core::lab::DamageBuffer) -> Value {
    let (impulse,impulse_bits)=raw_f64(buffer.impulse);
    json!({"present":buffer.present,"shape":buffer.shape,"other":buffer.other,
        "own_body":buffer.own_body,"other_body":buffer.other_body,"impulse":impulse,"impulse_bits":impulse_bits})
}

/// Preserve every recorded event and its direct identities. No poststep actor
/// alias, label lookup, TOI pairing, derived arithmetic or event filter is used.
fn raw_event(ordinal: usize, event: &Event) -> Value {
    match *event {
        Event::Solve { body, c0, c, v, w, injected } => {
            let (c0, c0_bits) = raw_pair(c0);
            let (c, c_bits) = raw_pair(c);
            let (v, v_bits) = raw_pair(v);
            let (w, w_bits) = raw_f64(w);
            json!({"ordinal":ordinal,"event":"solve","body":body,"c0":c0,"c0_bits":c0_bits,
                "c":c,"c_bits":c_bits,"v":v,"v_bits":v_bits,"w":w,"w_bits":w_bits,"injected":injected})
        }
        Event::Toi { toi, shape1, shape2, body1, body2 } => {
            let (toi, toi_bits) = raw_f64(toi);
            json!({"ordinal":ordinal,"event":"toi","toi":toi,"toi_bits":toi_bits,
                "shape1":shape1,"shape2":shape2,"body1":body1,"body2":body2})
        }
        Event::AfterToi { body, c0, c, v } => {
            let (c0, c0_bits) = raw_pair(c0);
            let (c, c_bits) = raw_pair(c);
            let (v, v_bits) = raw_pair(v);
            json!({"ordinal":ordinal,"event":"after_toi","body":body,
                "c0":c0,"c0_bits":c0_bits,"c":c,"c_bits":c_bits,"v":v,"v_bits":v_bits})
        }
        Event::Freeze { body } => json!({"ordinal":ordinal,"event":"freeze","body":body}),
        Event::Finish { x, y, inside, dead, other } => {
            let (x, x_bits) = raw_f64(x);
            let (y, y_bits) = raw_f64(y);
            json!({"ordinal":ordinal,"event":"finish","x":x,"x_bits":x_bits,
                "y":y,"y_bits":y_bits,"inside":inside,"dead":dead,"other":other})
        }
        Event::ChestSmash => json!({"ordinal":ordinal,"event":"chest_smash"}),
        Event::Strand { toi, shape1, shape2, body1, body2, pos1, pos2 } => {
            let (toi, toi_bits) = raw_f64(toi);
            let (pos1, pos1_bits) = raw_pair(pos1);
            let (pos2, pos2_bits) = raw_pair(pos2);
            json!({"ordinal":ordinal,"event":"strand","toi":toi,"toi_bits":toi_bits,
                "shape1":shape1,"shape2":shape2,"body1":body1,"body2":body2,
                "pos1":pos1,"pos1_bits":pos1_bits,"pos2":pos2,"pos2_bits":pos2_bits})
        }
        Event::DamageResult { phase, body, actor_kind, child, dead, shape, own, other,
            own_body, other_body, impulse, threshold, previous } => {
            let phase=match phase {hw_core::lab::ReportPhase::None=>"none",hw_core::lab::ReportPhase::Main=>"main",hw_core::lab::ReportPhase::Toi=>"toi"};
            let (impulse,impulse_bits)=raw_f64(impulse);let (threshold,threshold_bits)=raw_f64(threshold);
            json!({"ordinal":ordinal,"event":"damage_result","phase":phase,"body":body,"actor_kind":actor_kind,
                "child":child,"dead":dead,"shape":shape,"own":own,"other":other,"own_body":own_body,"other_body":other_body,
                "impulse":impulse,"impulse_bits":impulse_bits,"threshold":threshold,"threshold_bits":threshold_bits,
                "previous":raw_damage_buffer(previous)})
        }
        Event::DamageSnapshot { body, actor_kind, child, dead, shape, threshold, smash_limit, buffer }
        | Event::DamageConsume { body, actor_kind, child, dead, shape, threshold, smash_limit, buffer } => {
            let name=if matches!(*event,Event::DamageSnapshot {..}) {"damage_snapshot"}else{"damage_consume"};
            let (threshold,threshold_bits)=raw_f64(threshold);let (smash_limit,smash_limit_bits)=raw_f64(smash_limit);
            json!({"ordinal":ordinal,"event":name,"body":body,"actor_kind":actor_kind,"child":child,"dead":dead,"shape":shape,
                "threshold":threshold,"threshold_bits":threshold_bits,"smash_limit":smash_limit,"smash_limit_bits":smash_limit_bits,
                "buffer":raw_damage_buffer(buffer)})
        }
    }
}

fn raw_events(events: &[Event]) -> Vec<Value> {
    events.iter().enumerate().map(|(ordinal, event)| raw_event(ordinal, event)).collect()
}

fn raw_pair_exhaustion(capture: lab::PairExhaustionCapture) -> Value {
    let first = capture.first.map(|p| json!({
        "proxy_ids":p.proxy_ids,"hash_bucket":p.hash_bucket,"free_pair":p.free_pair,
        "pair_count":p.pair_count,"pair_buffer_count":p.pair_buffer_count,
        "proxy_slots":p.proxy_slots.map(|s| json!({"readable":s.readable,"valid":s.valid,"shape":s.shape})),
        "operation":{"kind":format!("{:?}",p.operation.kind),"initiating_proxy_id":p.operation.initiating_proxy_id},
        "world_phase":format!("{:?}",p.world_phase),"report_phase":format!("{:?}",p.report_phase),
        "events_before":p.events_before,
        "failed_new_pair_lookup_at_capture":true,"proxy_valid_is_only_slot_flag":true,
        "shape_handles_are_capture_time_slots_not_historical_body_owners":true
    }));
    json!({"first":first,"count":capture.count,"count_saturated":capture.count_saturated,
        "scope":"armed global AddPair exhaustion; not attributed to primary Dad"})
}

/// Immutable linked-list reads. A missing slot, cycle or excessive length makes
/// absence unknown; no gameplay getter that changes query scratch is invoked.
fn bounded_handles(head: H, capacity: usize, next: impl Fn(H) -> Option<H>) -> (BTreeSet<H>, bool) {
    let mut found=BTreeSet::new();let mut h=head;
    while h!=NONE {
        if found.len()>=capacity || found.contains(&h) {return (found,false);}
        match next(h) {Some(n)=>{found.insert(h);h=n;},None=>return (found,false)}
    }
    (found,true)
}

fn raw_proxy(ctx: &State, id: u32, shape: H) -> Value {
    let bp=&ctx.world.broad_phase;
    let Some(p)=bp.proxy_pool.get(id as usize) else {return json!({"id":id,"readable":false});};
    let count=bp.proxy_count.checked_mul(2).filter(|&n|n>=0).map(|n|n as usize);
    let bounds:Vec<_>=(0..2).map(|axis| {
        let read=|index:u32| bp.bounds[axis].get(index as usize).map(|b|json!({
            "index":index,"value":b.value,"proxy_id":b.proxy_id,"stabbing_count":b.stabbing_count,
            "within_declared_bound_count":count.map(|n|(index as usize)<n)}));
        json!({"lower":read(p.lower_bounds[axis]),"upper":read(p.upper_bounds[axis])})
    }).collect();
    json!({"id":id,"readable":true,"valid_slot_flag":p.overlap_count!=hw_core::b2::state::B2_INVALID,
        "shape":p.shape,"shape_back_reference_matches":p.shape==shape,
        "overlap_count":p.overlap_count,"time_stamp":p.time_stamp,"bounds":bounds,
        "declared_bound_count_valid":count.is_some_and(|n|n<=bp.bounds[0].len()),
        "valid_slot_is_not_fully_committed_proxy_certificate":true})
}

/// Host-derived bounds, with no collision query or simulation mutation. Exact
/// stored vertex/centroid words are exported independently of these bounds.
fn polygon_bounds(ctx: &State, shape: H) -> Option<[f64;4]> {
    let s=ctx.world.shapes.items.get(shape as usize)?;
    if shape==NONE || s.type_!=1 || s.vertex_count!=4 {return None;}
    let b=ctx.world.bodies.items.get(s.body as usize)?;
    let mut bounds=[f64::INFINITY,f64::INFINITY,f64::NEG_INFINITY,f64::NEG_INFINITY];
    for v in &s.vertices[..4] {
        let x=b.xf.position.x+b.xf.r.col1.x*v.x+b.xf.r.col2.x*v.y;
        let y=b.xf.position.y+b.xf.r.col1.y*v.x+b.xf.r.col2.y*v.y;
        if !x.is_finite() || !y.is_finite() {return None;}
        bounds[0]=bounds[0].min(x);bounds[1]=bounds[1].min(y);
        bounds[2]=bounds[2].max(x);bounds[3]=bounds[3].max(y);
    }
    Some(bounds)
}

fn fixed_box_matches(ctx: &State, shape: H, sensor: bool, expected: [f64;4]) -> bool {
    ctx.world.shapes.items.get(shape as usize).is_some_and(|s|
        s.body==ctx.level.level_body && s.is_sensor.get()==sensor &&
        polygon_bounds(ctx,shape).is_some_and(|bounds|bounds.map(f64::to_bits)==expected.map(f64::to_bits)))
}

fn inventory_shape(ctx: &State, h: H, world_bodies: &BTreeSet<H>, bodies_complete: bool) -> Value {
    let w=&ctx.world;
    let Some(s)=w.shapes.items.get(h as usize) else {return json!({"handle":h,"readable":false});};
    let (centroid,centroid_bits)=raw_pair([s.centroid.x,s.centroid.y]);
    let vertices:Vec<_>=s.vertices.iter().take(s.vertex_count.max(0) as usize).map(|v| {
        let (value,bits)=raw_pair([v.x,v.y]);json!({"value":value,"bits":bits})
    }).collect();
    json!({"handle":h,"readable":true,"arena_alive":w.shapes.alive[h as usize]!=0,
        "retained":w.shapes.retained[h as usize],"body":s.body,
        "body_world_list_member":if world_bodies.contains(&s.body){Some(true)}else if bodies_complete{Some(false)}else{None},
        "body_readable":w.bodies.items.get(s.body as usize).is_some(),
        "sensor":s.is_sensor.get(),"type":s.type_,"material":s.material,"handlers":s.handlers,
        "filter":{"category":s.filter.category_bits,"mask":s.filter.mask_bits,"group":s.filter.group_index},
        "vertex_count":s.vertex_count,"vertex_count_within_capacity":s.vertex_count>=0 && (s.vertex_count as usize)<=s.vertices.len(),
        "centroid":centroid,"centroid_bits":centroid_bits,"vertices":vertices,
        "derived_world_box":polygon_bounds(ctx,h),"proxy":raw_proxy(ctx,s.proxy_id,h)})
}

/// A current-slot identity map, copied before input so a later rejected pair can
/// refer back to it. No proxy/body generation or intrastep owner is inferred.
fn pair_shape_identities(ctx: &State, phase: &str, extra_shapes: &[H]) -> BTreeMap<H,Value> {
    let w=&ctx.world;
    let (world_bodies,complete)=bounded_handles(w.body_list,w.bodies.items.len(),|h|w.bodies.items.get(h as usize).map(|b|b.next));
    let mut body_refs=Vec::new();let mut shape_refs=Vec::new();
    actor_refs("primary",&ctx.character.primary,&mut body_refs,&mut shape_refs);
    actor_refs("child",&ctx.character.child,&mut body_refs,&mut shape_refs);
    body_refs.push(("level_body".into(),ctx.level.level_body));
    let mut identities=BTreeMap::new();
    for h in w.broad_phase.proxy_pool.iter().map(|p|p.shape).chain(extra_shapes.iter().copied()) {
        if h==NONE || identities.contains_key(&h) {continue;}
        let Some(s)=w.shapes.items.get(h as usize) else {continue;};
        let names:Vec<_>=shape_refs.iter().filter(|(_,shape)|*shape==h).map(|(name,_)|name).collect();
        let body_names:Vec<_>=body_refs.iter().filter(|(_,body)|*body==s.body).map(|(name,_)|name).collect();
        let special_refs:Vec<_>=ctx.level.specials.iter().enumerate().filter(|(_,special)|special.shape==h)
            .map(|(i,special)|json!({"index":i,"kind":format!("{:?}",special.kind)})).collect();
        let body=w.bodies.items.get(s.body as usize).map(|b|json!({
            "handle":s.body,"arena_alive":w.bodies.alive[s.body as usize]!=0,"retained":w.bodies.retained[s.body as usize],
            "type":b.type_,"flags":b.flags,"user_data":b.user_data,
            "world_list_member":if world_bodies.contains(&s.body){Some(true)}else if complete{Some(false)}else{None}}));
        identities.insert(h,json!({"phase":phase,"shape":h,"arena_alive":w.shapes.alive[h as usize]!=0,
            "retained":w.shapes.retained[h as usize],"body_handle":s.body,"body":body,"proxy_id":s.proxy_id,
            "material":s.material,"id_tag":s.id_tag,"handlers":s.handlers,"sensor":s.is_sensor.get(),
            "filter":{"category":s.filter.category_bits,"mask":s.filter.mask_bits,"group":s.filter.group_index},
            "current_shape_actor_refs":names,"current_body_actor_refs":body_names,"current_special_sensor_refs":special_refs,
            "identity_is_current_slot_only":true}));
    }
    identities
}

fn failed_pair_resolution(ctx: &State, capture: &lab::PairExhaustionCapture, before: &BTreeMap<H,Value>) -> Value {
    let Some(p)=capture.first else {return Value::Null;};
    let copied_shapes:Vec<_>=p.proxy_slots.iter().filter_map(|s|s.shape).collect();
    let after=pair_shape_identities(ctx,"after_end_frame_current_slot",&copied_shapes);
    let rows:Vec<_>=p.proxy_slots.iter().enumerate().map(|(i,slot)| {
        let proxy=ctx.world.broad_phase.proxy_pool.get(p.proxy_ids[i] as usize);
        let before_shape=slot.shape.and_then(|s|before.get(&s));
        let after_shape=slot.shape.and_then(|s|after.get(&s));
        json!({"captured_proxy_id":p.proxy_ids[i],"captured_shape":slot.shape,
            "captured_slot_readable":slot.readable,"captured_slot_valid_flag":slot.valid,
            "before_input_current_slot":before_shape,"after_end_frame_current_slot":after_shape,
            "after_proxy_slot_shape":proxy.map(|v|v.shape),
            "after_proxy_shape_matches_capture":proxy.and_then(|v|slot.shape.map(|h|v.shape==h)),
            "capture_time_body_owner_proven":false,
            "qualification":"matching pre/post shape slots and actor references do not prove no intrastep handle reuse"})
    }).collect();
    json!({"slots":rows,"pre_or_post_missing_resolution_is_unknown":true})
}

fn inventory_pair(ctx: &State, a: H, b: H, world_contacts: &BTreeSet<H>, contacts_complete: bool) -> Value {
    use hw_core::b2::state::{B2_INVALID,B2_NULL_PAIR,B2_TABLE_MASK,NULL_CONTACT};
    let w=&ctx.world;let bp=&w.broad_phase;
    let shapes=[a,b].map(|h|w.shapes.items.get(h as usize));
    let ids=shapes.map(|s|s.map(|s|s.proxy_id));
    let slots=ids.map(|id|id.and_then(|id|bp.proxy_pool.get(id as usize)));
    let usable=slots.iter().zip([a,b]).all(|(p,h)|p.is_some_and(|p|p.overlap_count!=B2_INVALID && p.shape==h));
    if !usable {return json!({"shapes":[a,b],"proxy_ids":ids,"lookup_performed":false,"absence_known":false});}
    let mut ids=ids.map(|id|id.unwrap_or(u32::MAX));ids.sort_unstable();
    let hash=hw_core::b2::broad_phase::hash(ids[0],ids[1]) & B2_TABLE_MASK;
    let mut h=bp.hash_table[hash as usize];let mut seen=BTreeSet::new();let mut matches=Vec::new();let mut complete=true;
    while h!=B2_NULL_PAIR {
        if seen.len()>=bp.pairs.len() || !seen.insert(h) {complete=false;break;}
        let Some(p)=bp.pairs.get(h as usize) else {complete=false;break;};
        if [p.proxy_id1,p.proxy_id2]==ids {
            let contact=if p.contact==NONE {json!({"kind":"none_or_not_committed"})}
                else if p.contact==NULL_CONTACT {json!({"kind":"null_contact_sentinel"})}
                else if let Some(c)=w.contacts.items.get(p.contact as usize) {
                    let (toi,toi_bits)=raw_f64(c.toi);
                    json!({"kind":"arena_slot","handle":p.contact,"arena_alive":w.contacts.alive[p.contact as usize]!=0,
                        "retained":w.contacts.retained[p.contact as usize],"flags":c.flags,"shapes":[c.shape1,c.shape2],
                        "matches_target_shapes":(c.shape1==a && c.shape2==b)||(c.shape1==b && c.shape2==a),
                        "world_list_member":if world_contacts.contains(&p.contact){Some(true)}else if contacts_complete{Some(false)}else{None},
                        "manifold_count":c.manifold_count,"point_count":c.manifold.point_count,"toi":toi,"toi_bits":toi_bits})
                } else {json!({"kind":"unreadable_slot","handle":p.contact})};
            matches.push(json!({"index":h,"proxy_ids":[p.proxy_id1,p.proxy_id2],"status":p.status,
                "buffered":p.status&1!=0,"removed":p.status&2!=0,"final":p.status&4!=0,"contact":contact}));
        }
        h=p.next;
    }
    let n=bp.pair_buffer_count.max(0) as usize;
    let buffer:Vec<_>=bp.pair_buffer.iter().take(n).enumerate().filter_map(|(i,p)| {
        let mut tuple=[p.proxy_id1,p.proxy_id2];tuple.sort_unstable();(tuple==ids).then_some(i)
    }).collect();
    let absent=if complete {Some(matches.is_empty())}else{None};
    json!({"shapes":[a,b],"proxy_ids":ids,"lookup_performed":true,"hash_bucket":hash,
        "hash_chain_complete":complete,"matches":matches,"absence_known":complete,"absent_from_complete_hash_chain":absent,
        "pair_buffer_matching_indices":buffer,"pair_buffer_scan_complete":bp.pair_buffer_count>=0 && n<=bp.pair_buffer.len(),
        "pair_absence_is_not_collision_disable_or_filter_certificate":true})
}

/// Immutable host snapshots, not instantaneous throw snapshots. Sim::step runs
/// end_frame before returning; all identities below are snapshot-local slots.
fn raw_pair_inventory(ctx: &State, primary: H, level: u32, phase: &str) -> Value {
    let w=&ctx.world;
    if level!=45 || ctx.character.primary.kind!=3 {return json!({"phase":phase,"applicable":false});}
    let (world_bodies,bodies_complete)=bounded_handles(w.body_list,w.bodies.items.len(),|h|w.bodies.items.get(h as usize).map(|b|b.next));
    let (world_contacts,contacts_complete)=bounded_handles(w.contact_list,w.contacts.items.len(),|h|w.contacts.items.get(h as usize).map(|c|c.next));
    let chain=|body:H|w.bodies.items.get(body as usize).map(|b|
        bounded_handles(b.shape_list,w.shapes.items.len(),|h|w.shapes.items.get(h as usize).map(|s|s.next)))
        .unwrap_or_else(||(BTreeSet::new(),false));
    let (primary_shapes,primary_complete)=chain(primary);let (ground_shapes,ground_complete)=chain(ctx.level.level_body);
    // Retain source-order candidates even if exact derived bounds differ. A
    // candidate's identity is not upgraded by an approximate geometry match.
    let mut targets=vec![("world_ceiling_source_shape2_candidate",2)];
    let ceiling_matches=fixed_box_matches(ctx,2,false,[-4.0,-84.0,324.0,-80.0]);
    let sensor=ctx.level.specials.get(12).filter(|s|s.kind==hw_data::SpecialKind::Spikes).map(|s|s.shape);
    let sensor_matches=sensor.is_some_and(|h|fixed_box_matches(ctx,h,true,[0.0,66.18,24.24,66.98]) && ground_shapes.contains(&h));
    if let Some(h)=sensor {targets.push(("starting_special12_sensor_current_reference",h));}
    // The fixed Spikes constructor creates its backing immediately after its
    // sensor. This slot is a source-order candidate, not a generation proof.
    let backing_source_slot=sensor.and_then(|h|h.checked_add(1)).filter(|&h|h!=NONE);
    if let Some(h)=backing_source_slot {targets.push(("starting_spikes_backing_following_slot_candidate",h));}
    let backing:Vec<_>=ground_shapes.iter().copied().filter(|&h|fixed_box_matches(ctx,h,false,[0.0,65.86,24.24,66.18])).collect();
    for &h in &backing {if Some(h)!=backing_source_slot {targets.push(("starting_spikes_backing_exact_bounds_candidate",h));}}
    let shape_rows:Vec<_>=primary_shapes.iter().copied().map(|h|inventory_shape(ctx,h,&world_bodies,bodies_complete)).collect();
    let target_rows:Vec<_>=targets.iter().map(|(name,h)|json!({"role":name,"shape":inventory_shape(ctx,*h,&world_bodies,bodies_complete)})).collect();
    let mut pairs=Vec::new();
    for &a in &primary_shapes {
        if !w.shapes.items.get(a as usize).is_some_and(|s|s.body==primary) {continue;}
        for &(name,b) in &targets {
            pairs.push(json!({"target_role":name,"pair":inventory_pair(ctx,a,b,&world_contacts,contacts_complete)}));
        }
    }
    let primary_read=w.bodies.items.get(primary as usize);
    json!({"phase":phase,"applicable":true,"captured_primary":primary,
        "current_primary":ctx.character.primary.bodies[hw_core::game::character::CHEST],"focus":ctx.level.focus,
        "primary_dead":ctx.character.primary.dead.get(),"world_locked":w.lock.get(),
        "primary_readable":primary_read.is_some(),"primary_arena_alive":w.bodies.alive.get(primary as usize).map(|&v|v!=0),
        "primary_dynamic":primary_read.map(|b|b.type_==hw_core::b2::state::E_DYNAMIC_TYPE),
        "primary_frozen":primary_read.map(|b|b.flags&hw_core::b2::state::E_FROZEN_FLAG!=0),
        "primary_world_list_member":if world_bodies.contains(&primary){Some(true)}else if bodies_complete{Some(false)}else{None},
        "primary_shape_chain_complete":primary_complete,"ground_shape_chain_complete":ground_complete,
        "world_body_chain_complete":bodies_complete,"world_contact_chain_complete":contacts_complete,
        "ceiling_shape2_derived_bounds_exact_words_match":ceiling_matches,"starting_special12_sensor_handle":sensor,
        "starting_sensor_derived_bounds_exact_words_match":sensor_matches,"backing_exact_bounds_match_count":backing.len(),
        "backing_following_source_slot_candidate":backing_source_slot,"geometry_comparison":"exact f64 words only; no approximate identity",
        "source_order_candidates_are_not_verified_historical_identity":true,"shape_ids_are_not_historical_generation_ids":true,
        "global_pair_count":w.broad_phase.pair_count,"global_pair_buffer_count":w.broad_phase.pair_buffer_count,
        "free_pair":w.broad_phase.free_pair,"proxy_count":w.broad_phase.proxy_count,
        "primary_shapes":shape_rows,"targets":target_rows,"pairs":pairs,
        "scope":"immutable targeted pair/hash/buffer/contact inventory; not QueryAABB or collision evaluation",
        "absence_is_not_global_collision_disable":true,"after_step_snapshot_is_after_end_frame":phase=="after_end_frame"})
}

fn actor_context(actor: &hw_core::game::character::CharacterState<f64>) -> Value {
    json!({"kind":actor.kind,"dead":actor.dead.get(),"dying":actor.dying.get(),
        "pose":actor.current_pose,"grabbing":actor.grabbing.get(),"camera_focus":actor.camera_focus,
        "bodies":actor.bodies,"shapes":actor.shapes,"stumps":actor.stumps,
        "brain":actor.brain,"heart":actor.heart,"grip_joints":actor.grip_joints,
        "bicycle":{"bodies":actor.bicycle.bodies,"shapes":actor.bicycle.shapes,
            "helmet":actor.bicycle.helmet,"helmet_shape":actor.bicycle.helmet_shape,
            "ejected":actor.bicycle.ejected.get()},
        "child_seat":{"seat":actor.child_seat.seat,"seat_shape":actor.child_seat.seat_shape,
            "ejected":actor.child_seat.ejected.get(),"detached":actor.child_seat.detached.get()},
        "spine_composite_index":actor.spine,"intestine_composite_index":actor.intestine,
        "composite_count":actor.composite_count})
}

fn actor_refs(name: &str, actor: &hw_core::game::character::CharacterState<f64>,
    bodies: &mut Vec<(String,H)>, shapes: &mut Vec<(String,H)>) {
    for (i,&h) in actor.bodies.iter().enumerate() { bodies.push((format!("{name}.bodies[{i}]"),h)); }
    for (i,&h) in actor.stumps.iter().enumerate() { bodies.push((format!("{name}.stumps[{i}]"),h)); }
    for (part,h) in [("brain",actor.brain),("heart",actor.heart),("camera_focus",actor.camera_focus),
        ("bicycle.helmet",actor.bicycle.helmet),("child_seat.seat",actor.child_seat.seat)] {
        bodies.push((format!("{name}.{part}"),h));
    }
    for (i,&h) in actor.bicycle.bodies.iter().enumerate() { bodies.push((format!("{name}.bicycle.bodies[{i}]"),h)); }
    for (i,&h) in actor.shapes.iter().enumerate() { shapes.push((format!("{name}.shapes[{i}]"),h)); }
    for (i,&h) in actor.bicycle.shapes.iter().enumerate() { shapes.push((format!("{name}.bicycle.shapes[{i}]"),h)); }
    shapes.push((format!("{name}.bicycle.helmet_shape"),actor.bicycle.helmet_shape));
    shapes.push((format!("{name}.child_seat.seat_shape"),actor.child_seat.seat_shape));
}

/// Read-only, bounded identities after the previous end_frame and before any
/// input/physics of this step. Arena readability, arena liveness, world-list
/// membership and actor references are separate facts, never an ownership or
/// force inference. H is a slot index; the arena has no per-slot generation.
fn raw_context(ctx: &State, primary: H) -> Value {
    let w=&ctx.world;
    let mut body_roots=Vec::new();let mut shape_roots=Vec::new();
    actor_refs("primary",&ctx.character.primary,&mut body_roots,&mut shape_roots);
    actor_refs("child",&ctx.character.child,&mut body_roots,&mut shape_roots);
    let mut selected_bodies:BTreeSet<H>=body_roots.iter().map(|(_,h)|*h).filter(|&h|h!=NONE).collect();
    if primary!=NONE {selected_bodies.insert(primary);}
    let mut selected_shapes:BTreeSet<H>=shape_roots.iter().map(|(_,h)|*h).filter(|&h|h!=NONE).collect();
    let mut world_bodies=BTreeSet::new();let mut h=w.body_list;let mut world_bodies_complete=true;
    while h!=NONE {
        if h as usize>=w.bodies.items.len() || !world_bodies.insert(h) {world_bodies_complete=false;break;}
        h=w.body(h).next;
    }
    let mut contacts=Vec::new();let mut seen_contacts=BTreeSet::new();let mut h=w.contact_list;
    let mut world_contacts_complete=true;
    while h!=NONE {
        if h as usize>=w.contacts.items.len() || !seen_contacts.insert(h) {world_contacts_complete=false;break;}
        let c=w.contact(h);let mut endpoints=[NONE;2];let mut readable=true;
        for (i,s) in [c.shape1,c.shape2].iter().copied().enumerate() {
            if s==NONE || s as usize>=w.shapes.items.len() {readable=false;}else{endpoints[i]=w.shape(s).body;}
        }
        if !readable {world_contacts_complete=false;}
        if readable && primary!=NONE && endpoints.contains(&primary) {
            selected_shapes.insert(c.shape1);selected_shapes.insert(c.shape2);
            for b in endpoints {if b!=NONE {selected_bodies.insert(b);}}
            let (toi,toi_bits)=raw_f64(c.toi);let (normal,normal_bits)=raw_pair([c.manifold.normal.x,c.manifold.normal.y]);
            contacts.push(json!({"handle":h,"arena_alive":w.contacts.is_alive(h),"retained":w.contacts.retained[h as usize],
                "kind":c.kind,"flags":c.flags,"shapes":[c.shape1,c.shape2],"bodies":endpoints,
                "manifold_count":c.manifold_count,"point_count":c.manifold.point_count,
                "normal":normal,"normal_bits":normal_bits,"toi":toi,"toi_bits":toi_bits}));
        }
        h=c.next;
    }
    let mut bodies=Vec::new();let mut body_shape_chains_complete=true;let mut chains=Vec::new();
    for &h in &selected_bodies {
        let refs:Vec<_>=body_roots.iter().filter(|(_,b)|*b==h).map(|(name,_)|name).collect();
        if h as usize>=w.bodies.items.len() {body_shape_chains_complete=false;bodies.push(json!({"handle":h,"readable":false,"direct_actor_refs":refs}));continue;}
        let b=w.body(h);let mut sh=b.shape_list;let mut chain=BTreeSet::new();let mut ordered=Vec::new();let mut complete=true;
        while sh!=NONE {
            if sh as usize>=w.shapes.items.len() || !chain.insert(sh) {complete=false;break;}
            let s=w.shape(sh);ordered.push(sh);
            if s.body!=h {complete=false;}
            // Ground geometry is represented only for selected contact shapes.
            if h!=ctx.level.level_body {selected_shapes.insert(sh);}
            sh=s.next;
        }
        body_shape_chains_complete &= complete;chains.push((h,chain,complete));
        let (c0,c0_bits)=raw_pair([b.sweep.c0.x,b.sweep.c0.y]);let (c,c_bits)=raw_pair([b.sweep.c.x,b.sweep.c.y]);
        let (v,v_bits)=raw_pair([b.linear_velocity.x,b.linear_velocity.y]);let (pos,pos_bits)=raw_pair([b.xf.position.x,b.xf.position.y]);
        let (r1,r1_bits)=raw_pair([b.xf.r.col1.x,b.xf.r.col1.y]);let (r2,r2_bits)=raw_pair([b.xf.r.col2.x,b.xf.r.col2.y]);
        let mut facts=json!({"handle":h,"readable":true,"direct_actor_refs":refs,"arena_alive":w.bodies.is_alive(h),
            "retained":w.bodies.retained[h as usize],
            "world_list_member":if world_bodies.contains(&h) {Some(true)}else if world_bodies_complete {Some(false)}else{None},
            "type":b.type_,"dynamic":body::is_dynamic(ctx,h),"frozen":body::is_frozen(ctx,h),"flags":b.flags,
            "user_data":b.user_data,"shape_list":b.shape_list,"shape_count":b.shape_count,
            "shape_chain":ordered,"shape_chain_complete":complete,"c0":c0,"c0_bits":c0_bits,"c":c,"c_bits":c_bits,
            "v":v,"v_bits":v_bits,"xf_position":pos,"xf_position_bits":pos_bits,
            "xf_rotation_columns":[r1,r2],"xf_rotation_columns_bits":[r1_bits,r2_bits]});
        for (name,n) in [("a0",b.sweep.a0),("a",b.sweep.a),("t0",b.sweep.t0),("w",b.angular_velocity),
            ("mass",b.mass),("inv_mass",b.inv_mass),("inertia",b.i),("inv_inertia",b.inv_i)] {
            let (value,bits)=raw_f64(n);facts[name]=value;facts[format!("{name}_bits")]=json!(bits);
        }
        bodies.push(facts);
    }
    let mut shapes=Vec::new();let mut selected_shapes_readable=true;
    for h in selected_shapes {
        let refs:Vec<_>=shape_roots.iter().filter(|(_,s)|*s==h).map(|(name,_)|name).collect();
        if h as usize>=w.shapes.items.len() {selected_shapes_readable=false;shapes.push(json!({"handle":h,"readable":false,"direct_actor_refs":refs}));continue;}
        let s=w.shape(h);let (centroid,centroid_bits)=raw_pair([s.centroid.x,s.centroid.y]);
        let chain_member=chains.iter().find(|(b,_,_)|*b==s.body).and_then(|(_,chain,complete)| {
            if chain.contains(&h) {Some(true)}else if *complete {Some(false)}else{None}
        });
        let mut facts=json!({"handle":h,"readable":true,"direct_actor_refs":refs,"arena_alive":w.shapes.is_alive(h),
            "retained":w.shapes.retained[h as usize],"body":s.body,"next":s.next,
            "in_selected_body_shape_chain":chain_member,
            "type":s.type_,"handlers":s.handlers,"material":s.material,"id_tag":s.id_tag,"user_data":s.user_data,
            "filter":{"category":s.filter.category_bits,"mask":s.filter.mask_bits,"group":s.filter.group_index},
            "sensor":s.is_sensor.get(),"proxy_id":s.proxy_id,"centroid":centroid,"centroid_bits":centroid_bits});
        for (name,n) in [("density",s.density),("friction",s.friction),("restitution",s.restitution)] {
            let (value,bits)=raw_f64(n);facts[name]=value;facts[format!("{name}_bits")]=json!(bits);
        }
        shapes.push(facts);
    }
    json!({"phase":"before_input_and_step_after_previous_end_frame","primary":actor_context(&ctx.character.primary),
        "child":actor_context(&ctx.character.child),"child_active":ctx.character.child_active.get(),"child_present":ctx.character.child_present.get(),
        "focus":ctx.level.focus,"level_dead":ctx.level.dead.get(),"world_locked":w.lock.get(),"level_body":ctx.level.level_body,
        "world_bodies_complete":world_bodies_complete,"world_contacts_complete":world_contacts_complete,
        "body_shape_chains_complete":body_shape_chains_complete,"selected_shapes_readable":selected_shapes_readable,
        "bodies":bodies,"shapes":shapes,"primary_contacts":contacts,
        "arena_allocations_aggregate_not_per_slot_generation":{"bodies":w.bodies.allocations,"shapes":w.shapes.allocations,"contacts":w.contacts.allocations},
        "actor_reference_and_contact_membership_are_not_force_or_historical_ownership_certificates":true})
}

/// These are byte-slot reads, not an arena lifetime or completed-creation test.
/// Pending absence is unknown if its declared length exceeds the fixed array.
fn raw_grip_slot<T: Copy, const N: usize>(arena: &hw_core::arena::Arena<T,N>, h: H) -> Value {
    let n=arena.pending_len as usize;
    let pending=arena.pending.iter().take(n).any(|&p|p==h);
    json!({"handle":h,"slot_in_bounds":arena.items.get(h as usize).is_some(),
        "arena_alive":arena.alive.get(h as usize).map(|&v|v!=0),
        "retained":arena.retained.get(h as usize),
        "pending":if pending{Some(true)}else if n<=N{Some(false)}else{None},
        "pending_scan_complete":n<=N})
}

struct RawGripWalk {
    ordered: Vec<H>,
    complete: bool,
    termination: &'static str,
}
impl RawGripWalk {
    fn member(&self,h:H)->Option<bool> {
        if self.ordered.contains(&h){Some(true)}else if self.complete{Some(false)}else{None}
    }
}

/// World handles and encoded edge handles have different capacities. Preserve
/// order: IsConnected short-circuits at the FIRST matching edge, even when a
/// later matching joint has a different collideConnected value.
fn raw_grip_walk(head:H,capacity:usize,next:impl Fn(H)->Option<H>)->RawGripWalk {
    let mut ordered=Vec::new();let mut seen=BTreeSet::new();let mut h=head;
    while h!=NONE {
        let termination=if seen.contains(&h){Some("cycle")}
            else if ordered.len()>=capacity{Some("capacity")}
            else{None};
        if let Some(termination)=termination{return RawGripWalk {ordered,complete:false,termination};}
        let Some(n)=next(h) else{return RawGripWalk {ordered,complete:false,termination:"unreadable_slot"};};
        seen.insert(h);ordered.push(h);h=n;
    }
    RawGripWalk {ordered,complete:true,termination:"none"}
}

fn raw_grip_edge(ctx:&State,h:H)->Option<(H,u32,&hw_core::b2::state::Joint<f64>,hw_core::b2::state::Edge)> {
    use hw_core::b2::state::{edge_owner,edge_node};
    if h==NONE{return None;}
    let owner=edge_owner(h);let node=edge_node(h);let joint=ctx.world.joints.items.get(owner as usize)?;
    Some((owner,node,joint,if node==0{joint.node1}else{joint.node2}))
}
fn raw_grip_body_walk(ctx:&State,h:H)->RawGripWalk {
    match ctx.world.bodies.items.get(h as usize) {
        Some(b)=>raw_grip_walk(b.joint_list,2*hw_core::b2::state::MAX_JOINTS,
            |e|raw_grip_edge(ctx,e).map(|(_,_,_,edge)|edge.next)),
        None=>RawGripWalk {ordered:Vec::new(),complete:false,termination:"unreadable_body_slot"},
    }
}

fn raw_grip_first_match(ctx:&State,source:H,target:H,walk:&RawGripWalk)->Value {
    let first=walk.ordered.iter().find_map(|&h| {
        let (owner,_,joint,edge)=raw_grip_edge(ctx,h)?;
        (edge.other==target).then_some((h,owner,joint.collide_connected))
    });
    // Equality to FALSE matches body::is_connected, including a noncanonical
    // Flag32 word. get()==false alone would lose that distinction.
    let result=first.map(|(_,_,flag)|flag==hw_core::Flag32::FALSE)
        .or_else(||walk.complete.then_some(false));
    json!({"source_body":source,"target_body":target,"source_IsConnected_first_match_result":result,
        "first_matching_edge":first.map(|v|v.0),"first_matching_joint":first.map(|v|v.1),
        "first_match_collide_connected":first.map(|v|v.2.get()),
        "first_match_collide_connected_word":first.map(|v|v.2.word()),
        "evaluated_prefix_complete":first.is_some()||walk.complete,
        "source_body_slot_in_bounds":ctx.world.bodies.items.get(source as usize).is_some(),
        "target_body_slot_in_bounds":ctx.world.bodies.items.get(target as usize).is_some(),
        "scope":"source direct-joint predicate only; not whole contact eligibility, force, or historical ownership"})
}

fn raw_grip_body(ctx:&State,h:H,walk:&RawGripWalk,world_bodies:&BTreeSet<H>,complete:bool)->Value {
    let mut row=raw_grip_slot(&ctx.world.bodies,h);
    row["world_body_list_member"]=json!(if world_bodies.contains(&h){Some(true)}else if complete{Some(false)}else{None});
    row["joint_edge_chain_complete"]=json!(walk.complete);
    row["joint_edge_chain_termination"]=json!(walk.termination);
    row["joint_edge_walk_capacity"]=json!(2*hw_core::b2::state::MAX_JOINTS);
    row["joint_edge_head"]=json!(ctx.world.bodies.items.get(h as usize).map(|b|b.joint_list));
    if let Some(b)=ctx.world.bodies.items.get(h as usize) {
        row["type"]=json!(b.type_);row["flags"]=json!(b.flags);row["user_data"]=json!(b.user_data);
    }
    let mut previous=NONE;
    let edges:Vec<_>=walk.ordered.iter().filter_map(|&edge_handle| {
        let (owner,node,joint,edge)=raw_grip_edge(ctx,edge_handle)?;
        let (endpoint,opposite)=if node==0{(joint.body1,joint.body2)}else{(joint.body2,joint.body1)};
        let row=json!({"edge_handle":edge_handle,"joint_handle":owner,"node":node,
            "other":edge.other,"prev":edge.prev,"next":edge.next,
            "node_endpoint_matches_body":endpoint==h,"other_matches_opposite_endpoint":edge.other==opposite,
            "prev_matches_walk_predecessor":edge.prev==previous,
            "collide_connected":joint.collide_connected.get(),"collide_connected_word":joint.collide_connected.word()});
        previous=edge_handle;Some(row)
    }).collect();
    row["ordered_joint_edges"]=json!(edges);row
}

fn raw_grip_shape(ctx:&State,h:H,world_bodies:&BTreeSet<H>,complete:bool)->Value {
    let mut row=raw_grip_slot(&ctx.world.shapes,h);
    if let Some(s)=ctx.world.shapes.items.get(h as usize) {
        row["current_shape_body"]=json!(s.body);
        let mut body=raw_grip_slot(&ctx.world.bodies,s.body);
        body["world_body_list_member"]=json!(if world_bodies.contains(&s.body){Some(true)}else if complete{Some(false)}else{None});
        row["current_shape_body_slot"]=body;
        row["handlers"]=json!(s.handlers);
        row["sensor"]=json!(s.is_sensor.get());row["id_tag"]=json!(s.id_tag);
        row["filter"]=json!({"category":s.filter.category_bits,"mask":s.filter.mask_bits,"group":s.filter.group_index});
    }
    row
}

fn raw_grip_buffer(ctx:&State,actor:&hw_core::game::character::CharacterState<f64>,part:usize,
    world_bodies:&BTreeSet<H>,complete:bool)->Value {
    let shape=actor.shapes[part];let key=hw_core::game::damage::key(shape);
    let result=actor.contact_results.get(key).map(|r| {
        let (impulse,impulse_bits)=raw_f64(r.impulse);
        json!({"present":r.present.get(),"present_word":r.present.word(),"shape":r.shape,"other":r.other,
            "impulse":impulse,"impulse_bits":impulse_bits,
            "stored_shape_current_slot":raw_grip_shape(ctx,r.shape,world_bodies,complete),
            "stored_other_current_slot":raw_grip_shape(ctx,r.other,world_bodies,complete)})
    });
    let threshold=actor.contact_thresholds.get(key).map(|&v| {let (value,bits)=raw_f64(v);json!({"value":value,"bits":bits})});
    json!({"lookup_current_actor_shape":shape,"lookup_key":key,"key_in_bounds":key<actor.contact_results.len(),
        "none_shape_uses_sentinel_key":shape==NONE,
        "current_actor_shape_slot":raw_grip_shape(ctx,shape,world_bodies,complete),
        "threshold":threshold,"contact_add":actor.contact_adds.get(key).map(|v|v.get()),
        "contact_add_word":actor.contact_adds.get(key).map(|v|v.word()),"result":result,
        "body_resolution_is_current_not_contact_time":true,
        "buffer_has_no_contact_time_body_point_normal_or_timestamp":true})
}

/// Host-only immutable snapshots. No gameplay getters, actor swaps, collision
/// queries, lab watches, cache consumption, or source numeric operations occur.
fn raw_grip_context(ctx:&State,phase:&str)->Value {
    use hw_core::b2::state::{edge_handle,MAX_JOINTS};
    use hw_core::game::character::{LOWER_ARM1,LOWER_ARM2,SHOULDER1,SHOULDER2,ELBOW1,ELBOW2};
    let w=&ctx.world;
    let (world_bodies,bodies_complete)=bounded_handles(w.body_list,w.bodies.items.len(),|h|w.bodies.items.get(h as usize).map(|b|b.next));
    let world_joints=raw_grip_walk(w.joint_list,MAX_JOINTS,|h|w.joints.items.get(h as usize).map(|j|j.next));
    let mut refs=Vec::new();let mut joints=BTreeSet::new();let mut walks=BTreeMap::new();let mut arms=Vec::new();
    let mut actors=Vec::new();
    for (name,actor) in [("primary",&ctx.character.primary),("child",&ctx.character.child)] {
        let mut lower_arms=Vec::new();
        for (side,part,shoulder,elbow) in [(0,LOWER_ARM1,SHOULDER1,ELBOW1),(1,LOWER_ARM2,SHOULDER2,ELBOW2)] {
            let b=actor.bodies[part];arms.push((name,part,b));
            let walk=walks.entry(b).or_insert_with(||raw_grip_body_walk(ctx,b));
            for &e in &walk.ordered {if let Some((j,_,_,_))=raw_grip_edge(ctx,e){joints.insert(j);}}
            for (label,h) in [(format!("{name}.grip_joints[{side}]"),actor.grip_joints[side]),
                (format!("{name}.joints[{shoulder}]"),actor.joints[shoulder]),
                (format!("{name}.joints[{elbow}]"),actor.joints[elbow])] {
                if h!=NONE{joints.insert(h);}refs.push((label,h));
            }
            lower_arms.push(json!({"part_index":part,"body_ref":b,"shape_ref":actor.shapes[part],
                "stored_grip_joint":actor.grip_joints[side],"cached_shoulder_joint":actor.joints[shoulder],
                "cached_elbow_joint":actor.joints[elbow],
                "pending_contact_result":raw_grip_buffer(ctx,actor,part,&world_bodies,bodies_complete)}));
        }
        actors.push(json!({"name":name,"kind":actor.kind,"dead":actor.dead.get(),"grabbing":actor.grabbing.get(),
            "lower_arms":lower_arms}));
    }
    // Endpoint chains are a single bounded expansion, not recursive discovery.
    for &h in &joints {if let Some(j)=w.joints.items.get(h as usize) {
        for b in [j.body1,j.body2] {walks.entry(b).or_insert_with(||raw_grip_body_walk(ctx,b));}
    }}
    let mut targets=vec![(ctx.level.level_body,vec!["current_level_body_ref".to_owned()])];
    for (i,special) in ctx.level.specials.iter().enumerate().filter(|(_,s)|s.kind==hw_data::SpecialKind::FinishLine) {
        if let Some(shape)=w.shapes.items.get(special.shape as usize) {
            if shape.body==NONE{continue;}
            let role=format!("finish.specials[{i}].shape.current_body");
            if let Some((_,roles))=targets.iter_mut().find(|(b,_)|*b==shape.body) {roles.push(role);}
            else{targets.push((shape.body,vec![role]));}
        }
    }
    for &(b,_) in &targets {walks.entry(b).or_insert_with(||raw_grip_body_walk(ctx,b));}
    let finish_shapes:Vec<_>=ctx.level.specials.iter().enumerate().filter(|(_,s)|s.kind==hw_data::SpecialKind::FinishLine)
        .map(|(i,s)|json!({"special_index":i,"current_shape_ref":s.shape,
            "current_shape_slot":raw_grip_shape(ctx,s.shape,&world_bodies,bodies_complete)})).collect();
    let mut joint_rows=Vec::new();
    for h in joints {
        let mut row=raw_grip_slot(&w.joints,h);
        row["world_joint_list_member"]=json!(world_joints.member(h));
        row["stored_actor_refs"]=json!(refs.iter().filter(|(_,j)|*j==h).map(|(name,_)|name).collect::<Vec<_>>());
        if let Some(j)=w.joints.items.get(h as usize) {
            row["body1"]=json!(j.body1);row["body2"]=json!(j.body2);row["type"]=json!(j.type_);
            row["collide_connected"]=json!(j.collide_connected.get());row["collide_connected_word"]=json!(j.collide_connected.word());
            row["broken"]=json!(j.broken.get());row["broken_word"]=json!(j.broken.word());
            row["prev"]=json!(j.prev);row["next"]=json!(j.next);
            for (label,node,owner,edge) in [("node1",0,j.body1,j.node1),("node2",1,j.body2,j.node2)] {
                let encoded=edge_handle(h,node);
                row[label]=json!({"edge_handle":encoded,"other":edge.other,"prev":edge.prev,"next":edge.next,
                    "in_endpoint_body_edge_chain":walks.get(&owner).and_then(|walk|walk.member(encoded))});
            }
        }
        joint_rows.push(row);
    }
    let body_rows:Vec<_>=walks.iter().map(|(&h,walk)|raw_grip_body(ctx,h,walk,&world_bodies,bodies_complete)).collect();
    let mut connectivity=Vec::new();
    for &(name,part,b) in &arms {for (target,roles) in &targets {
        connectivity.push(json!({"actor":name,"part_index":part,"target_body":target,"target_roles":roles,
            "arm_to_target_body":raw_grip_first_match(ctx,b,*target,&walks[&b]),
            "target_body_to_arm":raw_grip_first_match(ctx,*target,b,&walks[target])}));
    }}
    json!({"schema":1,"phase":phase,"scope":"primary and child lower-arm stored refs, one-hop joint edges and current Result dictionary slots",
        "primary_kind":ctx.character.primary.kind,"child_present":ctx.character.child_present.get(),
        "child_active":ctx.character.child_active.get(),"world_locked":w.lock.get(),"current_level_body_ref":ctx.level.level_body,
        "world_body_chain_complete":bodies_complete,"world_joint_walk_capacity":MAX_JOINTS,
        "world_joint_chain_complete":world_joints.complete,"world_joint_chain_termination":world_joints.termination,
        "ordered_world_joint_handles":world_joints.ordered,"actors":actors,"joints":joint_rows,"bodies":body_rows,
        "directional_target_body_connectivity":connectivity,"finish_current_shape_refs":finish_shapes,
        "identity_is_snapshot_local_no_per_slot_generation":true,
        "stored_refs_arena_lifetime_world_membership_and_edge_reachability_are_separate":true,
        "absence_on_incomplete_walk_is_unknown":true,
        "pre_post_snapshots_do_not_prove_intrastep_creation_consumption_or_handle_reuse":true,
        "joint_presence_is_not_force_or_completed_creation_certificate":true,
        "connectivity_is_direct_joint_predicate_not_whole_contact_eligibility":true})
}

fn raw_attach_grip_context(line:&mut Value,enabled:bool,before:Value,ctx:&State) {
    if !enabled{return;}
    line["grip_context_before_input"]=before;
    line["grip_context_after_end_frame"]=raw_grip_context(ctx,"after_end_frame");
}

/// Fresh finite replay with a raw ordered view of the existing lab hooks. Only
/// the primary chest captured before each armed step is watched. Global hook
/// events (for example Finish or ChestSmash) remain unfiltered and are not an
/// ownership certificate. This does not expose contact-cache or pre-Advance
/// sweep snapshots that the existing core hooks did not record.
pub fn run_raw_primary(level: u32, character: u32, masks: &[u8], from: u32, to: u32) -> Result<(), String> {
    run_raw_primary_options(level,character,masks,from,to,false,false)
}
pub fn run_raw_primary_toi_target(level:u32,character:u32,masks:&[u8],from:u32,to:u32)->Result<(),String>{
    run_raw_primary_options(level,character,masks,from,to,true,false)
}
pub fn run_raw_primary_grip_context(level:u32,character:u32,masks:&[u8],from:u32,to:u32,probe:bool)->Result<(),String>{
    run_raw_primary_options(level,character,masks,from,to,probe,true)
}
fn run_raw_primary_options(level:u32,character:u32,masks:&[u8],from:u32,to:u32,probe:bool,grip:bool)->Result<(),String>{
    let mut sim = Sim::new(level, character)?;
    sim.replay = hw_core::game::session::ReplayState::new(masks.len() as u32);
    sim.masks = masks.to_vec();
    for frame in 1..=to {
        let primary_before = sim.ctx.character.primary.bodies[hw_core::game::character::CHEST];
        let focus_before = sim.ctx.level.focus;
        let iteration_before = sim.replay.iteration;
        let input_allowed_before = sim.replay.input_allowed.get();
        let primary_dead_before = sim.ctx.character.primary.dead.get();
        let world_frame_before = sim.frame();
        let armed = frame >= from;
        let grip_before=if armed && grip {raw_grip_context(&sim.ctx,"before_input")}else{Value::Null};
        let context_before=if armed {raw_context(&sim.ctx,primary_before)}else{Value::Null};
        let pair_inventory_before=if armed {raw_pair_inventory(&sim.ctx,primary_before,level,"before_input")}else{Value::Null};
        let pair_identities_before=if armed {pair_shape_identities(&sim.ctx,"before_input_current_slot",&[])}else{BTreeMap::new()};
        let mut target_eligibility=Value::Null;
        if armed {
            if primary_before == NONE { lab::arm(&[]); }
            else { lab::arm(&[primary_before]); }
            if probe {
                let (target,eligibility)=raw_toi_target_eligibility(&sim.ctx,primary_before,level);
                target_eligibility=eligibility;
                if let Some(target)=target {lab::configure_toi_target(target);}
            }
        }
        let f = sim.step(0);
        if armed {
            lab::disarm();
            let events = lab::take_events();
            let pair_exhaustion=lab::take_pair_exhaustion();
            let pair_exhaustion_resolution=failed_pair_resolution(&sim.ctx,&pair_exhaustion,&pair_identities_before);
            let pair_inventory_after=raw_pair_inventory(&sim.ctx,primary_before,level,"after_end_frame");
            let mut line = json!({"schema":1,"mode":"raw_primary_lab_events","level":level,"character":character,
                "frame":frame,"planned_entries":masks.len(),"input":super::sim::tas_text(&[f.mask]),
                "world_frame_before":world_frame_before,"world_frame_after":f.frame,
                "primary_handle_before":primary_before,"primary_handle_after":sim.ctx.character.primary.bodies[hw_core::game::character::CHEST],
                "focus_before":focus_before,"focus_after":sim.ctx.level.focus,
                "primary_dead_before":primary_dead_before,"primary_dead_after":sim.ctx.character.primary.dead.get(),
                "iteration_before":iteration_before,"iteration_after":sim.replay.iteration,
                "input_allowed_before":input_allowed_before,"input_allowed_after":sim.replay.input_allowed.get(),
                "site":f.site,"port_abort":f.port_abort,"lagging":f.lagging,"locked_after":sim.ctx.world.lock.get(),
                "actual_finished":f.finished,"finish_frame":sim.replay.finish_frame,
                "event_count":events.len(),"events":raw_events(&events),
                "context_before":context_before,
                "pair_exhaustion":raw_pair_exhaustion(pair_exhaustion),
                "pair_exhaustion_current_slot_resolution":pair_exhaustion_resolution,
                "pair_inventory_before":pair_inventory_before,"pair_inventory_after":pair_inventory_after,
                "nonfinite_numeric_values_are_null_with_exact_bits":true,
                "global_hook_events_are_not_primary_ownership_certificates":true});
            raw_attach_toi_target(&mut line,probe,target_eligibility);
            raw_attach_grip_context(&mut line,grip,grip_before,&sim.ctx);
            println!("{line}");
        }
    }
    Ok(())
}

fn raw_toi_target_body(b:Option<lab::ToiTargetBody>)->Value{
    b.map(|b|json!({"handle":b.handle,"arena_alive":b.alive,"retained":b.retained,
        "flags":b.flags,"type":b.body_type,"words_hi_lo_u32":b.words})).unwrap_or(Value::Null)
}
fn raw_attach_toi_target(line:&mut Value,probe:bool,eligibility:Value){
    if !probe{return;}
    let mut capture=raw_toi_target(lab::take_toi_target());capture["eligibility"]=eligibility;
    line["toi_target_probe"]=capture;
}
fn raw_toi_target_eligibility(ctx:&State,primary:H,level:u32)->(Option<lab::ToiTarget>,Value){
    use hw_core::b2::state::{E_DYNAMIC_TYPE,E_STATIC_TYPE,B2_INVALID};
    let fail=|reason:&str|(None,json!({"configured":false,"unavailable_reason":reason,
        "scope":"pre-step current live slots and complete bounded membership only"}));
    if level!=45 || ctx.character.primary.kind!=3 {return fail("unsupported_level_or_character");}
    if primary==NONE || ctx.character.primary.bodies[hw_core::game::character::CHEST]!=primary
        || ctx.character.primary.dead.get() || ctx.level.focus!=primary {return fail("primary_root_dead_absent_or_focus_mismatch");}
    let w=&ctx.world;let chest=ctx.character.primary.shapes[hw_core::game::character::CHEST];
    let ceiling=2;let ground=ctx.level.level_body;
    let alive_body=|h:H|w.bodies.alive.get(h as usize).is_some_and(|&a|a!=0);
    let alive_shape=|h:H|w.shapes.alive.get(h as usize).is_some_and(|&a|a!=0);
    if !alive_body(primary) || !alive_body(ground) || !alive_shape(chest) || !alive_shape(ceiling){return fail("required_arena_slot_not_alive");}
    let (Some(p),Some(g),Some(cs),Some(gs))=(w.bodies.items.get(primary as usize),w.bodies.items.get(ground as usize),
        w.shapes.items.get(chest as usize),w.shapes.items.get(ceiling as usize)) else{return fail("required_slot_unreadable");};
    if p.type_!=E_DYNAMIC_TYPE || g.type_!=E_STATIC_TYPE || cs.body!=primary || gs.body!=ground{return fail("body_type_or_shape_owner_mismatch");}
    let (bodies,body_complete)=bounded_handles(w.body_list,w.bodies.items.len(),|h|
        if alive_body(h){w.bodies.items.get(h as usize).map(|b|b.next)}else{None});
    if !body_complete{return fail("world_body_chain_incomplete");}
    if !bodies.contains(&primary) || !bodies.contains(&ground){return fail("required_body_not_in_current_world");}
    for (owner,head,shape) in [(primary,p.shape_list,chest),(ground,g.shape_list,ceiling)]{
        let (shapes,complete)=bounded_handles(head,w.shapes.items.len(),|h|
            if alive_shape(h){w.shapes.items.get(h as usize).filter(|s|s.body==owner).map(|s|s.next)}else{None});
        if !complete{return fail("required_body_shape_chain_incomplete");}
        if !shapes.contains(&shape){return fail("required_shape_not_in_current_body_chain");}
    }
    if !fixed_box_matches(ctx,ceiling,false,[-4.0,-84.0,324.0,-80.0]){return fail("world_ceiling_exact_geometry_unavailable");}
    for (h,s) in [(chest,cs),(ceiling,gs)]{
        if !w.broad_phase.proxy_pool.get(s.proxy_id as usize).is_some_and(|p|
            p.overlap_count!=B2_INVALID && p.shape==h){return fail("required_proxy_slot_invalid_or_back_reference_mismatch");}
    }
    (Some(lab::ToiTarget {primary,chest_shape:chest,ceiling_shape:ceiling,before_commit:true}),
        json!({"configured":true,"unavailable_reason":Value::Null,"primary":primary,"chest_shape":chest,
            "ceiling_shape":ceiling,"level_body":ground,"live_arena_and_complete_current_membership":true,
            "scope":"pre-step current slots only; no generation or historical identity claim"}))
}
fn raw_toi_target_contact(c:Option<lab::ToiTargetContact>)->Value{
    c.map(|c|json!({"handle":c.handle,"arena_alive":c.alive,"retained":c.retained,
        "shapes":c.shapes,"flags":c.flags,"manifold_count":c.manifold_count,
        "point_count":c.point_count,"cached_toi_words_hi_lo_u32":c.toi_words})).unwrap_or(Value::Null)
}
fn raw_toi_target_snapshot(s:lab::ToiTargetSnapshot)->Value {
    let slots=json!({"shape_readable":s.shape_readable,"shape_alive":s.shape_alive,
        "shape_retained":s.shape_retained,"shape_bodies":s.shape_bodies,
        "bodies":s.bodies.map(raw_toi_target_body),"proxies":s.proxies,
        "proxy_readable":s.proxy_readable,"proxy_valid_slot":s.proxy_valid_slot,
        "proxy_back_reference_matches":s.proxy_back_reference_matches});
    let pair=json!({"lookup_complete":s.pair_lookup_complete,"index":s.pair_index,
        "status":s.pair_status,"contact_handle":s.pair_contact,
        "contact":raw_toi_target_contact(s.contact),"global_pairs":s.pairs,
        "contact_world_membership":s.contact_world_membership,
        "target_contact_membership_lookup_complete":s.target_contact_membership_lookup_complete,
        "global_buffer_count":s.buffered,"free_pair":s.free});
    json!({"slots":slots,"pair":pair})
}
fn raw_toi_target(c:lab::ToiTargetCapture)->Value{
    use lab::ToiTargetRecord::*;
    let records:Vec<_>=c.records.iter().take(c.len).flatten().map(|r|match r{
        Snapshot {scan,events_before,stage,preceding_selected,selected_toi_words,value:s}=>{
            json!({"kind":"snapshot","scan":scan,"events_before":events_before,
                "stage":format!("{stage:?}"),"preceding_selected_contact":preceding_selected,
                "selected_toi_words_hi_lo_u32":selected_toi_words,"target_snapshot":raw_toi_target_snapshot(*s)})
        },
        Branch {scan,events_before,contact,bodies,branch,tested_toi_words,minimum_before_words}=>
            json!({"kind":"branch","scan":scan,"events_before":events_before,
                "branch":format!("{branch:?}"),"contact":raw_toi_target_contact(Some(*contact)),
                "bodies":(*bodies).map(|b|raw_toi_target_body(b)),
                "tested_toi_words_hi_lo_u32":tested_toi_words,
                "minimum_before_words_hi_lo_u32":minimum_before_words}),
        ScanEnd {scan,events_before,selected,minimum_words,original_done_branch}=>
            json!({"kind":"scan_end","scan":scan,"events_before":events_before,
                "selected_contact":selected,"minimum_words_hi_lo_u32":minimum_words,
                "original_done_branch":original_done_branch}),
        SelectedBeforeAdvance {scan,events_before,contact,bodies,selected_toi_words}=>
            json!({"kind":"selected_before_advance","scan":scan,"events_before":events_before,
                "contact":raw_toi_target_contact(Some(*contact)),
                "bodies":(*bodies).map(|b|raw_toi_target_body(b)),
                "selected_toi_words_hi_lo_u32":selected_toi_words}),
        Updated {scan,events_before,contact,bodies}=>json!({"kind":"selected_update","scan":scan,
            "events_before":events_before,"contact":raw_toi_target_contact(Some(*contact)),
            "bodies":(*bodies).map(|b|raw_toi_target_body(b))}),
        PrimarySynchronized {scan,events_before,preceding_selected,selected_toi_words,in_range,value:s}=>
            json!({"kind":"primary_synchronized","scan":scan,"events_before":events_before,
                "preceding_selected_contact":preceding_selected,"selected_toi_words_hi_lo_u32":selected_toi_words,
                "original_in_range_result":in_range,"target_snapshot":raw_toi_target_snapshot(*s)}),
    }).collect();
    json!({"target":c.target.map(|t|json!({"primary":t.primary,"chest_shape":t.chest_shape,
        "ceiling_shape":t.ceiling_shape,"before_commit":t.before_commit})),
        "records":records,"attempted":c.attempted,"truncated":c.truncated,
        "count_saturated":c.count_saturated,"last_scan":c.scan,
        "scan_saturated":c.scan_saturated,"last_scan_open":c.scan_open,
        "natural_solve_toi_end_observed":c.natural_toi_end,
        "records_complete":c.records_complete(),
        "record_length_in_fixed_capacity":c.len<=lab::TOI_TARGET_CAPACITY,
        "word_order":["t0","c0.x","c0.y","c.x","c.y","a0","a","v.x","v.y","w"],
        "scope":"fixed intrastep current-slot copies and original branch observations; scan absence requires naturally completed untruncated capture and complete target inventory",
        "historical_ownership_or_collision_omission_claimed":false})
}

#[cfg(test)]
mod raw_tests {
    use super::*;

    fn synthetic_grip_context(test:impl FnOnce(&mut State)+Send+'static) {
        std::thread::Builder::new().stack_size(64<<20).spawn(move|| {
            let mut ctx=Box::new(State::new());test(&mut ctx);
        }).unwrap().join().unwrap();
    }
    fn grip_row<'a>(rows:&'a Value,handle:H)->&'a Value {
        rows.as_array().unwrap().iter().find(|r|r["handle"]==json!(handle)).unwrap()
    }

    #[test]
    fn grip_context_separates_cached_refs_lifetime_world_list_and_endpoint_edges() {
        synthetic_grip_context(|ctx| {
            use hw_core::b2::state::{Body,Joint,edge_handle};
            use hw_core::game::character::LOWER_ARM1;
            let arm=ctx.world.bodies.alloc(Body::blank());let terrain=ctx.world.bodies.alloc(Body::blank());
            let j=ctx.world.joints.alloc(Joint::blank());ctx.world.joint_mut(j).body1=arm;ctx.world.joint_mut(j).body2=terrain;
            ctx.character.primary.bodies[LOWER_ARM1]=arm;ctx.character.primary.grip_joints[0]=j;ctx.level.level_body=terrain;
            ctx.world.joints.retain(j);ctx.world.joints.destroy(j);
            let snapshot=raw_grip_context(ctx,"synthetic");let row=grip_row(&snapshot["joints"],j);
            assert_eq!(row["stored_actor_refs"],json!(["primary.grip_joints[0]"]));
            assert_eq!(row["arena_alive"],json!(false));assert_eq!(row["retained"],json!(1));assert_eq!(row["pending"],json!(true));
            assert_eq!(row["world_joint_list_member"],json!(false));assert_eq!(row["node1"]["in_endpoint_body_edge_chain"],json!(false));
            // An edge can affect the source filter without a World-list entry
            // or a stored grip reference. Neither is upgraded to a force claim.
            ctx.character.primary.grip_joints[0]=NONE;
            ctx.world.body_mut(arm).joint_list=edge_handle(j,0);ctx.world.joint_mut(j).node1.other=terrain;
            let snapshot=raw_grip_context(ctx,"synthetic");let row=grip_row(&snapshot["joints"],j);
            assert_eq!(row["stored_actor_refs"],json!([]));assert_eq!(row["world_joint_list_member"],json!(false));
            assert_eq!(row["node1"]["in_endpoint_body_edge_chain"],json!(true));
            assert_eq!(row["node2"]["in_endpoint_body_edge_chain"],json!(false));
            assert_eq!(snapshot["directional_target_body_connectivity"][0]["arm_to_target_body"]["source_IsConnected_first_match_result"],json!(true));
            assert_eq!(snapshot["directional_target_body_connectivity"][0]["target_body_to_arm"]["source_IsConnected_first_match_result"],json!(false));
        });
    }

    #[test]
    fn grip_connectivity_preserves_first_match_order_in_both_directions() {
        synthetic_grip_context(|ctx| {
            use hw_core::b2::state::{Body,Joint,edge_handle};
            let arm=ctx.world.bodies.alloc(Body::blank());let ground=ctx.world.bodies.alloc(Body::blank());
            let a=ctx.world.joints.alloc(Joint::blank());let b=ctx.world.joints.alloc(Joint::blank());
            for h in [a,b] {let j=ctx.world.joint_mut(h);j.body1=arm;j.body2=ground;j.node1.other=ground;j.node2.other=arm;}
            ctx.world.joint_mut(a).collide_connected=hw_core::Flag32::TRUE;
            ctx.world.joint_mut(a).node1.next=edge_handle(b,0);ctx.world.body_mut(arm).joint_list=edge_handle(a,0);
            ctx.world.joint_mut(b).node2.next=edge_handle(a,1);ctx.world.body_mut(ground).joint_list=edge_handle(b,1);
            let forward=raw_grip_first_match(ctx,arm,ground,&raw_grip_body_walk(ctx,arm));
            let reverse=raw_grip_first_match(ctx,ground,arm,&raw_grip_body_walk(ctx,ground));
            assert_eq!(forward["first_matching_joint"],json!(a));assert_eq!(forward["source_IsConnected_first_match_result"],json!(false));
            assert_eq!(reverse["first_matching_joint"],json!(b));assert_eq!(reverse["source_IsConnected_first_match_result"],json!(true));
            ctx.world.body_mut(arm).joint_list=edge_handle(b,0);ctx.world.joint_mut(b).node1.next=edge_handle(a,0);
            ctx.world.joint_mut(a).node1.next=NONE;
            assert_eq!(raw_grip_first_match(ctx,arm,ground,&raw_grip_body_walk(ctx,arm))["source_IsConnected_first_match_result"],json!(true));
            assert_eq!(forward["first_match_collide_connected_word"],json!(1));
            assert_eq!(reverse["first_match_collide_connected_word"],json!(0));
        });
    }

    #[test]
    fn grip_walks_bound_encoded_edges_and_report_unknown_absence_on_incomplete_prefixes() {
        synthetic_grip_context(|ctx| {
            use hw_core::b2::state::{Body,Joint,edge_handle,MAX_JOINTS};
            let arm=ctx.world.bodies.alloc(Body::blank());let j=ctx.world.joints.alloc(Joint::blank());
            ctx.world.body_mut(arm).joint_list=edge_handle(j,0);ctx.world.joint_mut(j).node1.other=19;
            ctx.world.joint_mut(j).node1.next=edge_handle(j,0);ctx.world.joint_list=j;ctx.world.joint_mut(j).next=j;
            let walk=raw_grip_body_walk(ctx,arm);assert!(!walk.complete);assert_eq!(walk.termination,"cycle");
            assert_eq!(raw_grip_first_match(ctx,arm,19,&walk)["source_IsConnected_first_match_result"],json!(true));
            assert!(raw_grip_first_match(ctx,arm,20,&walk)["source_IsConnected_first_match_result"].is_null());
            assert_eq!(walk.member(edge_handle(j,0)),Some(true));assert_eq!(walk.member(edge_handle(j,1)),None);
            ctx.character.primary.grip_joints[0]=MAX_JOINTS as H;
            let snapshot=raw_grip_context(ctx,"synthetic");let invalid=grip_row(&snapshot["joints"],MAX_JOINTS as H);
            assert_eq!(invalid["slot_in_bounds"],json!(false));assert!(invalid["world_joint_list_member"].is_null());
            assert_eq!(snapshot["world_joint_chain_complete"],json!(false));
            ctx.world.body_mut(arm).joint_list=(MAX_JOINTS as H)*2;
            assert_eq!(raw_grip_body_walk(ctx,arm).termination,"unreadable_slot");
            assert_eq!(raw_grip_body_walk(ctx,NONE).termination,"unreadable_body_slot");
            ctx.world.joint_list=MAX_JOINTS as H;
            assert_eq!(raw_grip_context(ctx,"synthetic")["world_joint_chain_termination"],json!("unreadable_slot"));
            let limited=raw_grip_walk(0,1,|h|Some(h+1));assert_eq!(limited.termination,"capacity");assert_eq!(limited.ordered,[0]);
        });
    }

    #[test]
    fn grip_walk_accepts_all_384_distinct_nodes_of_192_self_joints_without_owner_deduplication() {
        synthetic_grip_context(|ctx| {
            use hw_core::b2::state::{Body,Joint,MAX_JOINTS};
            use hw_core::game::character::LOWER_ARM1;
            let b=ctx.world.bodies.alloc(Body::blank());ctx.character.primary.bodies[LOWER_ARM1]=b;
            ctx.world.body_mut(b).joint_list=0;ctx.world.joint_list=0;
            for i in 0..MAX_JOINTS {
                let h=ctx.world.joints.alloc(Joint::blank());assert_eq!(h,i as H);
                let j=ctx.world.joint_mut(h);j.body1=b;j.body2=b;
                j.next=if i+1==MAX_JOINTS{NONE}else{h+1};j.prev=if i==0{NONE}else{h-1};
                for (node,edge) in [&mut j.node1,&mut j.node2].into_iter().enumerate() {
                    let e=2*i+node;edge.other=b;edge.prev=if e==0{NONE}else{e as H-1};
                    edge.next=if e+1==2*MAX_JOINTS{NONE}else{e as H+1};
                }
            }
            let snapshot=raw_grip_context(ctx,"synthetic");let body=grip_row(&snapshot["bodies"],b);
            assert_eq!(body["joint_edge_chain_complete"],json!(true));assert_eq!(body["ordered_joint_edges"].as_array().unwrap().len(),384);
            assert_eq!(snapshot["world_joint_chain_complete"],json!(true));assert_eq!(snapshot["ordered_world_joint_handles"].as_array().unwrap().len(),192);
            let joint=grip_row(&snapshot["joints"],191);
            assert_eq!(joint["node1"]["edge_handle"],json!(382));assert_eq!(joint["node2"]["edge_handle"],json!(383));
            assert_eq!(joint["node1"]["in_endpoint_body_edge_chain"],json!(true));assert_eq!(joint["node2"]["in_endpoint_body_edge_chain"],json!(true));
            assert!(body["ordered_joint_edges"].as_array().unwrap().iter().all(|e|e["prev_matches_walk_predecessor"]==json!(true)));
        });
    }

    #[test]
    fn grip_results_use_current_actor_shape_key_and_current_body_resolution_even_for_dead_child() {
        synthetic_grip_context(|ctx| {
            use hw_core::b2::state::{Body,Shape};
            use hw_core::game::character::{LOWER_ARM1,LOWER_ARM2};
            let b=ctx.world.bodies.alloc(Body::blank());let s=ctx.world.shapes.alloc(Shape::blank());
            let other=ctx.world.shapes.alloc(Shape::blank());ctx.world.shape_mut(other).body=b;
            ctx.character.child_active=hw_core::Flag32::TRUE;ctx.character.child.dead=hw_core::Flag32::TRUE;
            ctx.character.child.shapes[LOWER_ARM1]=s;
            let result=&mut ctx.character.child.contact_results[s as usize];
            result.present=hw_core::Flag32::TRUE;result.shape=other;result.other=other;
            result.impulse=f64::from_bits(0x7ff8000000005678);
            ctx.character.child.contact_thresholds[s as usize]=-0.0;
            let sentinel=hw_core::game::damage::key(NONE);
            ctx.character.child.contact_results[sentinel].present=hw_core::Flag32::TRUE;
            ctx.character.child.contact_results[sentinel].impulse=f64::INFINITY;
            let snapshot=raw_grip_context(ctx,"synthetic");let child=&snapshot["actors"][1];
            assert_eq!(snapshot["child_active"],json!(true));assert_eq!(child["dead"],json!(true));assert_eq!(child["grabbing"],json!(false));
            let buffer=&child["lower_arms"][0]["pending_contact_result"];
            assert_eq!(buffer["lookup_key"],json!(s));assert_eq!(buffer["result"]["shape"],json!(other));
            assert_eq!(buffer["result"]["impulse_bits"],json!("7ff8000000005678"));assert!(buffer["result"]["impulse"].is_null());
            assert_eq!(buffer["threshold"]["bits"],json!("8000000000000000"));
            assert_eq!(buffer["result"]["stored_other_current_slot"]["current_shape_body"],json!(b));
            ctx.world.shape_mut(other).body=NONE;
            let changed=raw_grip_context(ctx,"synthetic");
            assert_eq!(changed["actors"][1]["lower_arms"][0]["pending_contact_result"]["result"]["stored_other_current_slot"]["current_shape_body"],json!(NONE));
            assert_eq!(child["lower_arms"][1]["pending_contact_result"]["lookup_key"],json!(sentinel));
            assert_eq!(child["lower_arms"][1]["pending_contact_result"]["result"]["impulse_bits"],json!("7ff0000000000000"));
            ctx.character.child.shapes[LOWER_ARM2]=u32::MAX-1;
            let invalid=raw_grip_context(ctx,"synthetic");let buffer=&invalid["actors"][1]["lower_arms"][1]["pending_contact_result"];
            assert_eq!(buffer["key_in_bounds"],json!(false));assert!(buffer["result"].is_null());
            assert_eq!(ctx.character.child_active.word(),1);assert!(ctx.character.child.contact_results[s as usize].present.get());
        });
    }

    #[test]
    fn grip_slot_pending_overflow_and_free_bytes_are_not_lifetime_certificates() {
        synthetic_grip_context(|ctx| {
            let h=1;ctx.world.joints.pending_len=193;
            let row=raw_grip_slot(&ctx.world.joints,h);
            assert_eq!(row["slot_in_bounds"],json!(true));assert_eq!(row["arena_alive"],json!(false));
            assert_eq!(row["retained"],json!(0));assert!(row["pending"].is_null());assert_eq!(row["pending_scan_complete"],json!(false));
            ctx.world.joints.pending[191]=h;
            assert_eq!(raw_grip_slot(&ctx.world.joints,h)["pending"],json!(true));
            assert_eq!(raw_grip_slot(&ctx.world.joints,NONE)["slot_in_bounds"],json!(false));
        });
    }

    #[test]
    fn grip_connectivity_resolves_deduplicated_finish_bodies_without_assuming_terrain_owner() {
        synthetic_grip_context(|ctx| {
            use hw_core::b2::state::{Body,Shape,Joint,edge_handle};
            use hw_core::game::character::LOWER_ARM1;
            let arm=ctx.world.bodies.alloc(Body::blank());let terrain=ctx.world.bodies.alloc(Body::blank());
            let finish_body=ctx.world.bodies.alloc(Body::blank());let shape=ctx.world.shapes.alloc(Shape::blank());
            ctx.world.shape_mut(shape).body=finish_body;ctx.level.level_body=terrain;
            for i in 0..2 {ctx.level.specials[i].kind=hw_data::SpecialKind::FinishLine;ctx.level.specials[i].shape=shape;}
            ctx.character.primary.bodies[LOWER_ARM1]=arm;
            let j=ctx.world.joints.alloc(Joint::blank());ctx.world.joint_mut(j).body1=arm;ctx.world.joint_mut(j).body2=finish_body;
            ctx.world.joint_mut(j).node1.other=finish_body;ctx.world.body_mut(arm).joint_list=edge_handle(j,0);
            let snapshot=raw_grip_context(ctx,"synthetic");let rows=snapshot["directional_target_body_connectivity"].as_array().unwrap();
            assert_eq!(rows.len(),8);assert_eq!(rows[0]["target_body"],json!(terrain));
            assert_eq!(rows[0]["arm_to_target_body"]["source_IsConnected_first_match_result"],json!(false));
            assert_eq!(rows[1]["target_body"],json!(finish_body));
            assert_eq!(rows[1]["target_roles"],json!(["finish.specials[0].shape.current_body","finish.specials[1].shape.current_body"]));
            assert_eq!(rows[1]["arm_to_target_body"]["source_IsConnected_first_match_result"],json!(true));
            assert_eq!(rows[1]["target_body_to_arm"]["source_IsConnected_first_match_result"],json!(false));
            assert_eq!(snapshot["finish_current_shape_refs"][0]["current_shape_slot"]["current_shape_body"],json!(finish_body));
        });
    }

    #[test]
    fn disabled_grip_context_omits_fields_and_preserves_existing_watches_and_captures() {
        synthetic_grip_context(|ctx| {
            lab::arm(&[24]);lab::configure_toi_target(lab::ToiTarget {primary:24,chest_shape:127,ceiling_shape:2,before_commit:true});
            let mut line=json!({"schema":1,"event_count":0,"events":[]});let before=line.clone();
            raw_attach_grip_context(&mut line,false,json!({"ignored":true}),ctx);
            assert_eq!(line,before);assert!(lab::armed());
            assert!(line.get("grip_context_before_input").is_none());assert!(line.get("grip_context_after_end_frame").is_none());
            raw_attach_grip_context(&mut line,true,raw_grip_context(ctx,"before_input"),ctx);
            assert!(lab::armed());
            assert_eq!(line["events"],before["events"]);assert_eq!(line["event_count"],before["event_count"]);
            assert_eq!(line["grip_context_after_end_frame"]["phase"],json!("after_end_frame"));
            lab::disarm();assert!(lab::take_toi_target().target.is_some());assert!(lab::take_events().is_empty());
        });
    }

    #[test]
    fn ordered_toi_strand_and_after_toi_keep_direct_identities_without_alias_filtering() {
        let events = [
            Event::Toi { toi:0.125, shape1:42, shape2:148, body1:24, body2:91 },
            Event::Strand { toi:0.125, shape1:42, shape2:148, body1:24, body2:91,
                pos1:[1.0,2.0], pos2:[3.0,4.0] },
            Event::Toi { toi:0.875, shape1:2, shape2:43, body1:0, body2:25 },
            Event::AfterToi { body:25, c0:[5.0,6.0], c:[7.0,8.0], v:[9.0,10.0] },
            Event::Freeze { body:25 },
            Event::Finish { x:11.0, y:12.0, inside:true, dead:true, other:2 },
            Event::ChestSmash,
        ];
        let rows = raw_events(&events);
        assert_eq!(rows.iter().map(|r|r["event"].as_str().unwrap()).collect::<Vec<_>>(),
            ["toi","strand","toi","after_toi","freeze","finish","chest_smash"]);
        for (ordinal,row) in rows.iter().enumerate() { assert_eq!(row["ordinal"],json!(ordinal)); }
        assert_eq!((rows[0]["shape2"].as_u64(),rows[0]["body2"].as_u64()),(Some(148),Some(91)));
        assert_eq!(rows[1]["pos1_bits"],json!(["3ff0000000000000","4000000000000000"]));
        assert_eq!((rows[2]["body2"].as_u64(),rows[3]["body"].as_u64()),(Some(25),Some(25)));
        assert_eq!(rows[3]["c0"],json!([5.0,6.0]));
        assert_eq!(rows[5]["dead"],json!(true));
        serde_json::to_string(&rows).unwrap();
    }

    #[test]
    fn raw_solve_preserves_signed_zero_nan_payload_and_infinities_as_exact_words() {
        let nan = f64::from_bits(0x7ff8000000001234);
        let row = raw_event(0,&Event::Solve { body:24,c0:[-0.0,nan],c:[f64::INFINITY,f64::NEG_INFINITY],
            v:[1.0,-1.0],w:-0.0,injected:false });
        assert_eq!(row["c0_bits"],json!(["8000000000000000","7ff8000000001234"]));
        assert_eq!(row["c_bits"],json!(["7ff0000000000000","fff0000000000000"]));
        assert_eq!(row["v_bits"],json!(["3ff0000000000000","bff0000000000000"]));
        assert_eq!(row["w_bits"],json!("8000000000000000"));
        assert_eq!(row["c0"][0].as_f64().unwrap().to_bits(),(-0.0f64).to_bits());
        assert!(row["c0"][1].is_null() && row["c"][0].is_null() && row["c"][1].is_null());
        let encoded = serde_json::to_string(&row).unwrap();
        let decoded:Value = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded["c0_bits"],row["c0_bits"]);
    }

    #[test]
    fn raw_damage_events_keep_phase_buffers_actor_scope_and_all_exact_words() {
        use hw_core::lab::{DamageBuffer,ReportPhase};
        let nan=f64::from_bits(0x7ff8000000005678);
        let buffer=DamageBuffer {present:true,shape:127,other:148,impulse:-0.0,own_body:Some(24),other_body:None};
        let events=[Event::DamageResult {phase:ReportPhase::Toi,body:24,actor_kind:3,child:false,dead:false,
            shape:127,own:127,other:148,own_body:Some(24),other_body:Some(43),impulse:f64::INFINITY,threshold:nan,previous:buffer},
            Event::DamageSnapshot {body:24,actor_kind:3,child:false,dead:false,shape:127,threshold:nan,smash_limit:3.5,buffer},
            Event::DamageConsume {body:24,actor_kind:3,child:false,dead:true,shape:127,threshold:nan,smash_limit:3.5,buffer}];
        let rows=raw_events(&events);
        assert_eq!(rows.iter().map(|r|r["event"].as_str().unwrap()).collect::<Vec<_>>(),["damage_result","damage_snapshot","damage_consume"]);
        assert_eq!(rows[0]["phase"],json!("toi"));assert_eq!(rows[0]["other_body"],json!(43));
        assert_eq!(rows[0]["impulse_bits"],json!("7ff0000000000000"));assert_eq!(rows[0]["threshold_bits"],json!("7ff8000000005678"));
        assert!(rows[0]["impulse"].is_null()&&rows[0]["threshold"].is_null());
        assert_eq!(rows[0]["previous"]["impulse_bits"],json!("8000000000000000"));assert!(rows[0]["previous"]["other_body"].is_null());
        assert_eq!(rows[1]["buffer"],rows[2]["buffer"]);assert_eq!(rows[1]["dead"],json!(false));assert_eq!(rows[2]["dead"],json!(true));
        let roundtrip:Vec<Value>=serde_json::from_str(&serde_json::to_string(&rows).unwrap()).unwrap();assert_eq!(roundtrip,rows);
    }

    #[test]
    fn raw_context_links_direct_roots_and_unowned_contact_neighbors_without_remapping() {
        std::thread::Builder::new().stack_size(64<<20).spawn(|| {
            use hw_core::b2::state::{Body,Shape,Contact};
            let mut ctx=Box::new(State::new());
            let primary=ctx.world.bodies.alloc(Body::blank());
            let other=ctx.world.bodies.alloc(Body::blank());
            let ps=ctx.world.shapes.alloc(Shape::blank());let os=ctx.world.shapes.alloc(Shape::blank());
            ctx.character.primary.bodies[hw_core::game::character::CHEST]=primary;
            ctx.character.primary.shapes[0]=ps;ctx.level.focus=primary;
            ctx.world.body_list=primary;ctx.world.body_mut(primary).next=other;
            ctx.world.body_mut(primary).shape_list=ps;ctx.world.body_mut(other).shape_list=os;
            ctx.world.shape_mut(ps).body=primary;ctx.world.shape_mut(os).body=other;
            let contact=ctx.world.contacts.alloc(Contact::blank());ctx.world.contact_list=contact;
            ctx.world.contact_mut(contact).shape1=ps;ctx.world.contact_mut(contact).shape2=os;
            let context=raw_context(&ctx,primary);
            let bodies=context["bodies"].as_array().unwrap();let shapes=context["shapes"].as_array().unwrap();
            assert_eq!(bodies.len(),2);assert_eq!(shapes.len(),2);
            assert_eq!(bodies.iter().find(|b|b["handle"]==json!(primary)).unwrap()["direct_actor_refs"],json!(["primary.bodies[0]"]));
            let neighbor=bodies.iter().find(|b|b["handle"]==json!(other)).unwrap();
            assert_eq!(neighbor["direct_actor_refs"],json!([]));assert_eq!(neighbor["world_list_member"],json!(true));
            assert_eq!(context["primary_contacts"][0]["bodies"],json!([primary,other]));
            assert_eq!(shapes.iter().find(|s|s["handle"]==json!(os)).unwrap()["body"],json!(other));
            assert_eq!(context["world_bodies_complete"],json!(true));assert_eq!(context["world_contacts_complete"],json!(true));
        }).unwrap().join().unwrap();
    }

    #[test]
    fn raw_context_reports_cycles_and_out_of_range_roots_without_unchecked_reads() {
        std::thread::Builder::new().stack_size(64<<20).spawn(|| {
            use hw_core::b2::state::{Body,Shape};
            let mut ctx=Box::new(State::new());let b=ctx.world.bodies.alloc(Body::blank());let other=ctx.world.bodies.alloc(Body::blank());let s=ctx.world.shapes.alloc(Shape::blank());
            ctx.world.body_list=b;ctx.world.body_mut(b).next=b;ctx.world.body_mut(b).shape_list=s;
            ctx.world.shape_mut(s).body=b;ctx.world.shape_mut(s).next=s;
            ctx.character.primary.bodies[0]=b;ctx.character.primary.bodies[1]=ctx.world.bodies.items.len() as H;
            ctx.character.primary.bodies[2]=other;
            ctx.character.primary.shapes[0]=ctx.world.shapes.items.len() as H;
            ctx.world.contact_list=ctx.world.contacts.items.len() as H;
            let context=raw_context(&ctx,b);
            assert_eq!(context["world_bodies_complete"],json!(false));assert_eq!(context["world_contacts_complete"],json!(false));
            assert_eq!(context["body_shape_chains_complete"],json!(false));assert_eq!(context["selected_shapes_readable"],json!(false));
            assert!(context["bodies"].as_array().unwrap().iter().any(|b|b["readable"]==json!(false)));
            assert!(context["shapes"].as_array().unwrap().iter().any(|s|s["readable"]==json!(false)));
            assert!(context["bodies"].as_array().unwrap().iter().find(|b|b["handle"]==json!(other)).unwrap()["world_list_member"].is_null());
        }).unwrap().join().unwrap();
    }

    fn broadphase_words(ctx: &State) -> Vec<u64> {
        let b=&ctx.world.broad_phase;
        let mut words=vec![b.free_proxy as u64,b.proxy_count as u32 as u64,b.query_result_count as u32 as u64,
            b.time_stamp as u64,b.pair_count as u32 as u64,b.pair_buffer_count as u32 as u64,b.free_pair as u64];
        for v in [b.quantization_factor.x,b.quantization_factor.y,b.world_aabb_lower.x,b.world_aabb_lower.y,
            b.world_aabb_upper.x,b.world_aabb_upper.y] {words.push(v.to_bits());}
        words.extend(b.query_results.iter().map(|&v|v as u64));
        words.extend(b.hash_table.iter().map(|&v|v as u64));
        for axis in &b.bounds {for v in axis {words.extend([v.value as u64,v.proxy_id as u64,v.stabbing_count as u64]);}}
        for p in &b.proxy_pool {words.extend([p.lower_bounds[0] as u64,p.lower_bounds[1] as u64,
            p.upper_bounds[0] as u64,p.upper_bounds[1] as u64,p.overlap_count as u64,p.time_stamp as u64,p.shape as u64]);}
        for p in &b.pairs {words.extend([p.contact as u64,p.proxy_id1 as u64,p.proxy_id2 as u64,p.status as u64,p.next as u64]);}
        for p in &b.pair_buffer {words.extend([p.proxy_id1 as u64,p.proxy_id2 as u64]);}
        words
    }

    fn grip_words(ctx:&State)->Vec<u64> {
        let w=&ctx.world;
        let mut words=vec![w.joint_list as u64,w.joint_count as u32 as u64,ctx.character.child_active.word() as u64];
        words.extend(w.bodies.items.iter().map(|b|b.joint_list as u64));
        words.extend(w.joints.alive.iter().map(|&v|v as u64));words.extend(w.joints.retained.iter().map(|&v|v as u64));
        words.extend(w.joints.pending.iter().map(|&v|v as u64));words.push(w.joints.pending_len as u64);
        for j in &w.joints.items {words.extend([j.prev as u64,j.next as u64,j.body1 as u64,j.body2 as u64,
            j.node1.other as u64,j.node1.prev as u64,j.node1.next as u64,j.node2.other as u64,j.node2.prev as u64,j.node2.next as u64,
            j.collide_connected.word() as u64,j.broken.word() as u64]);}
        for actor in [&ctx.character.primary,&ctx.character.child] {
            words.extend(actor.joints.iter().chain(actor.grip_joints.iter()).map(|&v|v as u64));
            words.extend(actor.contact_adds.iter().map(|v|v.word() as u64));
            words.extend(actor.contact_thresholds.iter().map(|v|v.to_bits()));
            for r in &actor.contact_results {words.extend([r.present.word() as u64,r.shape as u64,r.other as u64,r.impulse.to_bits()]);}
        }
        words
    }

    #[test]
    fn grip_context_is_read_only_for_all_observers_buffers_joint_words_and_broadphase_scratch() {
        std::thread::Builder::new().stack_size(64<<20).spawn(|| {
            let mut sim=Sim::new(45,3).unwrap();sim.ctx.character.child_active=hw_core::Flag32::TRUE;
            let observers=|s:&Sim|[hw_core::diagnostic::broadphase_string(&s.ctx),
                hw_core::physics_observer::physics_string(&s.ctx),hw_core::game_observer::game_string(&s.ctx),
                hw_core::links_observer::links_string(&s.ctx),hw_core::mechanics_observer::mechanics_string(&s.ctx),
                hw_core::diagnostic::replay_string(&s.ctx,&s.replay)];
            let before=observers(&sim);let bp=broadphase_words(&sim.ctx);let grips=grip_words(&sim.ctx);
            let masks=sim.masks.clone();let snapshot=raw_grip_context(&sim.ctx,"before_input");
            assert_eq!(snapshot["world_joint_chain_complete"],json!(true));
            assert_eq!(snapshot["actors"][0]["name"],json!("primary"));assert_eq!(snapshot["actors"][1]["name"],json!("child"));
            assert_eq!(snapshot["finish_current_shape_refs"].as_array().unwrap().len(),1);
            let finish=&snapshot["finish_current_shape_refs"][0];
            assert_eq!(finish["current_shape_slot"]["current_shape_body"],json!(sim.ctx.level.level_body));
            let roundtrip:Value=serde_json::from_str(&serde_json::to_string(&snapshot).unwrap()).unwrap();assert_eq!(snapshot,roundtrip);
            let _=raw_grip_context(&sim.ctx,"after_end_frame");
            assert_eq!(before,observers(&sim));assert_eq!(bp,broadphase_words(&sim.ctx));assert_eq!(grips,grip_words(&sim.ctx));
            assert_eq!(masks,sim.masks);assert_eq!(sim.ctx.character.child_active.word(),1);
        }).unwrap().join().unwrap();
    }

    #[test]
    #[ignore="eight full299 grip-off/on pairs including TOI combination; run serially with global lag tests"]
    fn grip_context_preserves_full_299_observers_replay_and_events() {
        std::thread::Builder::new().stack_size(64<<20).spawn(|| {
            let _hang=super::super::sim::HangOverride::new(0);super::super::sim::set_lag_ms(0.0);
            let fixtures=[
                ("original",include_str!("../../../../reference/tas/obstacle-dad-ceiling-strand-f143.tas")),
                ("roof",include_str!("../../../../reference/tas/obstacle-dad-roofsolve-f143.tas")),
                ("f274",include_str!("../../../../reference/tas/obstacle-pairmax-f274.tas")),
                ("late353",include_str!("../../../../reference/tas/obstacle-dad-late-overflow-f291.tas"))];
            let observers=|s:&Sim|[hw_core::diagnostic::broadphase_string(&s.ctx),
                hw_core::physics_observer::physics_string(&s.ctx),hw_core::game_observer::game_string(&s.ctx),
                hw_core::links_observer::links_string(&s.ctx),hw_core::mechanics_observer::mechanics_string(&s.ctx)];
            let replay=|s:&Sim|[s.replay.iteration,s.replay.frames,s.replay.total_iterations,
                s.replay.input_allowed.word(),s.replay.completed.word(),s.replay.finish_frame];
            for probe in [false,true] {for (name,raw) in fixtures {
                assert_eq!(raw.len(),2392);assert!(raw.bytes().all(|b|b==b'0'||b==b'1'));
                let masks=super::super::sim::parse_tas(raw).unwrap();assert_eq!(masks.len(),299);
                let mut off=Sim::new(45,3).unwrap();let mut on=Sim::new(45,3).unwrap();
                off.replay=hw_core::game::session::ReplayState::new(299);on.replay=off.replay;
                off.masks=masks.clone();on.masks=masks.clone();let mut rows=0;
                for frame in 1..=299 {
                    let h=off.ctx.character.primary.bodies[hw_core::game::character::CHEST];
                    if h==NONE{lab::arm(&[]);}else{lab::arm(&[h]);}
                    let mut eligibility_a=Value::Null;
                    if probe {let (target,e)=raw_toi_target_eligibility(&off.ctx,h,45);eligibility_a=e;
                        if let Some(t)=target{lab::configure_toi_target(t);}}
                    let a=off.step(0);lab::disarm();let ea=lab::take_events();let pa=lab::take_pair_exhaustion();
                    let mut line_a=json!({"events":raw_events(&ea),"pair_exhaustion":raw_pair_exhaustion(pa)});
                    raw_attach_toi_target(&mut line_a,probe,eligibility_a);
                    raw_attach_grip_context(&mut line_a,false,Value::Null,&off.ctx);
                    assert!(line_a.get("grip_context_before_input").is_none());
                    assert!(line_a.get("grip_context_after_end_frame").is_none());
                    let disabled=lab::take_toi_target();assert!(disabled.target.is_none());assert_eq!(disabled.len,0);
                    let before=raw_grip_context(&on.ctx,"before_input");
                    let h=on.ctx.character.primary.bodies[hw_core::game::character::CHEST];
                    if h==NONE{lab::arm(&[]);}else{lab::arm(&[h]);}
                    let mut eligibility_b=Value::Null;
                    if probe {let (target,e)=raw_toi_target_eligibility(&on.ctx,h,45);eligibility_b=e;
                        if let Some(t)=target{lab::configure_toi_target(t);}}
                    let b=on.step(0);lab::disarm();let eb=lab::take_events();let pb=lab::take_pair_exhaustion();
                    let mut line_b=json!({"events":raw_events(&eb),"pair_exhaustion":raw_pair_exhaustion(pb)});
                    raw_attach_toi_target(&mut line_b,probe,eligibility_b);
                    raw_attach_grip_context(&mut line_b,true,before,&on.ctx);rows+=1;
                    assert_eq!(line_b["grip_context_before_input"]["phase"],json!("before_input"));
                    assert_eq!(line_b["grip_context_after_end_frame"]["phase"],json!("after_end_frame"));
                    line_b.as_object_mut().unwrap().remove("grip_context_before_input");
                    line_b.as_object_mut().unwrap().remove("grip_context_after_end_frame");
                    assert_eq!(line_a,line_b,"{name} F{frame} probe={probe}: prior raw events and captures");
                    assert!(lab::take_toi_target().target.is_none());
                    assert_eq!((a.frame,a.mask,a.site,a.port_abort,a.finished,a.pairs,a.contacts),
                        (b.frame,b.mask,b.site,b.port_abort,b.finished,b.pairs,b.contacts));
                    assert_eq!((a.frame,a.mask),(frame,masks[frame as usize-1]));assert!(!a.lagging && !b.lagging);
                    assert_eq!(replay(&off),replay(&on));assert_eq!(off.ctx.world.lock.word(),on.ctx.world.lock.word());
                    assert_eq!(broadphase_words(&off.ctx),broadphase_words(&on.ctx));assert_eq!(grip_words(&off.ctx),grip_words(&on.ctx));
                    assert_eq!(hw_core::diagnostic::replay_string(&off.ctx,&off.replay),hw_core::diagnostic::replay_string(&on.ctx,&on.replay));
                    for (left,right) in observers(&off).into_iter().zip(observers(&on)){assert_eq!(left,right,"{name} F{frame} probe={probe}");}
                }
                assert_eq!(rows,299);assert_eq!(replay(&off),[299,299,299,1,0,0]);assert_eq!(off.masks,masks);assert_eq!(on.masks,masks);
            }}
        }).unwrap().join().unwrap();
    }

    #[test]
    fn targeted_pair_inventories_are_exactly_read_only_and_keep_source_candidates() {
        std::thread::Builder::new().stack_size(64<<20).spawn(|| {
            let sim=Sim::new(45,3).unwrap();
            let primary=sim.ctx.character.primary.bodies[hw_core::game::character::CHEST];
            let bp_before=broadphase_words(&sim.ctx);
            let physics_before=hw_core::physics_observer::physics_string(&sim.ctx);
            let before=raw_pair_inventory(&sim.ctx,primary,45,"before_input");
            let identities=pair_shape_identities(&sim.ctx,"before_input_current_slot",&[]);
            let after=raw_pair_inventory(&sim.ctx,primary,45,"after_end_frame");
            assert_eq!(before["ceiling_shape2_derived_bounds_exact_words_match"],json!(true));
            assert_eq!(before["primary_shape_chain_complete"],json!(true));
            let sensor=sim.ctx.level.specials[12].shape;
            assert_ne!(sensor,NONE);
            assert!(before["targets"].as_array().unwrap().iter().any(|r|
                r["role"]==json!("starting_special12_sensor_current_reference") && r["shape"]["handle"]==json!(sensor)));
            assert!(before["targets"].as_array().unwrap().iter().any(|r|
                r["role"]==json!("starting_spikes_backing_following_slot_candidate") && r["shape"]["handle"]==json!(sensor+1)));
            assert!(identities.contains_key(&sensor));
            assert_eq!(before["pairs"],after["pairs"]);
            assert_eq!(bp_before,broadphase_words(&sim.ctx));
            assert_eq!(physics_before,hw_core::physics_observer::physics_string(&sim.ctx));
            // The source-order backing remains visible even if host-derived
            // decimal bounds fail an exact-word match; no approximate identity.
            assert!(before.get("geometry_identity_tolerance").is_none());
        }).unwrap().join().unwrap();
    }

    #[test]
    fn pair_inventory_malformed_lists_slots_and_hash_chains_report_unknown() {
        std::thread::Builder::new().stack_size(64<<20).spawn(|| {
            use hw_core::b2::state::{Body,Shape,B2_TABLE_MASK,NULL_CONTACT,B2_INVALID};
            let mut ctx=Box::new(State::new());ctx.character.primary.kind=3;
            let body=ctx.world.bodies.alloc(Body::blank());
            let a=ctx.world.shapes.alloc(Shape::blank());let b=ctx.world.shapes.alloc(Shape::blank());
            ctx.world.body_list=body;ctx.world.body_mut(body).next=body;
            ctx.world.body_mut(body).shape_list=a;ctx.world.shape_mut(a).body=body;ctx.world.shape_mut(a).next=a;
            ctx.world.shape_mut(a).proxy_id=0;ctx.world.shape_mut(b).proxy_id=1;
            for (id,shape) in [(0,a),(1,b)] {ctx.world.broad_phase.proxy_pool[id].shape=shape;
                ctx.world.broad_phase.proxy_pool[id].overlap_count=0;}
            let bucket=(hw_core::b2::broad_phase::hash(0,1)&B2_TABLE_MASK) as usize;
            ctx.world.broad_phase.hash_table[bucket]=0;
            let p=&mut ctx.world.broad_phase.pairs[0];
            p.proxy_id1=0;p.proxy_id2=1;p.contact=NULL_CONTACT;p.next=0;
            let words=broadphase_words(&ctx);
            let row=inventory_pair(&ctx,a,b,&BTreeSet::new(),false);
            assert_eq!(row["hash_chain_complete"],json!(false));
            assert!(row["absent_from_complete_hash_chain"].is_null());
            assert_eq!(row["matches"][0]["contact"]["kind"],json!("null_contact_sentinel"));
            let inventory=raw_pair_inventory(&ctx,body,45,"before_input");
            assert_eq!(inventory["world_body_chain_complete"],json!(false));
            assert_eq!(inventory["primary_shape_chain_complete"],json!(false));
            assert_eq!(words,broadphase_words(&ctx));
            ctx.world.broad_phase.pairs[0].next=ctx.world.broad_phase.pairs.len() as u32;
            assert_eq!(inventory_pair(&ctx,a,b,&BTreeSet::new(),true)["hash_chain_complete"],json!(false));
            ctx.world.broad_phase.proxy_pool[0].overlap_count=B2_INVALID;
            let invalid=inventory_pair(&ctx,a,b,&BTreeSet::new(),true);
            assert_eq!(invalid["lookup_performed"],json!(false));assert_eq!(invalid["absence_known"],json!(false));
            assert_eq!(raw_proxy(&ctx,u32::MAX,a)["readable"],json!(false));
            let (found,complete)=bounded_handles(ctx.world.bodies.items.len() as H,ctx.world.bodies.items.len(),
                |h|ctx.world.bodies.items.get(h as usize).map(|b|b.next));
            assert!(!complete && found.is_empty());
        }).unwrap().join().unwrap();
    }

    #[test]
    fn toi_target_requires_current_live_membership_and_is_read_only(){
        std::thread::Builder::new().stack_size(64<<20).spawn(||{
            let mut sim=Sim::new(45,3).unwrap();let ctx=&mut sim.ctx;
            let h=ctx.character.primary.bodies[hw_core::game::character::CHEST];
            let chest=ctx.character.primary.shapes[hw_core::game::character::CHEST];
            let words=broadphase_words(ctx);let physics=hw_core::physics_observer::physics_string(ctx);
            let (target,status)=raw_toi_target_eligibility(ctx,h,45);assert!(target.is_some());
            assert_eq!(status["configured"],json!(true));assert_eq!(words,broadphase_words(ctx));
            assert_eq!(physics,hw_core::physics_observer::physics_string(ctx));
            let a=ctx.world.shapes.alive[2];ctx.world.shapes.alive[2]=0;
            let (target,status)=raw_toi_target_eligibility(ctx,h,45);assert!(target.is_none());
            assert_eq!(status["unavailable_reason"],json!("required_arena_slot_not_alive"));ctx.world.shapes.alive[2]=a;
            let next=ctx.world.body(h).next;ctx.world.body_mut(h).next=h;
            let (target,status)=raw_toi_target_eligibility(ctx,h,45);assert!(target.is_none());
            assert_eq!(status["unavailable_reason"],json!("world_body_chain_incomplete"));ctx.world.body_mut(h).next=next;
            let head=ctx.world.body(h).shape_list;ctx.world.body_mut(h).shape_list=NONE;
            let (target,status)=raw_toi_target_eligibility(ctx,h,45);assert!(target.is_none());
            assert_eq!(status["unavailable_reason"],json!("required_shape_not_in_current_body_chain"));ctx.world.body_mut(h).shape_list=head;
            let next=ctx.world.shape(chest).next;ctx.world.shape_mut(chest).next=chest;
            let (target,status)=raw_toi_target_eligibility(ctx,h,45);assert!(target.is_none());
            assert_eq!(status["unavailable_reason"],json!("required_body_shape_chain_incomplete"));ctx.world.shape_mut(chest).next=next;
            let proxy=ctx.world.shape(chest).proxy_id as usize;let shape=ctx.world.broad_phase.proxy_pool[proxy].shape;
            ctx.world.broad_phase.proxy_pool[proxy].shape=NONE;
            let (target,status)=raw_toi_target_eligibility(ctx,h,45);assert!(target.is_none());
            assert_eq!(status["unavailable_reason"],json!("required_proxy_slot_invalid_or_back_reference_mismatch"));
            ctx.world.broad_phase.proxy_pool[proxy].shape=shape;
        }).unwrap().join().unwrap();
    }
    #[test]
    fn disabled_toi_target_output_is_exactly_unchanged_and_does_not_drain(){
        lab::arm(&[]);lab::configure_toi_target(lab::ToiTarget {primary:24,chest_shape:127,ceiling_shape:2,before_commit:true});
        let mut line=json!({"schema":1,"event_count":0,"events":[]});let before=line.clone();
        raw_attach_toi_target(&mut line,false,json!({"configured":false}));assert_eq!(line,before);
        lab::disarm();assert!(lab::take_toi_target().target.is_some());assert!(lab::take_events().is_empty());
        lab::arm(&[]);let mut line=before.clone();
        raw_attach_toi_target(&mut line,true,json!({"configured":false,"unavailable_reason":"fixture"}));
        assert_eq!(line["events"],before["events"]);assert_eq!(line["event_count"],before["event_count"]);
        assert_eq!(line["toi_target_probe"]["eligibility"]["unavailable_reason"],json!("fixture"));
        assert!(line["toi_target_probe"]["target"].is_null());lab::disarm();
    }
    #[test]
    #[ignore="four full299 target-off/on exact observer and Event replays; run serially with global lag tests"]
    fn target_toi_probe_preserves_full299_replay_events_and_all_five_observers(){
        std::thread::Builder::new().stack_size(64<<20).spawn(||{
            let _hang=super::super::sim::HangOverride::new(0);super::super::sim::set_lag_ms(0.0);
            let fixtures=[
                ("original",include_str!("../../../../reference/tas/obstacle-dad-ceiling-strand-f143.tas")),
                ("roof",include_str!("../../../../reference/tas/obstacle-dad-roofsolve-f143.tas")),
                ("f274",include_str!("../../../../reference/tas/obstacle-pairmax-f274.tas")),
                ("late353",include_str!("../../../../reference/tas/obstacle-dad-late-overflow-f291.tas"))];
            let observers=|s:&Sim|[hw_core::diagnostic::broadphase_string(&s.ctx),
                hw_core::physics_observer::physics_string(&s.ctx),hw_core::game_observer::game_string(&s.ctx),
                hw_core::links_observer::links_string(&s.ctx),hw_core::mechanics_observer::mechanics_string(&s.ctx)];
            let replay=|s:&Sim|[s.replay.iteration,s.replay.frames,s.replay.total_iterations,
                s.replay.input_allowed.word(),s.replay.completed.word(),s.replay.finish_frame];
            for (name,raw) in fixtures{
                assert_eq!(raw.len(),2392);assert!(raw.bytes().all(|b|b==b'0'||b==b'1'));
                let masks=super::super::sim::parse_tas(raw).unwrap();assert_eq!(masks.len(),299);
                let mut off=Sim::new(45,3).unwrap();let mut on=Sim::new(45,3).unwrap();
                off.replay=hw_core::game::session::ReplayState::new(299);on.replay=off.replay;
                off.masks=masks.clone();on.masks=masks.clone();let mut records=0;
                for frame in 1..=299{
                    let h=off.ctx.character.primary.bodies[hw_core::game::character::CHEST];
                    if h==NONE{lab::arm(&[]);}else{lab::arm(&[h]);}
                    let a=off.step(0);lab::disarm();let ea=lab::take_events();let pa=lab::take_pair_exhaustion();
                    let disabled=lab::take_toi_target();assert!(disabled.target.is_none());assert_eq!(disabled.len,0);
                    let h=on.ctx.character.primary.bodies[hw_core::game::character::CHEST];
                    if h==NONE{lab::arm(&[]);}else{lab::arm(&[h]);}
                    let (target,_)=raw_toi_target_eligibility(&on.ctx,h,45);
                    if let Some(t)=target{lab::configure_toi_target(t);}
                    let b=on.step(0);lab::disarm();let eb=lab::take_events();let pb=lab::take_pair_exhaustion();
                    let enabled=lab::take_toi_target();records+=enabled.len;
                    assert!(lab::take_toi_target().target.is_none());
                    assert_eq!(raw_events(&ea),raw_events(&eb),"{name} F{frame}: existing ordered Event words");
                    assert_eq!(raw_pair_exhaustion(pa),raw_pair_exhaustion(pb),"{name} F{frame}: prior side capture");
                    assert_eq!((a.frame,a.mask,a.site,a.port_abort,a.finished,a.pairs,a.contacts),
                        (b.frame,b.mask,b.site,b.port_abort,b.finished,b.pairs,b.contacts),"{name} F{frame}: frame fields");
                    assert_eq!((a.frame,a.mask),(frame,masks[frame as usize-1]));assert!(!a.lagging && !b.lagging);
                    assert_eq!(replay(&off),replay(&on));assert_eq!(off.ctx.world.lock.word(),on.ctx.world.lock.word());
                    assert_eq!(hw_core::diagnostic::replay_string(&off.ctx,&off.replay),
                        hw_core::diagnostic::replay_string(&on.ctx,&on.replay));
                    for (left,right) in observers(&off).into_iter().zip(observers(&on)){assert_eq!(left,right,"{name} F{frame}");}
                }
                assert!(records>0,"{name}: exercised no optional records");assert_eq!(replay(&off),[299,299,299,1,0,0]);
                assert_eq!(off.masks,masks);assert_eq!(on.masks,masks);
            }
        }).unwrap().join().unwrap();
    }
    #[test]
    fn pair_exhaustion_formatting_preserves_fixed_capture_and_current_slot_limits() {
        std::thread::Builder::new().stack_size(64<<20).spawn(|| {
            use lab::{PairExhaustion,PairExhaustionCapture,PairProxySlot,ProxyOperation,ProxyOperationKind,WorldPhase,ReportPhase};
            use hw_core::b2::state::{Body,Shape};
            let mut ctx=Box::new(State::new());
            let body=ctx.world.bodies.alloc(Body::blank());let shape=ctx.world.shapes.alloc(Shape::blank());
            ctx.world.shape_mut(shape).body=body;ctx.character.child.shapes[4]=shape;ctx.character.child.bodies[4]=body;
            ctx.world.broad_phase.proxy_pool[0].shape=shape;
            let before=pair_shape_identities(&ctx,"before_input_current_slot",&[]);
            let capture=PairExhaustionCapture {first:Some(PairExhaustion {proxy_ids:[0,u32::MAX],hash_bucket:4095,
                free_pair:65535,pair_count:4096,pair_buffer_count:-1,
                proxy_slots:[PairProxySlot {readable:true,valid:true,shape:Some(shape)},PairProxySlot {readable:false,valid:false,shape:None}],
                operation:ProxyOperation {kind:ProxyOperationKind::MoveProxy,initiating_proxy_id:Some(0)},
                world_phase:WorldPhase::MainSolve,report_phase:ReportPhase::None,events_before:7}),count:2,count_saturated:true};
            let row=raw_pair_exhaustion(capture);
            assert_eq!(row["first"]["proxy_ids"],json!([0,u32::MAX]));assert_eq!(row["first"]["pair_buffer_count"],json!(-1));
            assert_eq!(row["first"]["events_before"],json!(7));assert_eq!(row["count"],json!(2));
            assert_eq!(row["first"]["operation"]["kind"],json!("MoveProxy"));
            // A copied shape can remain readable after losing its current proxy
            // reference. Resolve that slot without asserting historical owner.
            ctx.world.broad_phase.proxy_pool[0].shape=NONE;
            let resolution=failed_pair_resolution(&ctx,&capture,&before);
            assert_eq!(resolution["slots"][0]["before_input_current_slot"]["current_shape_actor_refs"],json!(["child.shapes[4]"]));
            assert_eq!(resolution["slots"][0]["after_end_frame_current_slot"]["body_handle"],json!(body));
            assert_eq!(resolution["slots"][0]["capture_time_body_owner_proven"],json!(false));
            assert!(resolution["slots"][1]["after_end_frame_current_slot"].is_null());
            serde_json::to_string(&resolution).unwrap();
        }).unwrap().join().unwrap();
    }
}
