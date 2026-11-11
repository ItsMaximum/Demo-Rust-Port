//! Field names for the legacy diagnostic stream; indices follow Flash list order.
use serde_json::Value;

pub fn compare(expected: &Value, actual: &Value) -> Option<String> {
    for key in [
        "frame",
        "iteration",
        "input",
        "err",
        "lock",
        "dead",
        "inputAllowed",
    ] {
        if expected[key] != actual[key] {
            return Some(format!(
                "{key}: Flash {} vs Rust {}",
                expected[key], actual[key]
            ));
        }
    }
    if !expected["finish"].is_null() && expected["finish"] != actual["finish"] {
        return Some(format!(
            "finish: Flash {} vs Rust {}",
            expected["finish"], actual["finish"]
        ));
    }
    for key in [
        "state",
        "bpState",
        "physicsState",
        "gameState",
        "linksState",
        "mechanicsState",
        "callbacksState",
        "levelState",
        "levelAddState",
        "listenersState",
    ] {
        if let Some(a) = expected[key].as_str() {
            let b = actual[key].as_str().unwrap_or("");
            if a != b {
                let left = if key != "state" && key != "bpState" {
                    named_fields(a)
                } else {
                    fields(a, key == "bpState")
                };
                let right = if key != "state" && key != "bpState" {
                    named_fields(b)
                } else {
                    fields(b, key == "bpState")
                };
                for i in 0..left.len().max(right.len()) {
                    if left.get(i) != right.get(i) {
                        let name = left
                            .get(i)
                            .or_else(|| right.get(i))
                            .map(|x| x.0.as_str())
                            .unwrap_or(key);
                        return Some(format!(
                            "{name}: Flash {} vs Rust {}",
                            left.get(i).map(|x| x.1).unwrap_or("<missing>"),
                            right.get(i).map(|x| x.1).unwrap_or("<missing>")
                        ));
                    }
                }
                return Some(format!("{key}: stream structure differs"));
            }
        }
    }
    for key in [
        "hash",
        "len",
        "bpHash",
        "physicsHash",
        "gameHash",
        "linksHash",
        "mechanicsHash",
        "callbacksHash",
        "levelHash",
        "listenersHash",
    ] {
        if key != "hash" && key != "len" && expected[key].is_null() {
            continue;
        }
        if expected[key] != actual[key] {
            return Some(format!(
                "{key}: Flash {} vs Rust {} (raw state unavailable at this frame)",
                expected[key], actual[key]
            ));
        }
    }
    None
}

fn append<'a>(out: &mut Vec<(String, &'a str)>, prefix: &str, text: &'a str, names: &[&str]) {
    for (i, value) in text.split(',').enumerate() {
        let name = names
            .get(i)
            .map(|s| s.to_string())
            .unwrap_or_else(|| format!("field[{i}]"));
        out.push((format!("{prefix}.{name}"), value));
    }
}
fn fields(text: &str, broadphase: bool) -> Vec<(String, &str)> {
    let mut out = Vec::new();
    if broadphase {
        let mut segments = text.split('|');
        append(
            &mut out,
            "broadphase",
            segments.next().unwrap_or(""),
            &[
                "proxyCount",
                "freeProxy",
                "timeStamp",
                "queryResultCount",
                "pairCount",
                "pairBufferCount",
                "freePair",
            ],
        );
        let mut indices = [0; 5];
        for segment in segments {
            let (tag, names): (usize, &[&str]) = match segment.as_bytes().first() {
                Some(b'b') => (0, &["value", "proxyId", "stabbingCount"]),
                Some(b'p') => (
                    1,
                    &[
                        "lowerBounds[0]",
                        "lowerBounds[1]",
                        "upperBounds[0]",
                        "upperBounds[1]",
                        "overlapCount",
                        "timeStamp",
                    ],
                ),
                Some(b'q') => (2, &["proxyId"]),
                Some(b'r') => (3, &["proxyId1", "proxyId2", "status", "next", "hashTable"]),
                Some(b'u') => (4, &["proxyId1", "proxyId2"]),
                _ => {
                    out.push(("broadphase.invalid".into(), segment));
                    continue;
                }
            };
            let labels = ["bounds", "proxies", "queryResults", "pairs", "pairBuffer"];
            append(
                &mut out,
                &format!("broadphase.{}[{}]", labels[tag], indices[tag]),
                &segment[1..],
                names,
            );
            indices[tag] += 1;
        }
        return out;
    }
    for section in text.split('#') {
        let Some(tag) = section.as_bytes().first() else {
            continue;
        };
        let mut entries = section[1..].split('|');
        let header = entries.next().unwrap_or("");
        match tag {
            b'b' => {
                out.push(("world.bodyCount".into(), header));
                for (i, body) in entries.enumerate() {
                    let prefix = format!("bodies[{i}]");
                    let mut shapes = body.split('/');
                    append(
                        &mut out,
                        &prefix,
                        shapes.next().unwrap_or(""),
                        &[
                            "sweep.c.x",
                            "sweep.c.y",
                            "sweep.a",
                            "sweep.c0.x",
                            "sweep.c0.y",
                            "sweep.a0",
                            "sweep.localCenter.x",
                            "sweep.localCenter.y",
                            "linearVelocity.x",
                            "linearVelocity.y",
                            "angularVelocity",
                            "force.x",
                            "force.y",
                            "torque",
                            "mass",
                            "I",
                            "invMass",
                            "invI",
                            "sleepTime",
                            "flags",
                            "shapeCount",
                        ],
                    );
                    for (j, shape) in shapes.enumerate() {
                        let prefix = format!("{prefix}.shapes[{j}]");
                        let circle = shape.starts_with("0,");
                        let mut names = vec![
                            "type".to_owned(),
                            "proxyId".into(),
                            "categoryBits".into(),
                            "maskBits".into(),
                            "groupIndex".into(),
                            "isSensor".into(),
                            "material".into(),
                            "sweepRadius".into(),
                        ];
                        if circle {
                            names.extend(
                                ["localPosition.x", "localPosition.y", "radius"].map(String::from),
                            );
                        } else {
                            names.push("vertexCount".into());
                            for k in 0..shape.split(',').count().saturating_sub(9) {
                                names.push(format!(
                                    "vertices[{}].{}",
                                    k / 2,
                                    if k % 2 == 0 { "x" } else { "y" }
                                ));
                            }
                        }
                        append(
                            &mut out,
                            &prefix,
                            shape,
                            &names.iter().map(String::as_str).collect::<Vec<_>>(),
                        );
                    }
                }
            }
            b'j' => {
                out.push(("world.jointCount".into(), header));
                for (i, joint) in entries.enumerate() {
                    let names: &[&str] = match joint.split(',').next() {
                        Some("1") => &[
                            "type",
                            "pivotForce.x",
                            "pivotForce.y",
                            "motorForce",
                            "limitForce",
                            "limitState",
                            "enableMotor",
                            "enableLimit",
                            "motorSpeed",
                            "lowerAngle",
                            "upperAngle",
                            "maxMotorTorque",
                            "broken",
                        ],
                        Some("2") => &[
                            "type",
                            "force",
                            "torque",
                            "motorForce",
                            "limitForce",
                            "limitState",
                            "enableMotor",
                            "enableLimit",
                            "motorSpeed",
                            "lowerTranslation",
                            "upperTranslation",
                            "maxMotorForce",
                            "broken",
                        ],
                        Some("3") => &["type", "impulse", "u.x", "u.y", "length", "broken"],
                        Some("6") => &[
                            "type",
                            "force",
                            "mass",
                            "ratio",
                            "constant",
                            "J.linear1.x",
                            "J.linear1.y",
                            "J.angular1",
                            "J.linear2.x",
                            "J.linear2.y",
                            "J.angular2",
                            "broken",
                        ],
                        _ => &[],
                    };
                    append(&mut out, &format!("joints[{i}]"), joint, names);
                }
            }
            b'c' => append(
                &mut out,
                "contacts",
                header,
                &[
                    "count",
                    "normalImpulseSum",
                    "proxyCount",
                    "pairCount",
                    "pairBufferCount",
                    "lock",
                ],
            ),
            b'g' => append(
                &mut out,
                "game",
                header,
                &["dead", "pose", "iteration", "focus.x", "focus.y"],
            ),
            _ => out.push(("invalid".into(), section)),
        }
    }
    out
}

fn named_fields(s: &str) -> Vec<(String, &str)> {
    s.split_terminator(';')
        .map(|entry| {
            let (name, value) = entry.split_once('=').unwrap_or((entry, "<missing>"));
            (name.to_owned(), value)
        })
        .collect()
}
