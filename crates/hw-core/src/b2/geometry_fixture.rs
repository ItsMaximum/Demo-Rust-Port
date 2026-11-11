//! Test-only raw-bit expectations measured by Box2dGeometryProbe in the original player.
use super::{math::Vec2, state::Manifold};
use std::{collections::BTreeMap, sync::OnceLock};

fn expected(field: &str) -> u64 {
    static VALUES: OnceLock<BTreeMap<String, u64>> = OnceLock::new();
    let values = VALUES.get_or_init(|| {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../reference/traces/flash32/b2-geometry-v1.jsonl"
        ))
        .unwrap();
        let mut values = BTreeMap::new();
        for line in text.lines() {
            let row: serde_json::Value = serde_json::from_str(line).unwrap();
            assert_eq!(row["schema_version"], 1);
            let name = row["field"].as_str().unwrap().to_owned();
            let bits = u64::from_str_radix(row["bits"].as_str().unwrap(), 16).unwrap();
            assert!(
                values.insert(name, bits).is_none(),
                "duplicate geometry field"
            );
        }
        values
    });
    *values
        .get(field)
        .unwrap_or_else(|| panic!("missing Flash geometry field {field}"))
}

pub fn number(field: &str, actual: f64) {
    let expected = expected(field);
    assert_eq!(
        actual.to_bits(),
        expected,
        "{field}: Flash {expected:016x}, Rust {:016x}",
        actual.to_bits()
    );
}

pub fn vector(field: &str, actual: Vec2<f64>) {
    number(&format!("{field}.x"), actual.x);
    number(&format!("{field}.y"), actual.y);
}

pub fn manifold(field: &str, actual: &Manifold<f64>) {
    number(&format!("{field}.point_count"), actual.point_count as f64);
    vector(&format!("{field}.normal"), actual.normal);
    for i in 0..actual.point_count as usize {
        let p = &actual.points[i];
        let field = format!("{field}.points[{i}]");
        number(&format!("{field}.separation"), p.separation);
        vector(&format!("{field}.local_point1"), p.local_point1);
        vector(&format!("{field}.local_point2"), p.local_point2);
        number(&format!("{field}.id"), p.id as f64);
    }
}
