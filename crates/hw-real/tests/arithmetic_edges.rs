use hw_real::{as3, Real};

#[test]
fn original_arithmetic_edges() {
    let trace = include_str!("../../../reference/traces/flash32/arithmetic-edges-v1.jsonl");
    let mut differences = Vec::new();
    let mut count = 0;
    for line in trace.lines() {
        let row: serde_json::Value = serde_json::from_str(line).unwrap();
        let word = |name: &str| u64::from_str_radix(row[name].as_str().unwrap(), 16).unwrap();
        let a = f64::from_bits(word("a"));
        let b = f64::from_bits(word("b"));
        let frame = row["frame"].as_u64().unwrap();
        count += 1;
        for (field, value) in [
            ("add", a + b),
            ("sub", a - b),
            ("mul", a * b),
            ("div", a / b),
            ("sqrt", <f64 as Real>::sqrt(a)),
            ("neg", -a),
            ("abs", as3::math_abs(a)),
            ("floor", <f64 as Real>::floor(a)),
            ("ceil", <f64 as Real>::ceil(a)),
            ("round", as3::math_round(a)),
            ("min", as3::math_min(a, b)),
            ("max", as3::math_max(a, b)),
            ("uint_number", <f64 as Real>::from_u32(a.to_uint32())),
            ("int_number", <f64 as Real>::from_i32(a.to_int32())),
        ] {
            if value.to_bits() != word(field) {
                differences.push(format!(
                    "frame {frame} {field}: Flash {:016x}, Rust {:016x}",
                    word(field),
                    value.to_bits()
                ));
            }
        }
        assert_eq!(
            row["uint"].as_u64().unwrap(),
            a.to_uint32() as u64,
            "frame {frame}"
        );
        assert_eq!(
            row["int"].as_i64().unwrap(),
            a.to_int32() as i64,
            "frame {frame}"
        );
        for (field, value) in [
            ("lt", a.lt(b)),
            ("le", a.le(b)),
            ("gt", a.gt(b)),
            ("ge", a.ge(b)),
            ("eq", a.eq_num(b)),
        ] {
            assert_eq!(
                row[field].as_u64().unwrap(),
                value as u64,
                "frame {frame} {field}"
            );
        }
    }
    assert_eq!(count, 1024);
    assert!(
        differences.is_empty(),
        "{} mismatches:\n{}",
        differences.len(),
        differences
            .iter()
            .take(20)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n")
    );
}
