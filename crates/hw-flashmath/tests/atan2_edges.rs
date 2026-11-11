use hw_real::Real;

fn check<R: Real>() {
    let trace = include_str!("../../../reference/traces/flash32/atan2-edges-v1.jsonl");
    let mut count = 0;
    for line in trace.lines() {
        let row: serde_json::Value = serde_json::from_str(line).unwrap();
        let words = |field: &str| {
            let bits = u64::from_str_radix(row[field].as_str().unwrap(), 16).unwrap();
            [(bits >> 32) as u32, bits as u32]
        };
        let y = R::from_words(words("a"));
        let x = R::from_words(words("b"));
        let actual = hw_flashmath::atan2(y, x).to_words();
        assert_eq!(
            actual,
            words("atan2"),
            "frame {} y={} x={}",
            row["frame"],
            row["a"],
            row["b"]
        );
        assert_eq!(
            hw_flashmath::portable::atan2(y, x).to_words(),
            words("atan2"),
            "portable frame {} y={} x={}",
            row["frame"],
            row["a"],
            row["b"]
        );
        count += 1;
    }
    assert_eq!(count, 1444);
}

#[test]
fn native_atan2_edges() {
    check::<f64>();
}

fn original_math<R: Real>() {
    let trace = include_str!("../../../reference/traces/flash32/math-v1.jsonl");
    let mut count = 0;
    for line in trace.lines() {
        let row: serde_json::Value = serde_json::from_str(line).unwrap();
        if !row["atan2"].is_string() {
            continue;
        }
        let words = |field: &str| {
            let bits = u64::from_str_radix(row[field].as_str().unwrap(), 16).unwrap();
            [(bits >> 32) as u32, bits as u32]
        };
        assert_eq!(
            hw_flashmath::atan2(R::from_words(words("y")), R::from_words(words("x"))).to_words(),
            words("atan2"),
            "frame {}",
            row["frame"]
        );
        assert_eq!(
            hw_flashmath::portable::atan2(R::from_words(words("y")), R::from_words(words("x")))
                .to_words(),
            words("atan2"),
            "portable frame {}",
            row["frame"]
        );
        count += 1;
    }
    assert_eq!(count, 6047);
}

#[test]
fn native_original_math_atan2() {
    original_math::<f64>();
}

#[test]
fn portable_x87_load_quiets_nan_before_payload_comparison() {
    // Host x87 differential regression, not an additional Flash trace row.
    // y is a negative quiet NaN; x has the same payload and is signaling.
    let y = f64::from_bits(0xfff9_9882_4694_f248);
    let x = f64::from_bits(0x7ff1_9882_4694_f248);
    let expected = 0x7ff9_9882_4694_f248;
    assert_eq!(hw_flashmath::portable::atan2(y, x).to_bits(), expected);
}
