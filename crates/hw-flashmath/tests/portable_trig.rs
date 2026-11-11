use hw_real::Real;

fn check<R: Real>() {
    let trace = include_str!("../../../reference/traces/flash32/math-v1.jsonl");
    let mut count = 0;
    let mut nonfinite = 0;
    let mut large = 0;
    for line in trace.lines() {
        let row: serde_json::Value = serde_json::from_str(line).unwrap();
        if !row["x"].is_string() {
            continue;
        }
        let words = |field: &str| {
            let b = u64::from_str_radix(row[field].as_str().unwrap(), 16).unwrap();
            [(b >> 32) as u32, b as u32]
        };
        let x = R::from_words(words("x"));
        let special = x.to_words()[0] & 0x7ff0_0000 == 0x7ff0_0000;
        if !special && !hw_flashmath::candidates::flash11_sse2::supported(x) {
            large += 1;
        }
        let (cos, sin) = hw_flashmath::portable::sin_cos(x);
        assert_eq!(sin.to_words(), words("sin"), "frame {} sin", row["frame"]);
        assert_eq!(cos.to_words(), words("cos"), "frame {} cos", row["frame"]);
        count += 1;
        nonfinite += special as usize;
    }
    assert_eq!((count, nonfinite, large), (6047, 3, 1764));
}

#[test]
fn native_all_trig() {
    check::<f64>();
}
