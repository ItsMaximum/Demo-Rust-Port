use hw_data::{GuideKey, GuideLookup, GuideNumbers, RuntimeGuide};

#[test]
fn runtime_guides_match_all_named_measured_properties() {
    for index in 1..=4 {
        let original = hw_data::character(index).unwrap();
        for child in [false, true] {
            let runtime = RuntimeGuide::new(index, child).unwrap();
            for key in GuideKey::ALL {
                let named = if child { original.kid_part(key.name()) }
                            else { original.guide_part(key.name()) };
                assert_eq!(runtime.guide_numbers(key), named.map(GuideNumbers::from),
                           "character {index}, child={child}, key={key:?}");
            }
        }
    }
    for index in [0, 5, u32::MAX] {
        assert!(RuntimeGuide::new(index, false).is_none());
        assert!(RuntimeGuide::new(index, true).is_none());
    }
}
