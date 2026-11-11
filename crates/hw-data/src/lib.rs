//! Baked, measured constants: the display-object properties the game reads for each character
//! and level (post-Flash-quantisation bits, measured in the game; the recording probe is not part of
//! this repository), and the official TAS strings. Numbers are `[hi, lo]` words of an IEEE double.
#![no_std]

pub mod generated;
pub mod runtime_guides;
pub use runtime_guides::GuideKey;
pub use generated::{CHARACTERS, LEVELS, TAS_STRINGS};

pub type Words = [u32; 2];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeClass {
    Rectangle,
    Circle,
    Triangle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum SpecialKind {
    Van,
    Table,
    Mine,
    IBeam,
    Log,
    SpringBox,
    Spikes,
    WreckingBall,
    Fan,
    FinishLine,
    SoccerBall,
    Meteor,
    Boost,
    Building1,
    Building2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyRef {
    LevelBody,
    Shape(u32),
    Special(u32),
    Group(u32),
}

#[derive(Clone, Copy, Debug)]
pub struct ShapeRec {
    pub class: ShapeClass,
    pub x: Words,
    pub y: Words,
    pub rotation: Words,
    pub scale_x: Words,
    pub scale_y: Words,
    pub width: Words,
    pub height: Words,
    pub interactive: bool,
    pub immovable: bool,
    pub sleeping: bool,
    pub density: Words,
}

#[derive(Clone, Copy, Debug)]
pub struct SpecialRec {
    pub kind: SpecialKind,
    pub x: Words,
    pub y: Words,
    pub rotation: Words,
    pub scale_x: Words,
    pub scale_y: Words,
    pub width: Words,
    pub height: Words,
    pub immovable2: bool,
    pub num_spikes: Words,
    pub num_panels: Words,
    pub num_floors: Words,
    pub floor_width: Words,
    pub rope_length: Words,
    pub spring_delay: Words,
    pub shape_width: Words,
    pub shape_height: Words,
}

#[derive(Clone, Copy, Debug)]
pub struct GroupRec {
    pub x: Words,
    pub y: Words,
    pub rotation: Words,
    pub offset_x: Words,
    pub offset_y: Words,
    pub sleeping: bool,
    pub shapes_used: i32,
    pub shapes: &'static [ShapeRec],
    pub specials: &'static [SpecialRec],
}

#[derive(Clone, Copy, Debug)]
pub struct JointRec {
    pub x: Words,
    pub y: Words,
    pub limit: bool,
    pub upper_angle: Words,
    pub lower_angle: Words,
    pub motor: bool,
    pub torque: Words,
    pub speed: Words,
    pub collide_self: bool,
    pub body1: BodyRef,
    pub body2: BodyRef,
}

#[derive(Clone, Copy, Debug)]
pub struct LevelTable {
    pub id: u32,
    pub start_x: Words,
    pub start_y: Words,
    pub start_character: u32,
    pub bd_index: u32,
    pub shapes: &'static [ShapeRec],
    pub specials: &'static [SpecialRec],
    pub groups: &'static [GroupRec],
    pub joints: &'static [JointRec],
}

#[derive(Clone, Copy, Debug)]
pub struct CharPart {
    pub name: &'static str,
    pub index: u32,
    pub x: Words,
    pub y: Words,
    pub rotation: Words,
    pub scale_x: Words,
    pub scale_y: Words,
    pub width: Words,
    pub height: Words,
}

#[derive(Clone, Copy, Debug)]
pub struct CharTable {
    pub index: u32,
    pub guide: &'static [CharPart],
    pub kid_guide: &'static [CharPart],
    pub src: &'static [CharPart],
}

/// Numeric display-object properties, without names or borrowed data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct GuideNumbers {
    pub index: u32,
    pub x: Words,
    pub y: Words,
    pub rotation: Words,
    pub scale_x: Words,
    pub scale_y: Words,
    pub width: Words,
    pub height: Words,
}
impl From<&CharPart> for GuideNumbers {
    fn from(p: &CharPart) -> Self {
        Self { index: p.index, x: p.x, y: p.y, rotation: p.rotation,
               scale_x: p.scale_x, scale_y: p.scale_y, width: p.width, height: p.height }
    }
}

/// Static dispatch keeps callers' supplied tables and the baked constants equivalent.
pub trait GuideLookup {
    fn guide_numbers(&self, key: GuideKey) -> Option<GuideNumbers>;
}
impl GuideLookup for CharTable {
    #[inline]
    fn guide_numbers(&self, key: GuideKey) -> Option<GuideNumbers> {
        self.guide_part(key.name()).map(GuideNumbers::from)
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct RuntimeGuide { index: u32, child: u32 }
impl RuntimeGuide {
    #[inline]
    pub fn new(index: u32, child: bool) -> Option<Self> {
        if index >= 1 && index <= 4 { Some(Self { index, child: child as u32 }) }
        else { None }
    }
}
impl GuideLookup for RuntimeGuide {
    #[inline]
    fn guide_numbers(&self, key: GuideKey) -> Option<GuideNumbers> {
        runtime_guides::guide_numbers(self.index, self.child, key)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TasString {
    pub name: &'static str,
    pub file: &'static str,
    /// Stem of the recorded Flash traces (`reference/traces/flash32/tas-<trace>-*.jsonl`).
    pub trace: &'static str,
    pub level: u32,
    pub character: u32,
    pub frames: u32,
    /// '0'/'1' characters, 8 per frame: left, right, up, down, space, shift, ctrl, z.
    pub bits: &'static str,
}

#[inline]
pub fn level(id: u32) -> Option<&'static LevelTable> {
    LEVELS.iter().find(|l| l.id == id)
}

#[inline]
pub fn character(index: u32) -> Option<&'static CharTable> {
    CHARACTERS.iter().find(|c| c.index == index)
}

impl CharTable {
    #[inline]
    pub fn guide_part(&self, name: &str) -> Option<&'static CharPart> {
        self.guide.iter().find(|p| p.name == name)
    }
    #[inline]
    pub fn kid_part(&self, name: &str) -> Option<&'static CharPart> {
        self.kid_guide.iter().find(|p| p.name == name)
    }
}
