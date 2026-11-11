//! Host x87 atan2 matching the original player. Portable emulation remains separate.

#[cfg(target_arch = "x86")]
#[inline(never)]
#[allow(dead_code)] // Also compiled into the diagnostic binary, which uses the bare body.
pub fn atan2_body(y: f64, x: f64) -> f64 {
    evaluate(y, x, false)
}

#[cfg(target_arch = "x86")]
pub fn atan2(y: f64, x: f64) -> f64 {
    evaluate(y, x, y.is_nan() || x.is_nan())
}

#[cfg(target_arch = "x86")]
fn evaluate(y: f64, x: f64, nan: bool) -> f64 {
    let y = y.to_bits();
    let x = x.to_bits();
    let mut slots = [
        y as u32,
        (y >> 32) as u32,
        x as u32,
        (x >> 32) as u32,
        0,
        0,
        0x133f_0000u32,
        nan as u32,
    ];
    // Every pointer addresses a live local; the block balances both x87 pushes
    // and restores the caller's control word. Status flags are not restored.
    unsafe {
        core::arch::asm!(
            "fnstcw word ptr [{p} + 24]",
            "fldcw word ptr [{p} + 26]",
            "fld qword ptr [{p}]",
            "fld qword ptr [{p} + 8]",
            "cmp dword ptr [{p} + 28], 0",
            "jne 4f",
            "fabs",
            "fxch st(1)",
            "fabs",
            "fxch st(1)",
            "fpatan",
            "test byte ptr [{p} + 15], 0x80",
            "jz 2f",
            "fldpi",
            "fsubrp st(1), st(0)",
            "2:",
            "test byte ptr [{p} + 7], 0x80",
            "jz 3f",
            "fchs",
            "3:",
            "jmp 5f",
            "4:",
            "faddp st(1), st(0)",
            "5:",
            "fstp qword ptr [{p} + 16]",
            "fldcw word ptr [{p} + 24]",
            p = in(reg) slots.as_mut_ptr(),
            out("st(0)") _,
            out("st(1)") _,
            options(nostack),
        );
    }
    f64::from_bits(((slots[5] as u64) << 32) | slots[4] as u64)
}
