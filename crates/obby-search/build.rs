fn main() {
    // 32-bit process with large snapshot caches: allow the full 4 GB address space.
    println!("cargo:rustc-link-arg=/LARGEADDRESSAWARE");
}
