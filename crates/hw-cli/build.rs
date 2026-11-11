fn main() {
    // hw-search keeps snapshots and one context per worker: give the 32-bit process 4 GB.
    let target = std::env::var("TARGET").unwrap_or_default();
    if target == "i686-pc-windows-msvc" {
        println!("cargo:rustc-link-arg-bin=hw-search=/LARGEADDRESSAWARE");
    }
}
