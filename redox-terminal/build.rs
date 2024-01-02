fn main() {
    let _sub_dir = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };

    #[cfg(windows)]
    windows_only();
}

#[cfg(windows)]
fn windows_only() {
    extern crate winres;

    /*let mut res = winres::WindowsResource::new();
    res.set_icon("..//assets//icon.ico");
    res.compile().unwrap();*/
}
