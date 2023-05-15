#[cfg(windows)]
extern crate winres;

fn main() {
    let sub_dir = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };

    // Copy the paths.json file to the correct build directory.
    if std::fs::copy("paths.json", format!("target/{sub_dir}/paths.json")).is_err() {
        eprintln!("Error attempting to copy the paths JSON file.");
    }

    #[cfg(windows)]
    windows_only();
}

#[cfg(windows)]
fn windows_only() {
    /*let mut res = winres::WindowsResource::new();
    res.set_icon("..//assets//icon.ico");
    res.compile().unwrap();*/
}
