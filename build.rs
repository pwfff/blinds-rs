// Necessary because of this issue: https://github.com/rust-lang/cargo/issues/9641
fn main() {
    embuild::espidf::sysenv::output()
    //embuild::build::CfgArgs::output_propagated("ESP_IDF_SVC")?;
    //embuild::build::LinkArgs::output_propagated("ESP_IDF_SVC")?;
    //Ok(())
}
