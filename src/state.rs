#[derive(PartialEq)]
pub enum State {
    Init,
    PulseCe,
    HandleIrq,
    BeginSleep,
    Connected,
}
