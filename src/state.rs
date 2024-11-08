#[derive(PartialEq)]
pub enum State {
    HandshakePulse,
    HandshakeIrq,
    BeginSleep,
    ContinueSleep,
    Connected,
}
