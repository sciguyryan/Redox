use crate::reg::registers::RegisterId;

#[derive(Clone, Debug)]
pub struct JumpExpression {
    /// The base register ID associated with this expression.
    pub base_register: RegisterId,
    /// The 32-bit value by which the value of the specific register should be offset in the jump.
    pub offset: u32,
}
