use core::fmt;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::fmt::Display;

use crate::{reg::registers::RegisterId, utils};

#[derive(Clone, Debug)]
pub struct MoveExpressionHandler {
    args: Vec<ExpressionArgs>,
}

impl MoveExpressionHandler {
    pub fn new() -> Self {
        Self {
            args: Vec::with_capacity(3),
        }
    }

    pub fn decode(&mut self, encoded: u16) {
        self.args.clear();

        let first_is_const = utils::is_bit_set_u16(encoded, 0);
        let second_is_const = utils::is_bit_set_u16(encoded, 1);

        // Read the first value.
        let mut value_1 = 0u8;
        utils::set_bit_state_u8_inline(&mut value_1, 0, utils::is_bit_set_u16(encoded, 2));
        utils::set_bit_state_u8_inline(&mut value_1, 1, utils::is_bit_set_u16(encoded, 3));
        utils::set_bit_state_u8_inline(&mut value_1, 2, utils::is_bit_set_u16(encoded, 4));
        utils::set_bit_state_u8_inline(&mut value_1, 3, utils::is_bit_set_u16(encoded, 5));

        // Next, read the operator.
        let mut operator = 0u8;
        utils::set_bit_state_u8_inline(&mut operator, 0, utils::is_bit_set_u16(encoded, 6));
        utils::set_bit_state_u8_inline(&mut operator, 1, utils::is_bit_set_u16(encoded, 7));

        // Finally, read the second value.
        let mut value_2 = 0u8;
        utils::set_bit_state_u8_inline(&mut value_2, 0, utils::is_bit_set_u16(encoded, 8));
        utils::set_bit_state_u8_inline(&mut value_2, 1, utils::is_bit_set_u16(encoded, 9));
        utils::set_bit_state_u8_inline(&mut value_2, 2, utils::is_bit_set_u16(encoded, 10));
        utils::set_bit_state_u8_inline(&mut value_2, 3, utils::is_bit_set_u16(encoded, 11));

        // Next we need to cast and check each of these entries.
        if first_is_const {
            self.args.push(ExpressionArgs::Constant(value_1));
        } else {
            let reg_id: RegisterId =
                FromPrimitive::from_u8(value_1).expect("failed to get valid register id");
            self.args.push(ExpressionArgs::Register(reg_id));
        }

        let operator_id: ExpressionOperator =
            FromPrimitive::from_u8(operator).expect("failed to get valid operator id");
        self.args.push(ExpressionArgs::Operator(operator_id));

        if second_is_const {
            self.args.push(ExpressionArgs::Constant(value_2));
        } else {
            let reg_id: RegisterId =
                FromPrimitive::from_u8(value_2).expect("failed to get valid register id");
            self.args.push(ExpressionArgs::Register(reg_id));
        }

        // Validate that the read information actually makes sense.
        assert!(self.validate());
    }

    pub fn encode(&mut self, args: &[ExpressionArgs]) -> u16 {
        self.args = args.to_vec();
        assert!(self.validate());

        // Layout:
        // [BIT 0]   [BIT 1]   [BIT 2 - 5] [BIT 6 - 7] [BIT 8 - 11]
        // [TYPE_1]  [TYPE_2]  [TYPE 1]    [OPERATOR]  [TYPE 2]
        let mut value = 0u16;

        // If the first two bits are unset, the values will be assumed to be register IDs.
        if let ExpressionArgs::Constant(_) = args[0] {
            utils::set_bit_state_u16_inline(&mut value, 0, true);
        }
        if let ExpressionArgs::Constant(_) = args[2] {
            utils::set_bit_state_u16_inline(&mut value, 1, true);
        }

        let mut bit = 2;
        for o in args {
            match o {
                ExpressionArgs::Register(id) => {
                    let rid = *id as u16;
                    utils::set_bit_state_u16_inline(&mut value, bit, utils::is_bit_set_u16(rid, 0));
                    utils::set_bit_state_u16_inline(
                        &mut value,
                        bit + 1,
                        utils::is_bit_set_u16(rid, 1),
                    );
                    utils::set_bit_state_u16_inline(
                        &mut value,
                        bit + 2,
                        utils::is_bit_set_u16(rid, 2),
                    );
                    utils::set_bit_state_u16_inline(
                        &mut value,
                        bit + 3,
                        utils::is_bit_set_u16(rid, 3),
                    );

                    bit += 4;
                }
                ExpressionArgs::Operator(op) => {
                    let oid = *op as u16;
                    utils::set_bit_state_u16_inline(&mut value, bit, utils::is_bit_set_u16(oid, 0));
                    utils::set_bit_state_u16_inline(
                        &mut value,
                        bit + 1,
                        utils::is_bit_set_u16(oid, 1),
                    );

                    bit += 2;
                }
                ExpressionArgs::Constant(val) => {
                    let v = *val as u16;
                    utils::set_bit_state_u16_inline(&mut value, bit, utils::is_bit_set_u16(v, 0));
                    utils::set_bit_state_u16_inline(
                        &mut value,
                        bit + 1,
                        utils::is_bit_set_u16(v, 1),
                    );
                    utils::set_bit_state_u16_inline(
                        &mut value,
                        bit + 2,
                        utils::is_bit_set_u16(v, 2),
                    );
                    utils::set_bit_state_u16_inline(
                        &mut value,
                        bit + 3,
                        utils::is_bit_set_u16(v, 3),
                    );

                    bit += 4;
                }
            }
        }

        value
    }

    pub fn get_as_array(&self) -> [ExpressionArgs; 3] {
        [self.args[0], self.args[1], self.args[2]]
    }

    pub fn validate(&self) -> bool {
        self.args.len() == 3 && matches!(self.args[1], ExpressionArgs::Operator(_))
    }
}

impl Display for MoveExpressionHandler {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut printable = String::new();

        for arg in &self.args {
            match arg {
                ExpressionArgs::Register(id) => printable.push_str(&id.to_string()),
                ExpressionArgs::Operator(id) => printable.push_str(&id.to_string()),
                ExpressionArgs::Constant(val) => printable.push_str(&val.to_string()),
            }
        }

        write!(f, "{}", printable)
    }
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, FromPrimitive)]
pub enum ExpressionOperator {
    Add,
    Subtract,
    Multiply,
}

impl Display for ExpressionOperator {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let printable = match *self {
            ExpressionOperator::Add => "+",
            ExpressionOperator::Subtract => "-",
            ExpressionOperator::Multiply => "*",
        };
        write!(f, "{}", printable)
    }
}

#[derive(Clone, Copy, Debug)]
pub enum ExpressionArgs {
    Register(RegisterId),
    Operator(ExpressionOperator),
    Constant(u8),
}
