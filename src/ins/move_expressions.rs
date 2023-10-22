use core::fmt;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::fmt::Display;

use crate::{reg::registers::RegisterId, utils};

const VALUE_DECODE_MASK: u32 = 255;
const OPERATOR_DECODE_MASK: u32 = 3;
const VALUE_1_DECODE_SHIFT: u32 = 2;
const OPERATOR_DECODE_SHIFT: u32 = 10;
const VALUE_2_DECODE_SHIFT: u32 = 12;

#[allow(unused)]
const VALUE_1_ENCODE_MASK: u32 = 0b0000_0000_0000_1111_1111_1100_0000_0011;
const VALUE_1_ENCODE_SHIFT: u32 = 2;

#[allow(unused)]
const OPERATOR_ENCODE_MASK: u32 = 0b0000_0000_0000_1111_1111_0011_1111_1111;
const OPERATOR_ENCODE_SHIFT: u32 = 10;

#[allow(unused)]
const VALUE_2_ENCODE_MASK: u32 = 0b0000_0000_0000_0000_0000_1111_1111_1111;
const VALUE_2_ENCODE_SHIFT: u32 = 12;

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

    pub fn decode(&mut self, encoded: u32) {
        self.args.clear();

        let first_is_const = utils::is_bit_set(encoded, 0);
        let second_is_const = utils::is_bit_set(encoded, 1);

        let value_1 = ((encoded >> VALUE_1_DECODE_SHIFT) & VALUE_DECODE_MASK) as u8;
        let operator = ((encoded >> OPERATOR_DECODE_SHIFT) & OPERATOR_DECODE_MASK) as u8;
        let value_2 = ((encoded >> VALUE_2_DECODE_SHIFT) & VALUE_DECODE_MASK) as u8;

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

    pub fn encode(&mut self, args: Vec<ExpressionArgs>) -> u32 {
        self.args = args;
        assert!(self.validate());

        // Layout:
        // [BIT 0]   [BIT 1]   [BIT 2 - 5] [BIT 6 - 7] [BIT 8 - 11]
        // [TYPE_1]  [TYPE_2]  [TYPE 1]    [OPERATOR]  [TYPE 2]
        let mut encoded_value = 0u32;

        // If the first two bits are unset, the values will be assumed to be register IDs.
        if let ExpressionArgs::Constant(_) = self.args[0] {
            utils::set_bit_state_inline(&mut encoded_value, 0, true);
        }
        if let ExpressionArgs::Constant(_) = self.args[2] {
            utils::set_bit_state_inline(&mut encoded_value, 1, true);
        }

        // NOTE: we don't need to do the clearing mask here since we started from a value of zero.
        // NOTE: noted for posterity.

        // Encode the first operand.
        encoded_value |= match self.args[0] {
            ExpressionArgs::Register(rid) => rid as u32,
            ExpressionArgs::Constant(val) => val as u32,
            _ => panic!(),
        } << VALUE_1_ENCODE_SHIFT;

        // Encode the operator.
        encoded_value |= match self.args[1] {
            ExpressionArgs::Operator(oid) => oid as u32,
            _ => panic!(),
        } << OPERATOR_ENCODE_SHIFT;

        // Encode the second operand.
        encoded_value |= match self.args[2] {
            ExpressionArgs::Register(rid) => rid as u32,
            ExpressionArgs::Constant(val) => val as u32,
            _ => panic!(),
        } << VALUE_2_ENCODE_SHIFT;

        encoded_value
    }

    pub fn get_args_as_array(&self) -> [ExpressionArgs; 3] {
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

        write!(f, "{printable}")
    }
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, FromPrimitive)]
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
        write!(f, "{printable}")
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExpressionArgs {
    Register(RegisterId),
    Operator(ExpressionOperator),
    Constant(u8),
}

#[cfg(test)]
mod tests_move_expressions {
    use std::panic;

    use crate::{ins::move_expressions::MoveExpressionHandler, reg::registers::RegisterId};

    use super::{ExpressionArgs, ExpressionOperator};

    struct TestEntry<'a> {
        pub arguments: &'a [ExpressionArgs],
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl<'a> TestEntry<'a> {
        fn new(arguments: &'a [ExpressionArgs], should_panic: bool, fail_message: &str) -> Self {
            Self {
                arguments,
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        /// Run this specific test entry.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        pub fn run_test(&self, id: usize) {
            let result = panic::catch_unwind(|| {
                // Perform a roundtrip encode and decode.
                let mut handler = MoveExpressionHandler::new();
                let encoded = handler.encode(self.arguments.to_vec());
                handler.decode(encoded);

                // Yield the decoded arguments.
                handler.args
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                self.should_panic,
                "{}",
                self.fail_message(id, did_panic)
            );

            if let Ok(args) = result {
                assert_eq!(args, self.arguments, "{}", self.fail_message(id, false));
            }
        }

        /// Generate a fail message for a given test.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        /// * `did_panic` - Did the test panic?
        pub fn fail_message(&self, id: usize, did_panic: bool) -> String {
            format!(
                "Test {id} Failed - Should Panic? {}, Panicked? {did_panic}. Message = {}",
                self.should_panic, self.fail_message
            )
        }
    }

    /// Test the expression encoder and decoder via round-trips.
    #[test]
    fn test_round_trips() {
        let tests = [
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::R7),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R8),
                ],
                false,
                "failed to create register-register roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Constant(123),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(111),
                ],
                false,
                "failed to create constant-constant roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Constant(123),
                    ExpressionArgs::Operator(ExpressionOperator::Subtract),
                    ExpressionArgs::Register(RegisterId::R1),
                ],
                false,
                "failed to create constant-register roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Constant(123),
                ],
                false,
                "failed to create register-constant roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                ],
                true,
                "unexpected successful roundtrip with too few arguments",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Register(RegisterId::R1),
                ],
                true,
                "unexpected successful roundtrip with too many arguments",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Register(RegisterId::R1),
                ],
                true,
                "unexpected successful roundtrip with no operator",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Register(RegisterId::R1),
                ],
                true,
                "unexpected successful roundtrip with operator in first place",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                ],
                true,
                "unexpected successful in roundtrip with operator in last place",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }
}
