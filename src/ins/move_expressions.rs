use core::fmt;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::fmt::Display;

use crate::reg::registers::RegisterId;

const OPERATOR_1_SHIFT: u32 = 4;
const OPERATOR_2_SHIFT: u32 = 6;
const VALUE_1_SHIFT: u32 = 8;
const VALUE_2_SHIFT: u32 = 16;
const VALUE_3_SHIFT: u32 = 24;

const IS_EXTENDED_MASK: u32 = 0b0001;
const IS_VALUE_1_IMM_MASK: u32 = 0b0010;
const IS_VALUE_2_IMM_MASK: u32 = 0b0100;
const IS_VALUE_3_IMM_MASK: u32 = 0b1000;
const OPERATOR_MASK: u32 = 0b00000011;
const VALUE_MASK: u32 = 0b11111111;

#[derive(Clone, Debug)]
pub struct MoveExpressionHandler {
    pub is_extended: bool,

    operator_1: ExpressionArgs,
    operator_2: ExpressionArgs,

    pub argument_1: ExpressionArgs,
    pub argument_2: ExpressionArgs,
    pub argument_3: ExpressionArgs,
}

impl MoveExpressionHandler {
    pub fn new() -> Self {
        Self {
            is_extended: false,
            operator_1: ExpressionArgs::Operator(ExpressionOperator::default()),
            operator_2: ExpressionArgs::Operator(ExpressionOperator::default()),
            argument_1: ExpressionArgs::Immediate(0),
            argument_2: ExpressionArgs::Immediate(0),
            argument_3: ExpressionArgs::Immediate(0),
        }
    }

    pub fn as_vector(&self) -> Vec<ExpressionArgs> {
        let mut result = vec![self.argument_1, self.operator_1, self.argument_2];

        if self.is_extended {
            result.push(self.operator_2);
            result.push(self.argument_3);
        }

        result
    }

    pub fn decode(&mut self, encoded: u32) {
        self.is_extended = (encoded & IS_EXTENDED_MASK) != 0;

        let is_argument_1_immediate = (encoded & IS_VALUE_1_IMM_MASK) != 0;
        let is_argument_2_immediate = (encoded & IS_VALUE_2_IMM_MASK) != 0;
        let is_argument_3_immediate = (encoded & IS_VALUE_3_IMM_MASK) != 0;

        self.operator_1 = ExpressionArgs::Operator(
            FromPrimitive::from_u32((encoded >> OPERATOR_1_SHIFT) & OPERATOR_MASK)
                .expect("failed to get valid operator id"),
        );
        self.operator_2 = ExpressionArgs::Operator(
            FromPrimitive::from_u32((encoded >> OPERATOR_2_SHIFT) & OPERATOR_MASK)
                .expect("failed to get valid operator id"),
        );

        let value_1 = (encoded >> VALUE_1_SHIFT) & VALUE_MASK;
        self.argument_1 = if is_argument_1_immediate {
            ExpressionArgs::Immediate(value_1 as u8)
        } else {
            ExpressionArgs::Register(
                FromPrimitive::from_u32(value_1).expect("failed to get valid register id"),
            )
        };

        let value_2 = (encoded >> VALUE_2_SHIFT) & VALUE_MASK;
        self.argument_2 = if is_argument_2_immediate {
            ExpressionArgs::Immediate(value_2 as u8)
        } else {
            ExpressionArgs::Register(
                FromPrimitive::from_u32(value_2).expect("failed to get valid register id"),
            )
        };

        let value_3 = (encoded >> VALUE_3_SHIFT) & VALUE_MASK;
        self.argument_3 = if is_argument_3_immediate {
            ExpressionArgs::Immediate(value_3 as u8)
        } else {
            ExpressionArgs::Register(
                FromPrimitive::from_u32(value_3).expect("failed to get valid register id"),
            )
        };
    }

    pub fn encode(&self) -> u32 {
        // Layout:
        // [BIT 0]      [BIT 1]      [BIT 2]      [BIT 3]       [BIT 4-5]  [BIT 6-7]  [BIT 8-15]  [BIT 16-23]  [BIT 24-31]
        // [EXTENDED?]  [IS 1 IMM?]  [IS 2 IMM?]  [IS 2 IMM?*]  [OP 1]     [OP 2*]    [VALUE 1]   [VALUE 2]   [VALUE 3*]
        // Note, any entry with an * is only applicable in the instances where we are dealing with an
        // extended expression, otherwise the values should be ignored.
        let mut encoded_value = 0u32;

        // The first things we need to encode are the status flags. These are the simplest parts
        // and they come first.
        if self.is_extended {
            encoded_value |= IS_EXTENDED_MASK;
        }
        if matches!(self.argument_1, ExpressionArgs::Immediate(_)) {
            encoded_value |= IS_VALUE_1_IMM_MASK;
        }
        if matches!(self.argument_2, ExpressionArgs::Immediate(_)) {
            encoded_value |= IS_VALUE_2_IMM_MASK;
        }
        if matches!(self.argument_3, ExpressionArgs::Immediate(_)) {
            encoded_value |= IS_VALUE_3_IMM_MASK;
        }

        // Next, encode the operator IDs.
        encoded_value |= self.operator_1.get_as_value() << OPERATOR_1_SHIFT;
        encoded_value |= self.operator_2.get_as_value() << OPERATOR_2_SHIFT;

        // Finally, encode the operands.
        encoded_value |= self.argument_1.get_as_value() << VALUE_1_SHIFT;
        encoded_value |= self.argument_2.get_as_value() << VALUE_2_SHIFT;
        encoded_value |= self.argument_3.get_as_value() << VALUE_3_SHIFT;

        encoded_value
    }

    pub fn evaluate(&self, value_1: u32, value_2: u32, value_3: u32) -> u32 {
        let op_1 = match self.operator_1 {
            ExpressionArgs::Operator(op) => op,
            _ => panic!(),
        };

        // We don't need to worry about the argument ordering here
        // as the arguments can always be evaluated left-to-right.
        if !self.is_extended {
            return match op_1 {
                ExpressionOperator::Add => value_1 + value_2,
                ExpressionOperator::Subtract => value_1 - value_2,
                ExpressionOperator::Multiply => value_1 * value_2,
            };
        }

        let op_2 = match self.operator_2 {
            ExpressionArgs::Operator(op) => op,
            _ => panic!(),
        };

        // In this case we need to pay attention to the ordering precedence of the
        // arguments, specifically this depends on the priority of the operators.
        if matches!(op_2, ExpressionOperator::Multiply) {
            // We need to calculate the second set of arguments first.
            let temp = value_2 * value_3;

            // Now we can finish the calculation.
            return match op_1 {
                ExpressionOperator::Add => value_1 + temp,
                ExpressionOperator::Subtract => value_1 - temp,
                ExpressionOperator::Multiply => value_1 * temp,
            };
        }

        // We need to calculate the first set of arguments first.
        let temp = match op_1 {
            ExpressionOperator::Add => value_1 + value_2,
            ExpressionOperator::Subtract => value_1 - value_2,
            ExpressionOperator::Multiply => value_1 * value_2,
        };

        match op_2 {
            ExpressionOperator::Add => temp + value_3,
            ExpressionOperator::Subtract => temp - value_3,
            ExpressionOperator::Multiply => temp * value_3,
        }
    }
}

impl From<&[ExpressionArgs]> for MoveExpressionHandler {
    fn from(args: &[ExpressionArgs]) -> Self {
        let len = args.len();
        assert!(len == 3 || len == 5);

        assert!(matches!(
            args[0],
            ExpressionArgs::Register(_) | ExpressionArgs::Immediate(_)
        ));
        assert!(matches!(args[1], ExpressionArgs::Operator(_)));
        assert!(matches!(
            args[2],
            ExpressionArgs::Register(_) | ExpressionArgs::Immediate(_)
        ));

        if len == 5 {
            assert!(matches!(args[3], ExpressionArgs::Operator(_)));
            assert!(matches!(
                args[4],
                ExpressionArgs::Register(_) | ExpressionArgs::Immediate(_)
            ));
        }

        let operator_2 = if len == 3 {
            ExpressionArgs::Operator(ExpressionOperator::default())
        } else {
            args[3]
        };

        let argument_3 = if len == 3 {
            ExpressionArgs::Immediate(0)
        } else {
            args[4]
        };

        MoveExpressionHandler {
            is_extended: len == 5,
            operator_1: args[1],
            operator_2,
            argument_1: args[0],
            argument_2: args[2],
            argument_3,
        }
    }
}

impl Display for MoveExpressionHandler {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut parts = vec![];
        for arg in self.as_vector() {
            let part = match arg {
                ExpressionArgs::Register(id) => format!("%{id}"),
                ExpressionArgs::Operator(id) => format!("{id}"),
                ExpressionArgs::Immediate(imm) => format!("%{imm:04x}"),
            };

            parts.push(part);
        }

        write!(f, "{}", parts.join(" "))
    }
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, FromPrimitive)]
pub enum ExpressionOperator {
    #[default]
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
    Immediate(u8),
}

impl ExpressionArgs {
    pub fn get_as_value(&self) -> u32 {
        match self {
            ExpressionArgs::Register(id) => *id as u32,
            ExpressionArgs::Operator(id) => *id as u32,
            ExpressionArgs::Immediate(imm) => *imm as u32,
        }
    }
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
                let encoded = MoveExpressionHandler::from(&self.arguments.to_vec()[..]).encode();

                let mut decoded = MoveExpressionHandler::new();
                decoded.decode(encoded);

                // Yield the decoded arguments.
                decoded.as_vector()
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
                    ExpressionArgs::Immediate(123),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Immediate(111),
                ],
                false,
                "failed to create constant-constant roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Immediate(123),
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
                    ExpressionArgs::Immediate(123),
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
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::R7),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R8),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R6),
                ],
                false,
                "failed to create register-register-register roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Immediate(123),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Immediate(111),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Immediate(64),
                ],
                false,
                "failed to create constant-constant-constant roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::R7),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R8),
                    ExpressionArgs::Register(RegisterId::R6),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                ],
                true,
                "unexpected successful in roundtrip with operator in invalid position",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }
}
