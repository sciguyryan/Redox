use core::fmt;
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
pub struct Expression {
    /// Is the expression a simple expression or an extended one?
    pub is_extended: bool,

    /// The first operator, must always be included.
    operator_1: ExpressionArgs,
    /// The second operator, only applicable when dealing with an extended expression.
    operator_2: ExpressionArgs,

    /// The first operand, must always be included.
    pub operand_1: ExpressionArgs,
    /// The second operand, must always be included.
    pub operand_2: ExpressionArgs,
    /// The third operand, only applicable when dealing with an extended expression.
    pub operand_3: ExpressionArgs,
}

impl Expression {
    pub fn new() -> Self {
        Self {
            is_extended: false,
            operator_1: ExpressionArgs::Operator(ExpressionOperator::default()),
            operator_2: ExpressionArgs::Operator(ExpressionOperator::default()),
            operand_1: ExpressionArgs::Immediate(0),
            operand_2: ExpressionArgs::Immediate(0),
            operand_3: ExpressionArgs::Immediate(0),
        }
    }

    /// Return this expression object as a vector of [`ExpressionArgs`] objects.
    ///
    /// # Returns
    ///
    /// A vector containing the resulting [`ExpressionArgs`] objects in the correct order.
    fn as_vector(&self) -> Vec<ExpressionArgs> {
        let mut result = vec![self.operand_1, self.operator_1, self.operand_2];

        if self.is_extended {
            result.push(self.operator_2);
            result.push(self.operand_3);
        }

        result
    }

    /// Evaluate this move expression based on the supplied arguments.
    ///
    /// # Arguments
    ///
    /// * `value_1` - The value of the first operand.
    /// * `value_2` - The value of the second operand.
    /// * `value_3` - The value of the third operand, only used with an extended expression.
    ///
    /// # Returns
    ///
    /// A u32 that is the calculated result of the expression.
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

    /// Pack this move expression into its encoded format.
    #[allow(dead_code)]
    pub fn pack(&self) -> u32 {
        // Layout:
        // [BIT 0]      [BIT 1]      [BIT 2]      [BIT 3]       [BIT 4-5]  [BIT 6-7]  [BIT 8-15]  [BIT 16-23]  [BIT 24-31]
        // [EXTENDED?]  [IS 1 IMM?]  [IS 2 IMM?]  [IS 2 IMM?*]  [OP 1]     [OP 2*]    [VALUE 1]   [VALUE 2]   [VALUE 3*]
        // Note, any entry with an * is only applicable in the instances where we are dealing with an
        // extended expression, otherwise the values should be ignored.
        let mut packed = 0u32;

        // The first things we need to encode are the status flags. These are the simplest parts
        // and they come first.
        if self.is_extended {
            packed |= IS_EXTENDED_MASK;
        }
        if matches!(self.operand_1, ExpressionArgs::Immediate(_)) {
            packed |= IS_VALUE_1_IMM_MASK;
        }
        if matches!(self.operand_2, ExpressionArgs::Immediate(_)) {
            packed |= IS_VALUE_2_IMM_MASK;
        }
        if matches!(self.operand_3, ExpressionArgs::Immediate(_)) {
            packed |= IS_VALUE_3_IMM_MASK;
        }

        // Next, encode the operator IDs.
        packed |= self.operator_1.get_as_value() << OPERATOR_1_SHIFT;
        packed |= self.operator_2.get_as_value() << OPERATOR_2_SHIFT;

        // Finally, encode the operands.
        packed |= self.operand_1.get_as_value() << VALUE_1_SHIFT;
        packed |= self.operand_2.get_as_value() << VALUE_2_SHIFT;
        packed |= self.operand_3.get_as_value() << VALUE_3_SHIFT;

        packed
    }

    /// Unpack a packed move expression.
    ///
    /// # Arguments
    ///
    /// * `packed` - The encoded move expression.
    pub fn unpack(&mut self, packed: u32) {
        self.is_extended = (packed & IS_EXTENDED_MASK) != 0;

        let is_argument_1_immediate = (packed & IS_VALUE_1_IMM_MASK) != 0;
        let is_argument_2_immediate = (packed & IS_VALUE_2_IMM_MASK) != 0;
        let is_argument_3_immediate = (packed & IS_VALUE_3_IMM_MASK) != 0;

        self.operator_1 = ExpressionArgs::Operator(ExpressionOperator::from(
            ((packed >> OPERATOR_1_SHIFT) & OPERATOR_MASK) as u8,
        ));
        self.operator_2 = ExpressionArgs::Operator(ExpressionOperator::from(
            ((packed >> OPERATOR_2_SHIFT) & OPERATOR_MASK) as u8,
        ));

        let value_1 = (packed >> VALUE_1_SHIFT) & VALUE_MASK;
        self.operand_1 = if is_argument_1_immediate {
            ExpressionArgs::Immediate(value_1 as u8)
        } else {
            ExpressionArgs::Register(RegisterId::from(value_1 as u8))
        };

        let value_2 = (packed >> VALUE_2_SHIFT) & VALUE_MASK;
        self.operand_2 = if is_argument_2_immediate {
            ExpressionArgs::Immediate(value_2 as u8)
        } else {
            ExpressionArgs::Register(RegisterId::from(value_2 as u8))
        };

        let value_3 = (packed >> VALUE_3_SHIFT) & VALUE_MASK;
        self.operand_3 = if is_argument_3_immediate {
            ExpressionArgs::Immediate(value_3 as u8)
        } else {
            ExpressionArgs::Register(RegisterId::from(value_3 as u8))
        };
    }
}

impl Default for Expression {
    fn default() -> Self {
        Self::new()
    }
}

impl TryFrom<&[ExpressionArgs]> for Expression {
    type Error = ();

    fn try_from(args: &[ExpressionArgs]) -> Result<Self, Self::Error> {
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

        Ok(Expression {
            is_extended: len == 5,
            operator_1: args[1],
            operator_2,
            operand_1: args[0],
            operand_2: args[2],
            operand_3: argument_3,
        })
    }
}

impl Display for Expression {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut parts = vec![];
        for arg in self.as_vector() {
            let part = match arg {
                ExpressionArgs::Register(id) => format!("{id}"),
                ExpressionArgs::Operator(id) => format!("{id}"),
                ExpressionArgs::Immediate(imm) => format!("0x{imm:02x}"),
            };

            parts.push(part);
        }

        write!(f, "{}", parts.join(""))
    }
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ExpressionOperator {
    #[default]
    Add = 0,
    Subtract = 1,
    Multiply = 2,
}

impl From<u8> for ExpressionOperator {
    fn from(value: u8) -> Self {
        match value {
            0 => ExpressionOperator::Add,
            1 => ExpressionOperator::Subtract,
            2 => ExpressionOperator::Multiply,
            id => panic!("invalid expression operator id {id}"),
        }
    }
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

    use crate::{ins::expressions::Expression, reg::registers::RegisterId};

    use super::{ExpressionArgs, ExpressionOperator};

    struct TestEntry<'a> {
        pub arguments: &'a [ExpressionArgs],
        pub evaluation_arguments: &'a [u32],
        pub evaluation_results: u32,
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl<'a> TestEntry<'a> {
        fn new(
            arguments: &'a [ExpressionArgs],
            evaluation_arguments: &'a [u32],
            evaluation_results: u32,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            Self {
                arguments,
                evaluation_arguments,
                evaluation_results,
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
                let encoded = Expression::try_from(&self.arguments.to_vec()[..])
                    .expect("")
                    .pack();

                let mut decoded = Expression::new();
                decoded.unpack(encoded);

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

        /// Run this specific test entry by evaluation.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        pub fn run_test_valuation(&self, id: usize) {
            let handler = Expression::try_from(&self.arguments.to_vec()[..]).expect("");

            let evaluation_result = if self.arguments.len() == 3 {
                handler.evaluate(
                    self.evaluation_arguments[0],
                    self.evaluation_arguments[1],
                    0,
                )
            } else {
                handler.evaluate(
                    self.evaluation_arguments[0],
                    self.evaluation_arguments[1],
                    self.evaluation_arguments[2],
                )
            };

            assert_eq!(
                evaluation_result,
                self.evaluation_results,
                "{}",
                self.fail_message(id, false)
            );
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
                    ExpressionArgs::Register(RegisterId::ER7),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::ER8),
                ],
                &[],
                0,
                false,
                "failed to create register-register roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Immediate(123),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Immediate(111),
                ],
                &[],
                0,
                false,
                "failed to create constant-constant roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Immediate(123),
                    ExpressionArgs::Operator(ExpressionOperator::Subtract),
                    ExpressionArgs::Register(RegisterId::ER1),
                ],
                &[],
                0,
                false,
                "failed to create constant-register roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::ER1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Immediate(123),
                ],
                &[],
                0,
                false,
                "failed to create register-constant roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::ER1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                ],
                &[],
                0,
                true,
                "unexpected successful roundtrip with too few arguments",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::ER1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::ER1),
                    ExpressionArgs::Register(RegisterId::ER1),
                ],
                &[],
                0,
                true,
                "unexpected successful roundtrip with too many arguments",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::ER1),
                    ExpressionArgs::Register(RegisterId::ER1),
                    ExpressionArgs::Register(RegisterId::ER1),
                ],
                &[],
                0,
                true,
                "unexpected successful roundtrip with no operator",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::ER1),
                    ExpressionArgs::Register(RegisterId::ER1),
                ],
                &[],
                0,
                true,
                "unexpected successful roundtrip with operator in first place",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::ER1),
                    ExpressionArgs::Register(RegisterId::ER1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                ],
                &[],
                0,
                true,
                "unexpected successful in roundtrip with operator in last place",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::ER7),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::ER8),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::ER6),
                ],
                &[],
                0,
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
                &[],
                0,
                false,
                "failed to create constant-constant-constant roundtrip",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Register(RegisterId::ER7),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::ER8),
                    ExpressionArgs::Register(RegisterId::ER6),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                ],
                &[],
                0,
                true,
                "unexpected successful in roundtrip with operator in invalid position",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the expression evaluator.
    #[test]
    fn test_evaluation() {
        let tests = [
            TestEntry::new(
                &[
                    ExpressionArgs::Immediate(2),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Immediate(100),
                ],
                &[2, 100],
                200,
                false,
                "failed to correctly evaluate expression result",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Immediate(2),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Immediate(100),
                ],
                &[2, 100],
                102,
                false,
                "failed to correctly evaluate expression result",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Immediate(100),
                    ExpressionArgs::Operator(ExpressionOperator::Subtract),
                    ExpressionArgs::Immediate(2),
                ],
                &[100, 2],
                98,
                false,
                "failed to correctly evaluate expression result",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Immediate(100),
                    ExpressionArgs::Operator(ExpressionOperator::Subtract),
                    ExpressionArgs::Immediate(2),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Immediate(8),
                ],
                &[100, 2, 8],
                84,
                false,
                "failed to correctly evaluate expression result",
            ),
            TestEntry::new(
                &[
                    ExpressionArgs::Immediate(100),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Immediate(2),
                    ExpressionArgs::Operator(ExpressionOperator::Subtract),
                    ExpressionArgs::Immediate(8),
                ],
                &[100, 2, 8],
                192,
                false,
                "failed to correctly evaluate expression result",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test_valuation(id);
        }
    }
}
