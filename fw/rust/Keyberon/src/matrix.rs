//! Hardware pin switch matrix handling.

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

/// Describes the hardware-level matrix of switches.
///
/// Generic parameters are in order: The type of column pins,
/// the type of row pins, the number of columns and rows.
/// **NOTE:** In order to be able to put different pin structs
/// in an array they have to be downgraded (stripped of their
/// numbers etc.). Most HAL-s have a method of downgrading pins
/// to a common (erased) struct. (for example see
/// [stm32f0xx_hal::gpio::PA0::downgrade](https://docs.rs/stm32f0xx-hal/0.17.1/stm32f0xx_hal/gpio/gpioa/struct.PA0.html#method.downgrade))
pub struct Matrix<C, R, const CS: usize, const RS: usize>
where
    C: OutputPin,
    R: InputPin,
{
    cols: [C; CS],
    rows: [R; RS],
}

impl<C, R, const CS: usize, const RS: usize> Matrix<C, R, CS, RS>
where
    C: OutputPin,
    R: InputPin,
{
    /// Creates a new Matrix.
    ///
    /// Assumes columns are outputs,
    /// and rows are input pins which are set low when not being scanned.
    pub fn new<E>(cols: [C; CS], rows: [R; RS]) -> Result<Self, E>
    where
        C: OutputPin<Error = E>,
        R: InputPin<Error = E>,
    {
        let mut res = Self { cols, rows };
        res.clear()?;
        Ok(res)
    }
    fn clear<E>(&mut self) -> Result<(), E>
    where
        C: OutputPin<Error = E>,
        R: InputPin<Error = E>,
    {
        for r in self.cols.iter_mut() {
            r.set_low()?;
        }
        Ok(())
    }
    /// Scans the matrix and checks which keys are pressed.
    ///
    /// Every col pin in order is pulled high, and then each row
    /// pin is tested; if it's high, the key is marked as pressed.
    /// Scans the pins and checks which keys are pressed (state is "high").
    ///
    /// Delay function allows pause to let input pins settle
    pub fn get_with_delay<F: FnMut(), E>(&mut self, mut delay: F) -> Result<[[bool; CS]; RS], E>
    where
        C: OutputPin<Error = E>,
        R: InputPin<Error = E>,
    {
        let mut keys = [[false; CS]; RS];

        for (ci, col) in self.cols.iter_mut().enumerate() {
            col.set_high()?;
            delay();
            for (ri, row) in self.rows.iter_mut().enumerate() {
                if row.is_high()? {
                    keys[ri][ci] = true;
                }
            }
            col.set_low()?;
        }
        Ok(keys)
    }

    /// Scans the matrix and checks which keys are pressed.
    ///
    /// Every row pin in order is pulled low, and then each column
    /// pin is tested; if it's low, the key is marked as pressed.
    /// Scans the pins and checks which keys are pressed (state is "low").
    pub fn get<E>(&mut self) -> Result<[[bool; CS]; RS], E>
    where
        C: OutputPin<Error = E>,
        R: InputPin<Error = E>,
    {
        self.get_with_delay(|| ())
    }
}

/// Matrix-representation of switches directly attached to the pins ("diodeless").
///
/// Generic parameters are in order: The type of column pins,
/// the number of columns and rows.
pub struct DirectPinMatrix<P, const CS: usize, const RS: usize>
where
    P: InputPin,
{
    pins: [[Option<P>; RS]; CS],
}

impl<P, const CS: usize, const RS: usize> DirectPinMatrix<P, CS, RS>
where
    P: InputPin,
{
    /// Creates a new DirectPinMatrix.
    ///
    /// Assumes pins are pull-down inputs. Spots in the matrix that are
    /// not corresponding to any pins use ´None´.
    pub fn new<E>(pins: [[Option<P>; RS]; CS]) -> Result<Self, E>
    where
        P: InputPin<Error = E>,
    {
        let res = Self { pins };
        Ok(res)
    }

    /// Scans the pins and checks which keys are pressed (state is "low").
    pub fn get<E>(&mut self) -> Result<[[bool; CS]; RS], E>
    where
        P: InputPin<Error = E>,
    {
        let mut keys = [[false; CS]; RS];

        for (ci, col) in self.pins.iter_mut().enumerate() {
            for (ri, row_option) in col.iter_mut().enumerate() {
                if let Some(row) = row_option {
                    if row.is_high()? {
                        keys[ri][ci] = true;
                    }
                }
            }
        }
        Ok(keys)
    }
}
