use super::*;
use core::convert::Infallible;
use embedded_hal::{
    digital::OutputPin,
    spi::{Operation, SpiDevice},
};
use embedded_storage::nor_flash::{MultiwriteNorFlash, NorFlash, ReadNorFlash};

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> ReadNorFlash for W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.read_blocking(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.capacity() as usize
    }
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> NorFlash for W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = Series::SECTOR_SIZE as usize;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.erase_range_blocking(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_blocking(offset, bytes)
    }
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> MultiwriteNorFlash for W25<Series, SPI, HOLD, WP> where
    SPI: SpiDevice<Error = S>
{
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP, P: Debug> W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
    HOLD: OutputPin<Error = P>,
    WP: OutputPin<Error = P>,
{
    /// Create a new *blocking* instance of the flash.
    ///
    /// The capacity must be the total chip capacity in bytes.
    /// Weird things can happen if you provide the wrong value.
    /// No checks are done, you're believed at your word.
    pub fn new_blocking(
        spi: SPI,
        hold: HOLD,
        wp: WP,
        capacity: u32,
    ) -> Result<Self, InitError<S, P>> {
        let mut flash = Self::_new(spi, hold, wp, capacity)?;

        while flash.busy_blocking()? {}

        Ok(flash)
    }

    /// Create a new *blocking* instance of the flash, autodetecting the chip
    /// variant and capacity.
    pub fn new_autodetect_blocking(spi: SPI, hold: HOLD, wp: WP) -> Result<Self, InitError<S, P>> {
        let mut flash = Self::new_blocking(spi, hold, wp, 0)?;

        let jedec = flash.jedec_id_blocking()?;
        if jedec.manufacturer() != JedecId::MANUFACTURER {
            return Err(InitError::ManufacturerNotRecognized(jedec.manufacturer()));
        }

        let major_id = jedec
            .major_device_id()
            .map_err(InitError::DeviceNotRecognized)?;
        flash.capacity = major_id.capacity();

        Ok(flash)
    }
}

impl<Series: NorSeries, SPI, S: Debug> W25<Series, SPI, NoPin, NoPin>
where
    SPI: SpiDevice<Error = S>,
{
    /// Create a new *blocking* instance of the flash, but without the nHold and
    /// nWP pins.
    ///
    /// The capacity must be the total chip capacity.
    /// Weird things can happen if you provide the wrong value.
    /// No checks are done, you're believed at your word.
    pub fn new_no_pins_blocking(spi: SPI, capacity: u32) -> Result<Self, InitError<S, Infallible>> {
        Self::new_blocking(spi, NoPin {}, NoPin {}, capacity)
    }

    /// Create a new *blocking* instance of the flash, autodetecting the chip
    /// variant and capacity, but without the nHold and nWP pins.
    pub fn new_autodetect_no_pins_blocking(spi: SPI) -> Result<Self, InitError<S, Infallible>> {
        Self::new_autodetect_blocking(spi, NoPin {}, NoPin {})
    }
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    fn read_status_register_blocking(&mut self) -> Result<u8, Error<S>> {
        let mut buf: [u8; 2] = [Command::ReadStatusRegister1 as u8, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .map_err(Error::SpiError)?;

        Ok(buf[1])
    }

    /// The flash chip is unable to perform new commands while it is still
    /// working on a previous one. Especially erases take a long time.
    /// This function returns true while the chip is unable to respond to
    /// commands (with the exception of the busy command).
    pub(crate) fn busy_blocking(&mut self) -> Result<bool, Error<S>> {
        Ok((self.read_status_register_blocking()? & 0x01) != 0)
    }

    fn write_enabled_blocking(&mut self) -> Result<bool, Error<S>> {
        Ok((self.read_status_register_blocking()? & 0x02) != 0)
    }

    /// Request the 64 bit id that is unique to this chip (blocking).
    pub fn device_id_blocking(&mut self) -> Result<[u8; 8], Error<S>> {
        let mut buf: [u8; 13] = [0; 13];
        buf[0] = Command::UniqueId as u8;

        self.spi
            .transfer_in_place(&mut buf)
            .map_err(Error::SpiError)?;

        Ok(TryFrom::try_from(&buf[5..]).unwrap())
    }

    /// Request the 24-bit JEDEC ID that is specific to this chip variant (blocking).
    pub fn jedec_id_blocking(&mut self) -> Result<JedecId, Error<S>> {
        let mut buf = [Command::JedecId as u8, 0, 0, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .map_err(Error::SpiError)?;

        Ok(JedecId(TryFrom::try_from(&buf[1..]).unwrap()))
    }

    /// Reset the chip (blocking).
    pub fn reset_blocking(&mut self) -> Result<(), Error<S>>
    where
        Series: Reset,
    {
        self.spi
            .write(&[Command::EnableReset as u8])
            .map_err(Error::SpiError)?;
        self.spi
            .write(&[Command::Reset as u8])
            .map_err(Error::SpiError)?;
        Ok(())
    }

    /// Reads a chunk of bytes from the flash chip (blocking).
    ///
    /// The number of bytes read is equal to the length of the buf slice.
    /// The first byte is read from the provided address. This address is then
    /// incremented for each following byte.
    ///
    /// # Arguments
    /// * `address` - Address where the first byte of the buf will be read.
    /// * `buf` - Slice that is going to be filled with the read bytes.
    pub fn read_blocking(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Error<S>> {
        self.check_read(address, buf.len())?;

        self.spi
            .transaction(&mut [
                Operation::Write(&command_and_address(Command::ReadData as u8, address)),
                Operation::Read(buf),
            ])
            .map_err(Error::SpiError)?;

        Ok(())
    }

    /// Sets the enable_write flag on the flash chip to true (blocking).
    ///
    /// Writes and erases to the chip only have effect when this flag is true.
    /// Each write and erase clears the flag, requiring it to be set to true
    /// again for the next command.
    fn enable_write_blocking(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::WriteEnable as u8])
            .map_err(Error::SpiError)?;

        if !self.write_enabled_blocking()? {
            return Err(Error::WriteEnableFail);
        }

        Ok(())
    }

    /// Writes a chunk of bytes to the flash chip (blocking).
    ///
    /// The first byte is written to the provided address. This address is then
    /// incremented for each following byte.
    ///
    /// # Arguments
    /// * `address` - Address where the first byte of the buf will be written.
    /// * `buf` - Slice of bytes that will be written.
    pub fn write_blocking(&mut self, mut address: u32, mut buf: &[u8]) -> Result<(), Error<S>> {
        self.check_write(address, buf.len())?;

        let chunk_len = (Series::PAGE_SIZE - (address & 0x000000FF)) as usize;
        let mut chunk_len = chunk_len.min(buf.len());
        self.write_page_blocking(address, &buf[..chunk_len])?;

        loop {
            buf = &buf[chunk_len..];
            address += chunk_len as u32;
            chunk_len = buf.len().min(Series::PAGE_SIZE as usize);
            if chunk_len == 0 {
                break;
            }
            self.write_page_blocking(address, &buf[..chunk_len])?;
        }

        Ok(())
    }

    /// Execute a write on a single page (blocking).
    fn write_page_blocking(&mut self, address: u32, buf: &[u8]) -> Result<(), Error<S>> {
        if (address & 0x000000FF) + buf.len() as u32 > Series::PAGE_SIZE {
            return Err(Error::OutOfBounds);
        }

        self.enable_write_blocking()?;

        self.spi
            .transaction(&mut [
                Operation::Write(&command_and_address(Command::PageProgram as u8, address)),
                Operation::Write(buf),
            ])
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Erases a range of sectors (blocking).
    ///
    /// The range is expressed in bytes. These bytes need to be a multiple of
    /// SECTOR_SIZE. If the range starts at SECTOR_SIZE * 3 then the erase
    /// starts at the fourth sector.All sectors are erased in the range
    /// [start_sector..end_sector].  The start address may not be a higher value
    /// than the end address.
    ///
    /// # Arguments
    /// * `start_address` - Address of the first byte of the start of the range
    ///   of sectors that need to be erased.
    /// * `end_address` - Address of the first byte of the end of the range of
    ///   sectors that need to be erased.
    pub fn erase_range_blocking(
        &mut self,
        start_address: u32,
        end_address: u32,
    ) -> Result<(), Error<S>> {
        self.check_erase(start_address, end_address)?;

        let start_sector = start_address / Series::SECTOR_SIZE;
        let end_sector = end_address / Series::SECTOR_SIZE;

        for sector in start_sector..end_sector {
            self.erase_sector_blocking(sector)?;
        }

        Ok(())
    }

    /// Erases a single sector of flash memory with the size of SECTOR_SIZE
    /// (blocking).
    ///
    /// # Arguments
    /// * `index` - the index of the sector that needs to be erased. The address
    ///   of the first byte of the sector is the provided index * SECTOR_SIZE.
    pub fn erase_sector_blocking(&mut self, index: u32) -> Result<(), Error<S>> {
        if index >= self.n_sectors() {
            return Err(Error::OutOfBounds);
        }

        self.enable_write_blocking()?;

        let address: u32 = index * Series::SECTOR_SIZE;

        self.spi
            .write(&command_and_address(Command::SectorErase as u8, address))
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Erases a single block of flash memory with the size of BLOCK_32K_SIZE
    /// (blocking).
    ///
    /// # Arguments
    /// * `index` - the index of the block that needs to be erased. The address
    ///   of the first byte of the block is the provided index * BLOCK_32K_SIZE.
    pub fn erase_block_32k_blocking(&mut self, index: u32) -> Result<(), Error<S>> {
        if index >= self.n_blocks_32k() {
            return Err(Error::OutOfBounds);
        }

        self.enable_write_blocking()?;

        let address: u32 = index * 32768;

        self.spi
            .write(&command_and_address(Command::Block32Erase as u8, address))
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Erases a single block of flash memory with the size of BLOCK_64K_SIZE
    /// (blocking).
    ///
    /// # Arguments
    /// * `index` - the index of the block that needs to be erased. The address
    ///   of the first byte of the block is the provided index * BLOCK_64K_SIZE.
    pub fn erase_block_64k_blocking(&mut self, index: u32) -> Result<(), Error<S>> {
        if index >= self.n_blocks_64k() {
            return Err(Error::OutOfBounds);
        }

        self.enable_write_blocking()?;

        let address: u32 = index * 65536;

        self.spi
            .write(&command_and_address(Command::Block64Erase as u8, address))
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Erases all sectors on the flash chip (blocking).
    ///
    /// This is a very expensive operation.
    pub fn erase_chip_blocking(&mut self) -> Result<(), Error<S>> {
        self.enable_write_blocking()?;

        self.spi
            .write(&[Command::ChipErase as u8])
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Puts the chip into power down mode (blocking).
    ///
    /// While in the power-down state, only the Release Power-down/Device ID
    /// (0xAB) instruction will be recognized. This instruction restores the
    /// device to normal operation. All other instructions are ignored.
    pub fn enable_power_down_mode_blocking(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::PowerDown as u8])
            .map_err(Error::SpiError)?;

        Ok(())
    }

    /// Releases the chip from power down mode (blocking).
    ///
    /// Restores operation from power down mode by reading the deviceID from the
    /// device.
    pub fn disable_power_down_mode_blocking(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::ReleasePowerDown as u8])
            .map_err(Error::SpiError)?;

        Ok(())
    }
}
