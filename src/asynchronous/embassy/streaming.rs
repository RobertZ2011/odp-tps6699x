//! Streaming FW update implementation

use bincode::config;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use embedded_usb_pd::{Error, PdError};

use crate::{
    asynchronous::fw_update::{
        app_config_block_metadata_offset, data_block_index_to_block_index, data_block_metadata_offset, UpdateTarget,
    },
    command::{TfudArgs, TfuiArgs},
    fw_update::{
        APP_CONFIG_BLOCK_INDEX, APP_CONFIG_METADATA_LEN, APP_IMAGE_SIZE_LEN, APP_IMAGE_SIZE_OFFSET,
        DATA_BLOCK_METADATA_LEN, HEADER_BLOCK_INDEX, HEADER_BLOCK_LEN, HEADER_BLOCK_OFFSET, HEADER_METADATA_LEN,
        HEADER_METADATA_OFFSET,
    },
};

use super::Tps6699x;

const BUFFER_LENGTH: usize = 8;

/// Current update state
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// Receiving update args
    UpdateArgs,
    /// Receiving header block
    HeaderBlock,
    /// Receiving data block N header
    DataBlockHeader(usize),
    /// Receiving data block N
    DataBlock(usize),
    /// Receiving image size
    ImageSize,
    /// Receiving app config block header
    ConfigHeader,
    /// Receiving app config block
    ConfigBlock,
    /// Complete
    Complete,
}

/// Data stream state
enum DataState {
    /// Skip data until we reach the given byte
    Seek(usize),
    /// Consume the given number of bytes
    Read(usize),
}

/// Updater struct
pub struct Updater<'a, 'b, M: RawMutex, B: I2c> {
    /// Curent data index
    current_byte: usize,
    /// Current state of the updater
    state: State,
    /// Current data stream state
    data_state: DataState,
    /// Hardware driver
    driver: &'a mut Tps6699x<'b, M, B>,
    /// Buffer for various metadata
    buffer: [u8; BUFFER_LENGTH],
    /// Current image size
    image_size: [u8; APP_IMAGE_SIZE_LEN],
    /// Update args
    update_args: TfuiArgs,
}

impl<'a, 'b, M: RawMutex, B: I2c> Updater<'a, 'b, M, B> {
    pub fn new(driver: &'a mut Tps6699x<'b, M, B>) -> Self {
        Updater {
            current_byte: 0,
            data_state: DataState::Seek(HEADER_METADATA_OFFSET),
            state: State::UpdateArgs,
            driver,
            buffer: [0; BUFFER_LENGTH],
            image_size: [0; APP_IMAGE_SIZE_LEN],
            update_args: Default::default(),
        }
    }

    pub async fn start_fw_update(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<B::Error>> {
        self.driver.fw_update_mode_enter(delay).await
    }

    pub async fn write_bytes(&mut self, delay: &mut impl DelayNs, data: &[u8]) -> Result<bool, Error<B::Error>> {
        let mut data = data;

        while data.len() > 0 {
            self.current_byte += data.len();
            match self.data_state {
                DataState::Seek(target) => {
                    data = self.handle_seek(target, data).await?;
                }
                DataState::Read(required) => {
                    data = self.handle_read(delay, required, data).await?;
                }
            }
        }

        Ok(self.state == State::Complete)
    }

    async fn handle_seek<'c>(&mut self, target: usize, data: &'c [u8]) -> Result<&'c [u8], Error<B::Error>> {
        if self.current_byte <= target {
            // Still waiting for a particular byte
            return Ok(data);
        }

        // Drop any bytes before the target
        let available = target - self.current_byte;
        let data = &data[available..];

        // Switch to needs bytes state
        match self.state {
            State::UpdateArgs => {
                self.data_state = DataState::Read(HEADER_METADATA_LEN);
            }
            State::HeaderBlock => {
                self.data_state = DataState::Read(HEADER_BLOCK_LEN);
            }
            State::DataBlockHeader(_) => {
                self.data_state = DataState::Read(DATA_BLOCK_METADATA_LEN);
            }
            State::ImageSize => {
                self.data_state = DataState::Read(APP_IMAGE_SIZE_LEN);
            }
            State::ConfigHeader => {
                self.data_state = DataState::Read(APP_CONFIG_METADATA_LEN);
            }
            State::Complete | State::DataBlock(_) | State::ConfigBlock => {
                // No more data to process
            }
        }

        Ok(data)
    }

    async fn handle_read<'c>(
        &mut self,
        delay: &mut impl DelayNs,
        required: usize,
        data: &'c [u8],
    ) -> Result<&'c [u8], Error<B::Error>> {
        let bytes_to_take = required.min(data.len());
        let to_take = &data[..bytes_to_take];
        let remaining = required - bytes_to_take;

        match self.state {
            State::UpdateArgs => {
                self.read_update_args(delay, to_take, remaining).await?;
            }
            State::HeaderBlock => self.process_header_state(delay, to_take, remaining).await?,
            State::DataBlockHeader(block_index) => {
                self.read_data_block_header(delay, to_take, remaining, block_index)
                    .await?;
            }
            State::DataBlock(block_index) => {
                self.read_data_block(delay, to_take, remaining, block_index).await?;
            }
            State::ImageSize => {
                self.read_image_size(to_take, remaining).await?;
            }
            State::ConfigHeader => {
                self.read_config_block_header(delay, to_take, remaining).await?;
            }
            State::ConfigBlock => {
                self.read_config_block(delay, to_take, remaining).await?;
            }
            State::Complete => {
                // No more data to process
            }
        }

        Ok(&data[bytes_to_take..])
    }

    async fn read_update_args(
        &mut self,
        delay: &mut impl DelayNs,
        to_take: &[u8],
        remaining: usize,
    ) -> Result<(), Error<B::Error>> {
        let base = HEADER_METADATA_LEN - remaining;
        self.buffer[base..base + to_take.len()].copy_from_slice(to_take);

        if remaining == 0 {
            // We have the full header metadata
            let (args, _) = bincode::decode_from_slice(&self.buffer, config::standard().with_fixed_int_encoding())
                .map_err(|_| PdError::Serialize)?;
            self.driver.fw_update_init(delay, &args).await?;
            self.update_args = args;

            // Proceed to the next state
            self.state = State::HeaderBlock;
            self.data_state = DataState::Seek(HEADER_BLOCK_OFFSET);
        }

        Ok(())
    }

    async fn process_header_state(
        &mut self,
        delay: &mut impl DelayNs,
        to_take: &[u8],
        remaining: usize,
    ) -> Result<(), Error<B::Error>> {
        self.driver
            .fw_update_burst_write(self.update_args.broadcast_u16_address as u8, to_take)
            .await?;
        if remaining == 0 {
            // Full header has been written
            // Validate and proceed to the next state
            self.driver.fw_update_validate_stream(delay, HEADER_BLOCK_INDEX).await?;
            self.state = State::DataBlock(0);
            self.data_state = DataState::Seek(data_block_metadata_offset(0));
        }
        Ok(())
    }

    async fn read_block_header(
        &mut self,
        to_take: &[u8],
        remaining: usize,
    ) -> Result<Option<TfudArgs>, Error<B::Error>> {
        let base = DATA_BLOCK_METADATA_LEN - remaining;
        self.buffer[base..base + to_take.len()].copy_from_slice(to_take);

        if remaining == 0 {
            // We have the full header metadata
            let (args, _) = bincode::decode_from_slice(&self.buffer, config::standard().with_fixed_int_encoding())
                .map_err(|_| PdError::Serialize)?;
            Ok(Some(args))
        } else {
            // Still waiting for the rest of the header
            Ok(None)
        }
    }

    async fn read_data_block_header(
        &mut self,
        delay: &mut impl DelayNs,
        to_take: &[u8],
        remaining: usize,
        block_index: usize,
    ) -> Result<(), Error<B::Error>> {
        if let Some(args) = self.read_block_header(to_take, remaining).await? {
            // We have the full header metadata
            self.driver.fw_update_stream_data(delay, &args).await?;

            // Proceed to the next state
            self.state = State::DataBlock(block_index);
            self.data_state = DataState::Read(args.data_len.into());
        }

        Ok(())
    }

    async fn read_data_block(
        &mut self,
        delay: &mut impl DelayNs,
        to_take: &[u8],
        remaining: usize,
        block_index: usize,
    ) -> Result<(), Error<B::Error>> {
        self.driver
            .fw_update_burst_write(self.update_args.broadcast_u16_address as u8, to_take)
            .await?;

        if remaining == 0 {
            // Full block has been written
            // Validate and proceed to the next state
            self.driver
                .fw_update_validate_stream(delay, data_block_index_to_block_index(block_index))
                .await?;

            let next_block = block_index + 1;
            if next_block < self.update_args.num_data_blocks_tx.into() {
                // Proceed to the next block
                self.state = State::DataBlockHeader(next_block);
                self.data_state = DataState::Seek(data_block_metadata_offset(next_block));
            } else {
                // Read image size to find app config block
                self.state = State::ImageSize;
                self.data_state = DataState::Seek(APP_IMAGE_SIZE_OFFSET);
            }
        }

        Ok(())
    }

    async fn read_image_size(&mut self, to_take: &[u8], remaining: usize) -> Result<(), Error<B::Error>> {
        let base = APP_IMAGE_SIZE_LEN - remaining;
        self.image_size[base..base + to_take.len()].copy_from_slice(to_take);

        if remaining == 0 {
            // We have the full image size
            let (image_size, _): (u32, _) =
                bincode::decode_from_slice(&self.image_size, config::standard().with_fixed_int_encoding())
                    .map_err(|_| PdError::Serialize)?;
            self.state = State::ConfigHeader;
            self.data_state = DataState::Seek(app_config_block_metadata_offset(
                self.update_args.num_data_blocks_tx.into(),
                image_size as usize,
            ));
        }

        Ok(())
    }

    async fn read_config_block_header(
        &mut self,
        delay: &mut impl DelayNs,
        to_take: &[u8],
        remaining: usize,
    ) -> Result<(), Error<B::Error>> {
        if let Some(args) = self.read_block_header(to_take, remaining).await? {
            // We have the full header metadata
            self.driver.fw_update_stream_data(delay, &args).await?;

            // Proceed to the next state
            self.state = State::ConfigBlock;
            self.data_state = DataState::Read(args.data_len.into());
        }

        Ok(())
    }

    async fn read_config_block(
        &mut self,
        delay: &mut impl DelayNs,
        to_take: &[u8],
        remaining: usize,
    ) -> Result<(), Error<B::Error>> {
        self.driver
            .fw_update_burst_write(self.update_args.broadcast_u16_address as u8, to_take)
            .await?;

        if remaining == 0 {
            // Full block has been written
            // Validate and proceed to the next state
            self.driver
                .fw_update_validate_stream(delay, APP_CONFIG_BLOCK_INDEX)
                .await?;

            // Proceed to the next state
            self.state = State::Complete;
            self.data_state = DataState::Seek(0);
        }
        Ok(())
    }

    pub async fn exit_fw_update_mode(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<B::Error>> {
        self.driver.fw_update_mode_exit(delay).await
    }
}
