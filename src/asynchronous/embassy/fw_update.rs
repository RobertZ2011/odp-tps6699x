use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_time::with_timeout;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use embedded_usb_pd::{Error, PdError};

use super::Tps6699x;
use crate::asynchronous::fw_update::UpdateTarget;
use crate::command::*;
use crate::{PORT0, error, info, warn};

impl<M: RawMutex, B: I2c> UpdateTarget for Tps6699x<'_, M, B> {
    /// Enter firmware update mode with the TFUs command
    async fn fw_update_mode_enter(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<Self::BusError>> {
        let result = {
            let mut inner = self.lock_inner().await;
            with_timeout(Command::Tfus.timeout(), inner.execute_tfus(delay)).await
        };

        if let Ok(result) = result {
            result
        } else {
            error!("Enter FW mode timeout");
            PdError::Timeout.into()
        }
    }

    /// Initialize the firmware update with the TFUi command
    async fn fw_update_init(
        &mut self,
        _delay: &mut impl DelayNs,
        args: &TfuiArgs,
    ) -> Result<ReturnValue, Error<Self::BusError>> {
        let args_buf: [u8; TFUI_ARGS_LEN] = bytemuck::must_cast(TfuiArgsRaw::from(*args));

        self.execute_command(PORT0, Command::Tfui, Some(&args_buf), None).await
    }

    /// Attempt to exit fw update mode
    async fn fw_update_mode_exit(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<Self::BusError>> {
        // Reset the controller if we failed to exit fw update mode
        if let Ok(ret) = self.execute_command(PORT0, Command::Tfue, None, None).await {
            if ret != ReturnValue::Success {
                warn!("FW update exit failed: {:?}", ret);
            }
        } else {
            warn!("FW update exit failed");
        }

        // Reset to return to normal operating mode
        info!("Resetting controller");
        self.reset(delay).await?;

        Ok(())
    }

    /// Validate the most recent block of data with a TFUq command
    async fn fw_update_validate_stream(
        &mut self,
        _delay: &mut impl DelayNs,
        block_index: usize,
    ) -> Result<TfuqBlockStatus, Error<Self::BusError>> {
        if block_index > TFUQ_RETURN_BLOCK_STATUS_LEN {
            return PdError::InvalidParams.into();
        }

        let args = TfuqArgs {
            command: TfuqCommandType::QueryTfuStatus,
            status_query: TfuqStatusQuery::StatusInProgress,
        };

        let arg_bytes: [u8; TFUQ_ARGS_LEN] = bytemuck::must_cast(TfuqArgsRaw::from(args));
        let mut return_bytes = [0u8; TFUQ_RETURN_LEN];

        let result = self
            .execute_command(PORT0, Command::Tfuq, Some(&arg_bytes), Some(&mut return_bytes))
            .await?;

        if result != ReturnValue::Success {
            error!("Validate stream failed {:?}", result);
            return PdError::Failed.into();
        }

        let raw: TfuqReturnValueRaw = bytemuck::try_pod_read_unaligned(
            return_bytes
                .get(..TFUQ_RETURN_VALUE_LEN)
                .ok_or(Error::Pd(PdError::Serialize))?,
        )
        .map_err(|_| Error::Pd(PdError::Serialize))?;
        let ret = TfuqReturnValue::try_from(raw).map_err(Error::Pd)?;

        ret.block_status
            .get(block_index)
            .cloned()
            .ok_or(Error::Pd(PdError::InvalidParams))
    }

    async fn fw_update_stream_data(
        &mut self,
        _delay: &mut impl DelayNs,
        args: &TfudArgs,
    ) -> Result<(), Error<Self::BusError>> {
        let arg_bytes: [u8; TFUD_ARGS_LEN] = bytemuck::must_cast(TfudArgsRaw::from(*args));
        let result = self
            .execute_command(PORT0, Command::Tfud, Some(&arg_bytes), None)
            .await?;

        if result != ReturnValue::Success {
            error!("Stream data failed, {:?}", result);
            return PdError::Failed.into();
        }

        Ok(())
    }

    async fn fw_update_complete(
        &mut self,
        delay: &mut impl embedded_hal_async::delay::DelayNs,
    ) -> Result<(), Error<Self::BusError>> {
        let result = {
            let mut inner = self.lock_inner().await;
            with_timeout(Command::Tfuc.timeout(), inner.execute_tfuc(delay)).await
        };

        if let Ok(result) = result {
            result
        } else {
            error!("Complete timeout");
            PdError::Timeout.into()
        }
    }

    async fn fw_update_burst_write(&mut self, address: u8, data: &[u8]) -> Result<(), Error<Self::BusError>> {
        let mut inner = self.controller.inner.lock().await;

        inner.bus.write(address, data).await.map_err(Error::Bus)?;
        Ok(())
    }
}
