use std::io;
use std::os::unix::thread::JoinHandleExt;
use std::thread::JoinHandle;

use libc::{SCHED_FIFO, pthread_setschedparam, sched_get_priority_max, sched_get_priority_min};

use crate::core::{ActuatorError, Result};

pub fn validate_thread_priority(priority: i32) -> Result<()> {
    let (min, max) = thread_priority_bounds()?;
    if !(min..=max).contains(&priority) {
        return Err(ActuatorError::Protocol(format!(
            "rx thread priority must be between {min} and {max}, got {priority}"
        )));
    }
    Ok(())
}

pub fn apply_thread_priority<T>(handle: &JoinHandle<T>, priority: i32) -> Result<()> {
    validate_thread_priority(priority)?;

    let params = libc::sched_param {
        sched_priority: priority,
    };
    let result = unsafe { pthread_setschedparam(handle.as_pthread_t(), SCHED_FIFO, &params) };
    if result != 0 {
        let error = io::Error::from_raw_os_error(result);
        return Err(ActuatorError::Io(io::Error::new(
            error.kind(),
            format!("failed to set RX thread priority to {priority}: {error}"),
        )));
    }

    Ok(())
}

fn thread_priority_bounds() -> Result<(i32, i32)> {
    let min = unsafe { sched_get_priority_min(SCHED_FIFO) };
    if min == -1 {
        return Err(io::Error::last_os_error().into());
    }

    let max = unsafe { sched_get_priority_max(SCHED_FIFO) };
    if max == -1 {
        return Err(io::Error::last_os_error().into());
    }

    Ok((min, max))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thread_priority_rejects_zero() {
        let error = validate_thread_priority(0).unwrap_err();
        assert!(
            error
                .to_string()
                .contains("rx thread priority must be between")
        );
    }
}
