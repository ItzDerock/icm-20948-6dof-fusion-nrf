use embassy_nrf::uarte::Uarte;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Receiver;

use crate::icm::ImuData;

// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no input/output reflection
fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// COBS encode `input` into `out`. Returns slice length (including trailing 0x00).
fn cobs_encode<'a>(input: &[u8], out: &'a mut [u8]) -> &'a [u8] {
    let mut write_pos = 0usize;
    let mut code_pos = 0usize;
    let mut code: u8 = 1;

    // Reserve space for first overhead byte
    write_pos += 1;

    for &byte in input {
        if byte == 0 {
            out[code_pos] = code;
            code_pos = write_pos;
            write_pos += 1;
            code = 1;
        } else {
            out[write_pos] = byte;
            write_pos += 1;
            code += 1;
            if code == 0xFF {
                out[code_pos] = code;
                code_pos = write_pos;
                write_pos += 1;
                code = 1;
            }
        }
    }
    out[code_pos] = code;
    out[write_pos] = 0x00;
    write_pos += 1;
    &out[..write_pos]
}

pub async fn comms_run<'d>(
    receiver: Receiver<'d, ThreadModeRawMutex, ImuData, 16>,
    mut uart: Uarte<'d>,
) -> ! {
    // 7 x f32 = 28 bytes payload + 2 bytes CRC = 30 bytes
    // COBS worst case: 30 + ceil(30/254) + 1 = 32 bytes
    let mut tx_buffer = [0u8; 64];

    loop {
        let imu_data = receiver.receive().await;

        // Pack 7 f32s as little-endian bytes
        let mut payload = [0u8; 30];
        let floats = [
            imu_data.accel.x,
            imu_data.accel.y,
            imu_data.accel.z,
            imu_data.quat.w,
            imu_data.quat.i,
            imu_data.quat.j,
            imu_data.quat.k,
        ];
        for (i, &f) in floats.iter().enumerate() {
            payload[i * 4..i * 4 + 4].copy_from_slice(&f.to_le_bytes());
        }

        // Append CRC-16 LE over the 28-byte data portion
        let crc = crc16(&payload[..28]);
        payload[28] = (crc & 0xFF) as u8;
        payload[29] = (crc >> 8) as u8;

        let encoded = cobs_encode(&payload, &mut tx_buffer);
        let _ = uart.write(encoded).await;
    }
}
