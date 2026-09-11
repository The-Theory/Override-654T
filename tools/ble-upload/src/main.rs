//! Uploads the PROS hot/cold package to a V5 Brain over Bluetooth LE.
//!
//! Brain must have radio type set to Bluetooth and "Enable Data" turned on.

use std::{env, fs, time::Duration};

use vex_v5_serial::{
    Connection,
    bluetooth::{self, BluetoothError},
    commands::file::{ProgramData, UploadProgram},
    protocol::{
        FixedString,
        cdc2::file::{
            FileErasePacket, FileErasePayload, FileEraseReplyPacket, FileExitAction, FileLoadAction,
            FileLoadActionPacket, FileLoadActionPayload, FileVendor,
        },
    },
};

const slot: u8 = 1;

#[tokio::main]
async fn main() -> Result<(), BluetoothError> {
    let level = match std::env::var("V5_LOG").as_deref() {
        Ok("trace") => log::LevelFilter::Trace,
        Ok("debug") => log::LevelFilter::Debug,
        _ => log::LevelFilter::Info,
    };
    simplelog::TermLogger::init(
        level,
        simplelog::Config::default(),
        simplelog::TerminalMode::Mixed,
        simplelog::ColorChoice::Always,
    )
    .unwrap();

    let root = env::current_dir().unwrap();
    let bin = |name: &str| fs::read(root.join("bin").join(name)).ok();

    let force_cold = env::args().any(|a| a == "--cold");
    let cold_data = bin("cold.package.bin");

    // The brain rejects a hot binary linked against a cold library it no longer has,
    // so resend cold whenever the local one changed since the last successful upload.
    let stamp = root.join("bin/.ble-cold-stamp");
    let cold_hash = cold_data.as_ref().map(|d| hash(d).to_string());
    let cold_stale = cold_hash != fs::read_to_string(&stamp).ok();
    let send_cold = force_cold || cold_stale;

    log::info!("scanning for Brain...");
    let devices = bluetooth::find_devices(Duration::from_secs(15), Some(1)).await?;
    let mut connection = devices[0].connect().await?;

    if !connection.is_paired().await? {
        connection.request_pairing().await?;
        let pin = prompt_pin();
        connection.authenticate_pairing(pin).await?;
    }

    // A running program competes for the CPU that has to service BLE and commit
    // flash, which is enough to stall a cold upload. No-op if nothing is running.
    connection
        .send(FileLoadActionPacket::new(FileLoadActionPayload {
            vendor: FileVendor::User,
            action: FileLoadAction::Stop,
            file_name: FixedString::default(),
        }))
        .await?;
    tokio::time::sleep(Duration::from_millis(500)).await;

    // Erasing 2MB of flash blocks the Brain for seconds. Left implicit inside a
    // transfer it stalls the next handshake and the BLE link drops, so do it up
    // front as its own operation with nothing in flight to time out.
    if send_cold {
        for file in ["slot_1.bin", "slot_1_lib.bin"] {
            log::info!("erasing {file}");
            connection
                .handshake::<FileEraseReplyPacket>(
                    Duration::from_secs(30),
                    3,
                    FileErasePacket::new(FileErasePayload {
                        vendor: FileVendor::User,
                        reserved: 0,
                        file_name: FixedString::new(file.to_string()).unwrap(),
                    }),
                )
                .await?
                .payload
                .ok();
        }
    }

    connection
        .execute_command(UploadProgram {
            name: "Override".to_string(),
            description: "PROS over BLE".to_string(),
            icon: "USER902x.bmp".to_string(),
            program_type: "PROS".to_string(),
            slot,
            compress: true,
            data: ProgramData::HotCold {
                hot: bin("hot.package.bin"),
                cold: send_cold.then_some(cold_data).flatten(),
            },
            after_upload: FileExitAction::ShowRunScreen,
            ini_callback: progress("INI"),
            bin_callback: progress("BIN"),
            lib_callback: progress("LIB"),
        })
        .await?;

    if let Some(hash) = cold_hash {
        fs::write(&stamp, hash).ok();
    }

    Ok(())
}

/// ponytail: DefaultHasher, not a cryptographic digest. Only detects local rebuilds,
/// not a brain flashed by someone else. Swap for sha2 if that ever matters.
fn hash(data: &[u8]) -> u64 {
    use std::hash::{DefaultHasher, Hash, Hasher};
    let mut h = DefaultHasher::new();
    data.hash(&mut h);
    h.finish()
}

fn progress(step: &'static str) -> Option<Box<dyn FnMut(f32) + Send>> {
    Some(Box::new(move |pct| log::info!("{step}: {pct:.1}%")))
}

fn prompt_pin() -> [u8; 4] {
    use std::io::Write;
    print!("PIN shown on Brain screen: ");
    std::io::stdout().flush().unwrap();
    let mut line = String::new();
    std::io::stdin().read_line(&mut line).unwrap();
    let mut digits = line.trim().chars().filter_map(|c| c.to_digit(10));
    std::array::from_fn(|_| digits.next().expect("need 4 digits") as u8)
}
