use bytes::Bytes;
use st3215::protocol::{EepromRegister, RamRegister, ST3215Request, ST3215Response};
use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Duration;
use tokio_serial::SerialPortBuilderExt;

const BUS_BAUD: u32 = 1_000_000;
const IO_TIMEOUT_MS: u64 = 30;
const HOME_TICK: u16 = 2048;
const HOME_TOLERANCE_TICKS: u16 = 10;
const COMMIT_TOKEN: &str = "LF_FREEZE_COMMIT";
const LF_MOTORS: [u8; 3] = [11, 12, 13];

#[derive(Debug, Clone)]
struct StageMotor {
    motor_id: u8,
    joint_name: String,
    estimated_q0_tick: u16,
}

#[derive(Debug)]
struct StageProfile {
    bus_serial: String,
    urdf_sha256: String,
    motors: BTreeMap<u8, StageMotor>,
}

#[derive(Debug, Clone)]
struct MotorBackup {
    motor_id: u8,
    offset: i16,
    mode: u8,
    p: u8,
    i: u8,
    d: u8,
    return_delay: u8,
    max_temperature: u8,
    max_torque: u16,
    protection_current: u16,
    overload_torque: u8,
    lock: u8,
    present_position: u16,
    torque_enabled: u8,
}

#[derive(Debug, Clone)]
struct FreezePlan {
    stage: StageMotor,
    backup: MotorBackup,
    offset_delta: i16,
    new_offset: i16,
}

fn usage() -> ! {
    eprintln!(
        "Usage: matdog_lf_freeze --serial PATH --stage FILE --backup FILE --result FILE --commit LF_FREEZE_COMMIT"
    );
    std::process::exit(2);
}

fn parse_args() -> (PathBuf, PathBuf, PathBuf, PathBuf) {
    let mut serial = None;
    let mut stage = None;
    let mut backup = None;
    let mut result = None;
    let mut commit = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        let value = args.next().unwrap_or_else(|| usage());
        match arg.as_str() {
            "--serial" => serial = Some(PathBuf::from(value)),
            "--stage" => stage = Some(PathBuf::from(value)),
            "--backup" => backup = Some(PathBuf::from(value)),
            "--result" => result = Some(PathBuf::from(value)),
            "--commit" => commit = Some(value),
            _ => usage(),
        }
    }
    if commit.as_deref() != Some(COMMIT_TOKEN) {
        eprintln!("Refusing EEPROM writes without the exact commit token");
        std::process::exit(2);
    }
    (
        serial.unwrap_or_else(|| usage()),
        stage.unwrap_or_else(|| usage()),
        backup.unwrap_or_else(|| usage()),
        result.unwrap_or_else(|| usage()),
    )
}

fn parse_stage(path: &Path) -> Result<StageProfile, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("read stage: {e}"))?;
    let mut values = BTreeMap::<String, String>::new();
    for (index, raw) in text.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let (key, value) = line
            .split_once('=')
            .ok_or_else(|| format!("invalid stage line {}: {line}", index + 1))?;
        if values
            .insert(key.trim().to_owned(), value.trim().to_owned())
            .is_some()
        {
            return Err(format!("duplicate stage key: {}", key.trim()));
        }
    }
    if values.get("format").map(String::as_str) != Some("MATDOG_LF_STAGE_V1") {
        return Err("invalid stage format".into());
    }
    if values.get("status").map(String::as_str) != Some("LF_STAGED") {
        return Err("stage is not LF_STAGED".into());
    }
    let bus_serial = values
        .get("bus_serial")
        .cloned()
        .ok_or("missing bus_serial")?;
    let urdf_sha256 = values
        .get("urdf_sha256")
        .cloned()
        .ok_or("missing urdf_sha256")?;
    if urdf_sha256.len() != 64 || !urdf_sha256.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("invalid urdf_sha256".into());
    }
    let mut motors = BTreeMap::new();
    for motor_id in LF_MOTORS {
        let prefix = format!("motor.{motor_id}.");
        let joint_name = values
            .get(&(prefix.clone() + "joint_name"))
            .cloned()
            .ok_or_else(|| format!("missing {prefix}joint_name"))?;
        let estimated_q0_tick = values
            .get(&(prefix.clone() + "estimated_q0_tick"))
            .ok_or_else(|| format!("missing {prefix}estimated_q0_tick"))?
            .parse::<u16>()
            .map_err(|_| format!("invalid {prefix}estimated_q0_tick"))?;
        if estimated_q0_tick > 4095 {
            return Err(format!("M{motor_id} estimated q0 outside unsigned range"));
        }
        if values
            .get(&(prefix.clone() + "accepted"))
            .map(String::as_str)
            != Some("true")
        {
            return Err(format!("M{motor_id} stage is not accepted"));
        }
        motors.insert(
            motor_id,
            StageMotor {
                motor_id,
                joint_name,
                estimated_q0_tick,
            },
        );
    }
    Ok(StageProfile {
        bus_serial,
        urdf_sha256,
        motors,
    })
}

fn circular_distance(first: u16, second: u16) -> u16 {
    let direct = first.abs_diff(second);
    direct.min(4096 - direct)
}

fn signed_tick_delta(value: u16, reference: u16) -> i16 {
    ((i32::from(value) - i32::from(reference) + 2048).rem_euclid(4096) - 2048) as i16
}

fn normalize_offset(value: i32) -> Result<i16, String> {
    let wrapped = (value + 2048).rem_euclid(4096) - 2048;
    if wrapped == -2048 {
        return Err("ST3215 offset -2048 is outside documented range".into());
    }
    i16::try_from(wrapped).map_err(|_| "offset conversion failed".into())
}

async fn request(
    port: &mut tokio_serial::SerialStream,
    request: ST3215Request,
) -> Result<ST3215Response, String> {
    request
        .async_readwrite(port, IO_TIMEOUT_MS)
        .await
        .map_err(|e| e.to_string())
}

async fn ping(port: &mut tokio_serial::SerialStream, motor_id: u8) -> Result<(), String> {
    request(port, ST3215Request::Ping { motor: motor_id })
        .await
        .map(|_| ())
}

async fn read_bytes(
    port: &mut tokio_serial::SerialStream,
    motor_id: u8,
    address: u8,
    length: u8,
) -> Result<Bytes, String> {
    match request(
        port,
        ST3215Request::Read {
            motor: motor_id,
            address,
            length,
        },
    )
    .await?
    {
        ST3215Response::Read { data, .. } => Ok(data),
        _ => Err("unexpected ST3215 read response".into()),
    }
}

async fn read_u8(
    port: &mut tokio_serial::SerialStream,
    motor_id: u8,
    address: u8,
) -> Result<u8, String> {
    let data = read_bytes(port, motor_id, address, 1).await?;
    Ok(data[0])
}

async fn read_u16(
    port: &mut tokio_serial::SerialStream,
    motor_id: u8,
    address: u8,
) -> Result<u16, String> {
    let data = read_bytes(port, motor_id, address, 2).await?;
    Ok(u16::from_le_bytes([data[0], data[1]]))
}

async fn read_i16(
    port: &mut tokio_serial::SerialStream,
    motor_id: u8,
    address: u8,
) -> Result<i16, String> {
    let data = read_bytes(port, motor_id, address, 2).await?;
    Ok(i16::from_le_bytes([data[0], data[1]]))
}

async fn write_ram(
    port: &mut tokio_serial::SerialStream,
    motor_id: u8,
    register: RamRegister,
    data: Bytes,
) -> Result<(), String> {
    request(
        port,
        ST3215Request::Write {
            motor: motor_id,
            address: register.address(),
            data,
        },
    )
    .await
    .map(|_| ())
}

async fn action(port: &mut tokio_serial::SerialStream, motor_id: u8) -> Result<(), String> {
    request(port, ST3215Request::Action { motor: motor_id })
        .await
        .map(|_| ())
}

async fn set_lock(
    port: &mut tokio_serial::SerialStream,
    motor_id: u8,
    value: u8,
) -> Result<(), String> {
    write_ram(
        port,
        motor_id,
        RamRegister::Lock,
        Bytes::copy_from_slice(&[value]),
    )
    .await?;
    action(port, motor_id).await?;
    let readback = read_u8(port, motor_id, RamRegister::Lock.address()).await?;
    if readback != value {
        return Err(format!(
            "M{motor_id} Lock readback mismatch: expected={value}, got={readback}"
        ));
    }
    Ok(())
}

async fn write_eeprom_verified(
    port: &mut tokio_serial::SerialStream,
    motor_id: u8,
    register: EepromRegister,
    data: Bytes,
) -> Result<(), String> {
    if data.len() != usize::from(register.size()) {
        return Err(format!("{} write size mismatch", register.name()));
    }
    for attempt in 1..=5 {
        let reg_result = request(
            port,
            ST3215Request::RegWrite {
                motor: motor_id,
                address: register.address(),
                data: data.clone(),
            },
        )
        .await;
        if reg_result.is_err() || action(port, motor_id).await.is_err() {
            tokio::time::sleep(Duration::from_millis(20)).await;
            continue;
        }
        let readback = read_bytes(port, motor_id, register.address(), register.size()).await?;
        if readback == data {
            return Ok(());
        }
        eprintln!(
            "M{motor_id} {} verify mismatch on attempt {attempt}: expected={:02x?} got={:02x?}",
            register.name(),
            data,
            readback
        );
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    Err(format!(
        "M{motor_id} {} write failed after retries",
        register.name()
    ))
}

async fn read_backup(
    port: &mut tokio_serial::SerialStream,
    motor_id: u8,
) -> Result<MotorBackup, String> {
    Ok(MotorBackup {
        motor_id,
        offset: read_i16(port, motor_id, EepromRegister::Offset.address()).await?,
        mode: read_u8(port, motor_id, EepromRegister::Mode.address()).await?,
        p: read_u8(port, motor_id, EepromRegister::PCoef.address()).await?,
        i: read_u8(port, motor_id, EepromRegister::ICoef.address()).await?,
        d: read_u8(port, motor_id, EepromRegister::DCoef.address()).await?,
        return_delay: read_u8(port, motor_id, EepromRegister::ReturnDelay.address()).await?,
        max_temperature: read_u8(port, motor_id, EepromRegister::MaxTemperature.address()).await?,
        max_torque: read_u16(port, motor_id, EepromRegister::MaxTorque.address()).await?,
        protection_current: read_u16(port, motor_id, EepromRegister::ProtectionCurrent.address())
            .await?,
        overload_torque: read_u8(port, motor_id, EepromRegister::OverloadTorque.address()).await?,
        lock: read_u8(port, motor_id, RamRegister::Lock.address()).await?,
        present_position: read_u16(port, motor_id, RamRegister::PresentPosition.address()).await?,
        torque_enabled: read_u8(port, motor_id, RamRegister::TorqueEnable.address()).await?,
    })
}

fn backup_json(stage: &StageProfile, plans: &[FreezePlan]) -> String {
    let mut out = String::new();
    out.push_str("{\n  \"format\": \"MATDOG_LF_EEPROM_BACKUP_V1\",\n");
    out.push_str(&format!("  \"bus_serial\": \"{}\",\n", stage.bus_serial));
    out.push_str(&format!("  \"urdf_sha256\": \"{}\",\n", stage.urdf_sha256));
    out.push_str("  \"motors\": [\n");
    for (index, plan) in plans.iter().enumerate() {
        let b = &plan.backup;
        out.push_str(&format!(
            "    {{\"motor_id\":{},\"joint_name\":\"{}\",\"offset\":{},\"planned_offset\":{},\"estimated_q0_tick\":{},\"mode\":{},\"p\":{},\"i\":{},\"d\":{},\"return_delay\":{},\"max_temperature\":{},\"max_torque\":{},\"protection_current\":{},\"overload_torque\":{},\"lock\":{},\"present_position\":{},\"torque_enabled\":{}}}{}\n",
            b.motor_id,
            plan.stage.joint_name,
            b.offset,
            plan.new_offset,
            plan.stage.estimated_q0_tick,
            b.mode,
            b.p,
            b.i,
            b.d,
            b.return_delay,
            b.max_temperature,
            b.max_torque,
            b.protection_current,
            b.overload_torque,
            b.lock,
            b.present_position,
            b.torque_enabled,
            if index + 1 == plans.len() { "" } else { "," }
        ));
    }
    out.push_str("  ]\n}\n");
    out
}

fn atomic_write(path: &Path, content: &str) -> Result<(), String> {
    let parent = path.parent().ok_or("output path has no parent")?;
    fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    let temporary = path.with_extension("tmp");
    let _ = fs::remove_file(&temporary);
    let mut file = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temporary)
        .map_err(|e| format!("create {}: {e}", temporary.display()))?;
    file.write_all(content.as_bytes())
        .map_err(|e| e.to_string())?;
    file.sync_all().map_err(|e| e.to_string())?;
    drop(file);
    fs::rename(&temporary, path).map_err(|e| e.to_string())
}

async fn rollback(
    port: &mut tokio_serial::SerialStream,
    changed: &[FreezePlan],
) -> Result<(), String> {
    for plan in changed.iter().rev() {
        let motor_id = plan.stage.motor_id;
        set_lock(port, motor_id, 0).await?;
        write_eeprom_verified(
            port,
            motor_id,
            EepromRegister::Offset,
            Bytes::copy_from_slice(&plan.backup.offset.to_le_bytes()),
        )
        .await?;
        set_lock(port, motor_id, 1).await?;
        let restored = read_i16(port, motor_id, EepromRegister::Offset.address()).await?;
        if restored != plan.backup.offset {
            return Err(format!("M{motor_id} rollback offset mismatch"));
        }
    }
    Ok(())
}

async fn run() -> Result<(), String> {
    let (serial_path, stage_path, backup_path, result_path) = parse_args();
    let stage = parse_stage(&stage_path)?;
    let mut port = tokio_serial::new(serial_path.to_string_lossy(), BUS_BAUD)
        .timeout(Duration::from_millis(IO_TIMEOUT_MS))
        .open_native_async()
        .map_err(|e| format!("open serial: {e}"))?;

    let mut plans = Vec::new();
    for motor_id in LF_MOTORS {
        ping(&mut port, motor_id).await?;
        let backup = read_backup(&mut port, motor_id).await?;
        if backup.torque_enabled != 0 {
            return Err(format!(
                "M{motor_id} torque must be OFF before EEPROM freeze"
            ));
        }
        let stage_motor = stage.motors.get(&motor_id).unwrap().clone();
        let position_error =
            circular_distance(backup.present_position, stage_motor.estimated_q0_tick);
        if position_error > HOME_TOLERANCE_TICKS {
            return Err(format!(
                "M{motor_id} is not at staged q0: present={}, expected={}, error={} > {}",
                backup.present_position,
                stage_motor.estimated_q0_tick,
                position_error,
                HOME_TOLERANCE_TICKS
            ));
        }
        if backup.mode != 0 || backup.max_temperature != 70 || backup.lock != 1 {
            return Err(format!(
                "M{motor_id} provisioning prerequisite mismatch: mode={}, max_temperature={}, lock={}; rerun digital-home provisioning",
                backup.mode, backup.max_temperature, backup.lock
            ));
        }
        let offset_delta = signed_tick_delta(stage_motor.estimated_q0_tick, HOME_TICK);
        let new_offset = normalize_offset(i32::from(backup.offset) + i32::from(offset_delta))?;
        plans.push(FreezePlan {
            stage: stage_motor,
            backup,
            offset_delta,
            new_offset,
        });
    }

    atomic_write(&backup_path, &backup_json(&stage, &plans))?;

    let mut changed: Vec<FreezePlan> = Vec::new();
    for plan in &plans {
        let motor_id = plan.stage.motor_id;
        println!(
            "EEPROM M{motor_id}: old_offset={} delta={} new_offset={}",
            plan.backup.offset, plan.offset_delta, plan.new_offset
        );
        if plan.backup.offset == plan.new_offset {
            continue;
        }
        let operation = async {
            set_lock(&mut port, motor_id, 0).await?;
            write_eeprom_verified(
                &mut port,
                motor_id,
                EepromRegister::Offset,
                Bytes::copy_from_slice(&plan.new_offset.to_le_bytes()),
            )
            .await?;
            set_lock(&mut port, motor_id, 1).await?;
            let readback = read_i16(&mut port, motor_id, EepromRegister::Offset.address()).await?;
            if readback != plan.new_offset {
                return Err(format!(
                    "M{motor_id} persistent offset mismatch: expected={}, got={readback}",
                    plan.new_offset
                ));
            }
            Ok::<(), String>(())
        }
        .await;
        match operation {
            Ok(()) => changed.push(plan.clone()),
            Err(error) => {
                let rollback_error = rollback(&mut port, &changed).await.err();
                return Err(match rollback_error {
                    Some(rollback) => format!("{error}; rollback also failed: {rollback}"),
                    None => format!("{error}; previous EEPROM writes rolled back"),
                });
            }
        }
    }

    let mut result = String::new();
    result.push_str("format=MATDOG_LF_FREEZE_RESULT_V1\nstatus=LF_FROZEN\n");
    result.push_str(&format!("bus_serial={}\n", stage.bus_serial));
    result.push_str(&format!("urdf_sha256={}\n", stage.urdf_sha256));
    for plan in &plans {
        let motor_id = plan.stage.motor_id;
        let offset = read_i16(&mut port, motor_id, EepromRegister::Offset.address()).await?;
        let position =
            read_u16(&mut port, motor_id, RamRegister::PresentPosition.address()).await?;
        let lock = read_u8(&mut port, motor_id, RamRegister::Lock.address()).await?;
        let torque = read_u8(&mut port, motor_id, RamRegister::TorqueEnable.address()).await?;
        if offset != plan.new_offset
            || circular_distance(position, HOME_TICK) > HOME_TOLERANCE_TICKS
        {
            let rollback_error = rollback(&mut port, &changed).await.err();
            return Err(match rollback_error {
                Some(rollback) => format!(
                    "M{motor_id} final q0 verification failed; rollback also failed: {rollback}"
                ),
                None => format!("M{motor_id} final q0 verification failed; EEPROM rolled back"),
            });
        }
        if lock != 1 || torque != 0 {
            return Err(format!(
                "M{motor_id} final state invalid: lock={lock}, torque={torque}"
            ));
        }
        result.push_str(&format!(
            "motor.{motor_id}.joint_name={}\n",
            plan.stage.joint_name
        ));
        result.push_str(&format!(
            "motor.{motor_id}.old_offset={}\n",
            plan.backup.offset
        ));
        result.push_str(&format!(
            "motor.{motor_id}.new_offset={}\n",
            plan.new_offset
        ));
        result.push_str(&format!("motor.{motor_id}.position={}\n", position));
        result.push_str(&format!("motor.{motor_id}.lock={}\n", lock));
        result.push_str(&format!("motor.{motor_id}.torque_enabled={}\n", torque));
    }
    atomic_write(&result_path, &result)?;
    println!("MATDOG_LF_EEPROM_FREEZE=PASS");
    Ok(())
}

#[tokio::main]
async fn main() {
    if let Err(error) = run().await {
        eprintln!("MATDOG_LF_EEPROM_FREEZE=FAIL: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offset_delta_matches_displayed_position_convention() {
        assert_eq!(signed_tick_delta(2048, 2048), 0);
        assert_eq!(signed_tick_delta(2044, 2048), -4);
        assert_eq!(signed_tick_delta(2079, 2048), 31);
        assert_eq!(normalize_offset(100 + 31).unwrap(), 131);
        assert_eq!(normalize_offset(-100 - 4).unwrap(), -104);
    }

    #[test]
    fn circular_distance_handles_wrap() {
        assert_eq!(circular_distance(4095, 0), 1);
        assert_eq!(circular_distance(2048, 2048), 0);
    }
}
