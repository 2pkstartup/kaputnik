//! ==========================================================================
//!  kaputnik-downloader – CLI nástroj pro Kaputnik flight logger
//! ==========================================================================
//!
//!  Komunikuje s RP2040 firmware přes USB CDC sériovou linku.
//!
//!  Podporované příkazy:
//!    dump   – stažení letových dat jako CSV (s progress barem)
//!    save   – stažení dat a uložení do CSV pojmenovaného podle času záznamu
//!    status – zobrazení stavu zařízení (MPU, flash, čas, záznam)
//!    erase  – smazání celé flash paměti
//!    start  – vymazání flash + synchronizace času + spuštění záznamu vzdáleně
//!    stop   – zastavení záznamu vzdáleně
//!    list   – výpis dostupných sériových portů
//!
//!  Výchozí port: auto-detekce USB/serial portu, 115200 baud
//!  Ruční volba: kaputnik-downloader -p COM5 status

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};
use serialport::UsbPortInfo;
use std::fs::File;
use std::io::{Read, Write};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

// =========================================================================
//  CLI definice (clap derive)
// =========================================================================

#[derive(Parser)]
#[command(name = "kaputnik-downloader")]
#[command(about = "Kaputnik flight data downloader")]
struct Cli {
    /// Sériový port (např. COM5, /dev/ttyACM0). Když není uveden, zkusí se auto-detekce USB zařízení.
    #[arg(short, long)]
    port: Option<String>,

    /// Přenosová rychlost [baud]
    #[arg(short, long, default_value_t = 115200)]
    baud: u32,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Stáhnout letová data jako CSV
    Dump {
        /// Výstupní soubor (výchozí: stdout)
        #[arg(short, long)]
        output: Option<String>,
    },
    /// Stáhnout data a uložit do CSV s názvem podle času záznamu
    Save,
    /// Zobrazit stav zařízení
    Status,
    /// Smazat flash paměť
    Erase,
    /// Vymazat flash, synchronizovat čas a spustit záznam
    Start,
    /// Zastavit záznam
    Stop,
    /// Vypsat dostupné sériové porty
    List,
}

// =========================================================================
//  Pomocné funkce pro sériovou komunikaci
// =========================================================================

/// Otevře sériový port s daným názvem a rychlostí.
/// Timeout 5 s – pokud zařízení neodpoví, operace selhá.
fn open_port(port: &str, baud: u32) -> Result<Box<dyn serialport::SerialPort>> {
    let mut serial = serialport::new(port, baud)
        .timeout(Duration::from_secs(5))
        .open()
        .with_context(|| format!("Cannot open serial port {port}"))?;

    serial
        .write_data_terminal_ready(true)
        .with_context(|| format!("Cannot assert DTR on serial port {port}"))?;
    serial
        .write_request_to_send(true)
        .with_context(|| format!("Cannot assert RTS on serial port {port}"))?;

    Ok(serial)
}

/// Vrátí true, pokud metadata USB portu vypadají jako Kaputnik/RP2040 CDC zařízení.
fn is_likely_kaputnik_usb(usb: &UsbPortInfo) -> bool {
    let product = usb.product.as_deref().unwrap_or_default().to_lowercase();
    let manufacturer = usb
        .manufacturer
        .as_deref()
        .unwrap_or_default()
        .to_lowercase();

    [
        "kaputnik",
        "rp2040",
        "raspberry",
        "pico",
        "cdc",
        "usb serial",
    ]
    .iter()
    .any(|keyword| product.contains(keyword) || manufacturer.contains(keyword))
}

/// Auto-detekuje nejvhodnější sériový port.
///
/// Priorita:
/// 1) USB port, který metadata odpovídá Kaputnik/RP2040
/// 2) libovolný USB port
/// 3) fallback podle názvu portu na platformě (COMx na Windows, ttyACM/ttyUSB na Linuxu)
/// 4) první dostupný port
fn detect_auto_port() -> Result<String> {
    let ports = serialport::available_ports().context("Cannot list serial ports")?;

    if ports.is_empty() {
        bail!("No serial ports found. Connect the device and try again.");
    }

    if let Some(port) = ports.iter().find(|p| {
        matches!(&p.port_type, serialport::SerialPortType::UsbPort(usb) if is_likely_kaputnik_usb(usb))
    }) {
        return Ok(port.port_name.clone());
    }

    if let Some(port) = ports.iter().find(|p| {
        matches!(&p.port_type, serialport::SerialPortType::UsbPort(_))
    }) {
        return Ok(port.port_name.clone());
    }

    if cfg!(windows) {
        if let Some(port) = ports.iter().find(|p| p.port_name.to_uppercase().starts_with("COM")) {
            return Ok(port.port_name.clone());
        }
    } else if let Some(port) = ports
        .iter()
        .find(|p| p.port_name.contains("ttyACM") || p.port_name.contains("ttyUSB"))
    {
        return Ok(port.port_name.clone());
    }

    Ok(ports[0].port_name.clone())
}

/// Vrátí explicitně zadaný port, nebo auto-detekovaný port.
fn resolve_port_name(cli_port: Option<&str>) -> Result<String> {
    if let Some(port) = cli_port {
        return Ok(port.to_string());
    }

    let detected = detect_auto_port()?;
    eprintln!("Auto-detected serial port: {detected}");
    Ok(detected)
}

/// Odešle textový příkaz na zařízení (ukončený \r\n).
/// Po odeslání čeká 100 ms, aby firmware stihl příkaz zpracovat.
fn send_command(port: &mut Box<dyn serialport::SerialPort>, cmd: &str) -> Result<()> {
    port.write_all(cmd.as_bytes())?;
    port.write_all(b"\r\n")?;
    port.flush()?;
    // Small delay to let device process
    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

/// Po otevření CDC portu na RP2040 může dojít k resetu firmware.
/// Počká na dokončení bootu a odčerpá úvodní banner, aby další příkazy
/// už šly na připravené zařízení.
fn settle_port_after_open(port: &mut Box<dyn serialport::SerialPort>) -> Result<()> {
    std::thread::sleep(Duration::from_millis(2800));
    port.set_timeout(Duration::from_millis(250))?;
    let _ = read_lines(port)?;
    port.set_timeout(Duration::from_secs(5))?;
    Ok(())
}

/// Přečte jeden textový řádek přímo ze sériového portu bez klonování handle.
/// Vrací `None`, pokud během timeoutu nepřišla žádná data.
fn read_line(port: &mut Box<dyn serialport::SerialPort>) -> Result<Option<String>> {
    let mut buf = [0u8; 1];
    let mut line = Vec::new();

    loop {
        match port.read(&mut buf) {
            Ok(0) => {
                if line.is_empty() {
                    return Ok(None);
                }
                break;
            }
            Ok(_) => match buf[0] {
                b'\n' => break,
                b'\r' => {}
                byte => line.push(byte),
            },
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                if line.is_empty() {
                    return Ok(None);
                }
                break;
            }
            Err(e) => return Err(e.into()),
        }
    }

    Ok(Some(String::from_utf8_lossy(&line).trim().to_string()))
}

/// Čte řádky z portu dokud nenarazí na "# END", timeout nebo EOF.
/// Používá se pro jednoduché příkazy (status, erase, start, stop).
fn read_lines(port: &mut Box<dyn serialport::SerialPort>) -> Result<Vec<String>> {
    let mut lines = Vec::new();

    loop {
        match read_line(port)? {
            Some(line) => {
                if line == "# END" {
                    lines.push(line);
                    break;
                }
                if !line.is_empty() {
                    lines.push(line);
                }
            }
            None => break,
        }
    }

    Ok(lines)
}

// =========================================================================
//  Pomocná funkce: formátování epoch sekund → řetězec pro název souboru
// =========================================================================

fn epoch_secs_to_datetime_str(epoch_secs: u64) -> String {
    let secs_per_day = 86400u64;
    let time_of_day = epoch_secs % secs_per_day;
    let days = epoch_secs / secs_per_day;
    let hour = time_of_day / 3600;
    let min = (time_of_day % 3600) / 60;
    let sec = time_of_day % 60;

    // Civil date from days since 1970-01-01 (Howard Hinnant's algorithm)
    let z = days as i64 + 719468;
    let era = if z >= 0 { z } else { z - 146096 } / 146097;
    let doe = (z - era * 146097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };

    format!("{:04}{:02}{:02}_{:02}{:02}{:02}", y, m, d, hour, min, sec)
}

// =========================================================================
//  Příkaz: dump – stahování letových dat
//
//  Průběh:
//  1. Pošle "dump" na zařízení
//  2. Přečte hlavičkové řádky (začínají #) – z nich získá počet vzorků
//  3. Přečte CSV hlavičku (epoch_ms,ax,ay,az,gx,gy,gz,gp14,mx,my,mz,mag_valid)
//  4. Čte datové řádky s progress barem dokud nepřijde "# END"
//  5. Zapisuje do souboru (-o) nebo na stdout
// =========================================================================

fn cmd_dump(port_name: &str, baud: u32, output: Option<String>) -> Result<()> {
    let mut port = open_port(port_name, baud)?;

    settle_port_after_open(&mut port)?;

    // Set longer timeout for dump (large data)
    port.set_timeout(Duration::from_secs(2))?;

    eprintln!("Requesting data dump...");
    send_command(&mut port, "dump")?;

    // Parsování hlavičky – řádky začínající '#' jsou komentáře firmware,
    // první řádek bez '#' je CSV hlavička (názvy sloupců)
    let mut total_samples: u64 = 0;
    // Hlášky z hlavičky firmware
    let mut header_lines: Vec<String> = Vec::new();

    // Čtení hlavičkových řádků (komentáře # a CSV header)
    loop {
        match read_line(&mut port)? {
            Some(trimmed) => {
                if trimmed.starts_with("# Samples:") {
                    if let Some(n) = trimmed.strip_prefix("# Samples:") {
                        total_samples = n.trim().parse().unwrap_or(0);
                    }
                }
                header_lines.push(trimmed.clone());

                if trimmed.starts_with("ERROR") {
                    bail!("{trimmed}");
                }

                if !trimmed.starts_with('#') {
                    break;
                }
            }
            None => bail!("Timeout waiting for data"),
        }
    }

    // Výstup – soubor nebo stdout
    let mut out: Box<dyn Write> = match &output {
        Some(path) => {
            eprintln!("Saving to {path}");
            Box::new(File::create(path).with_context(|| format!("Cannot create {path}"))?)
        }
        None => Box::new(std::io::stdout()),
    };

    // Zápis hlavičky do výstupu
    for hl in &header_lines {
        writeln!(out, "{hl}")?;
    }

    // Progress bar – zobrazí se jen při zápisu do souboru a známém počtu vzorků
    let pb = if total_samples > 0 && output.is_some() {
        let pb = ProgressBar::new(total_samples);
        pb.set_style(
            ProgressStyle::default_bar()
                .template("{spinner:.green} [{bar:40.cyan/blue}] {pos}/{len} samples ({eta})")?
                .progress_chars("=>-"),
        );
        Some(pb)
    } else {
        None
    };

    // Čtení datových řádků CSV až do "# END"
    let mut count: u64 = 0;
    loop {
        match read_line(&mut port)? {
            Some(line) => {
                let trimmed = line.trim();
                if trimmed == "# END" {
                    writeln!(out, "{trimmed}")?;
                    break;
                }
                writeln!(out, "{trimmed}")?;
                count += 1;
                if let Some(ref pb) = pb {
                    pb.set_position(count);
                }
            }
            None => break,
        }
    }

    if let Some(pb) = pb {
        pb.finish_with_message("done");
    }

    eprintln!("Downloaded {count} samples.");
    Ok(())
}

// =========================================================================
//  Příkaz: save – stažení dat do CSV pojmenovaného podle času záznamu
//
//  Přečte "# Epoch start:" z hlavičky dump výstupu a vygeneruje
//  název souboru ve tvaru flight_YYYYMMDD_HHMMSS.csv.
// =========================================================================

fn cmd_save(port_name: &str, baud: u32) -> Result<()> {
    let mut port = open_port(port_name, baud)?;

    settle_port_after_open(&mut port)?;

    // Stop recording first so the header gets written to flash.
    port.set_timeout(Duration::from_secs(5))?;
    eprintln!("Stopping recording...");
    send_command(&mut port, "stop")?;
    let stop_lines = read_lines(&mut port)?;
    for line in &stop_lines {
        eprintln!("{line}");
    }
    // Give firmware time to flush page buffer and write header
    std::thread::sleep(Duration::from_millis(500));

    port.set_timeout(Duration::from_secs(2))?;
    eprintln!("Requesting data dump...");
    send_command(&mut port, "dump")?;

    let mut total_samples: u64 = 0;
    let mut epoch_ms_start: u64 = 0;
    let mut header_lines: Vec<String> = Vec::new();

    loop {
        match read_line(&mut port)? {
            Some(trimmed) => {
                if let Some(n) = trimmed.strip_prefix("# Samples:") {
                    total_samples = n.trim().parse().unwrap_or(0);
                }
                if let Some(n) = trimmed.strip_prefix("# Epoch start:") {
                    epoch_ms_start = n.trim().parse().unwrap_or(0);
                }
                header_lines.push(trimmed.clone());
                if trimmed.starts_with("ERROR") {
                    bail!("{trimmed}");
                }
                if !trimmed.starts_with('#') {
                    break; // CSV header line
                }
            }
            None => bail!("Timeout waiting for data"),
        }
    }

    let filename = if epoch_ms_start > 0 {
        format!("flight_{}.csv", epoch_secs_to_datetime_str(epoch_ms_start / 1000))
    } else {
        "flight_unknown.csv".to_string()
    };

    eprintln!("Saving to {filename}");
    let mut out = File::create(&filename)
        .with_context(|| format!("Cannot create {filename}"))?;

    for hl in &header_lines {
        writeln!(out, "{hl}")?;
    }

    let pb = if total_samples > 0 {
        let pb = ProgressBar::new(total_samples);
        pb.set_style(
            ProgressStyle::default_bar()
                .template("{spinner:.green} [{bar:40.cyan/blue}] {pos}/{len} samples ({eta})")?                .progress_chars("=>-"),
        );
        Some(pb)
    } else {
        None
    };

    let mut count: u64 = 0;
    loop {
        match read_line(&mut port)? {
            Some(line) => {
                let trimmed = line.trim();
                if trimmed == "# END" {
                    writeln!(out, "{trimmed}")?;
                    break;
                }
                writeln!(out, "{trimmed}")?;
                count += 1;
                if let Some(ref pb) = pb {
                    pb.set_position(count);
                }
            }
            None => break,
        }
    }

    if let Some(pb) = pb {
        pb.finish_with_message("done");
    }

    eprintln!("Downloaded {count} samples → {filename}");
    Ok(())
}

// =========================================================================
//  Příkaz: jednoduché operace (status, erase, start, stop)
//
//  Pošle příkaz a vypíše všechny řádky odpovědi.
// =========================================================================

fn cmd_simple(port_name: &str, baud: u32, cmd: &str) -> Result<()> {
    let mut port = open_port(port_name, baud)?;
    settle_port_after_open(&mut port)?;
    send_command(&mut port, cmd)?;

    let lines = read_lines(&mut port)?;
    for line in &lines {
        println!("{line}");
    }
    Ok(())
}

// =========================================================================
//  Příkaz: start – automatická synchronizace času + start záznamu
//
//  1) Získá aktuální epoch sekundy z PC a pošle "settime <epoch>"
//  2) Po potvrzení pošle "start"
//
//  RP2040 nemá RTC s baterií, takže synchronizace před startem
//  zabraňuje zapomenutí ručního kroku.
// =========================================================================

fn cmd_start(port_name: &str, baud: u32) -> Result<()> {
    let epoch_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .context("System clock error")?
        .as_secs();

    let settime_cmd = format!("settime {epoch_secs}");
    eprintln!("Setting device clock to epoch {epoch_secs}...");

    let mut port = open_port(port_name, baud)?;

    // Opening CDC serial on RP2040 can reset the firmware; wait until it is ready
    // to process commands and drain any boot banner output.
    std::thread::sleep(Duration::from_millis(2800));
    port.set_timeout(Duration::from_millis(250))?;
    let _ = read_lines(&mut port);

    port.set_timeout(Duration::from_secs(5))?;

    let mut sync_ok = false;
    for attempt in 1..=3 {
        send_command(&mut port, &settime_cmd)?;
        let sync_lines = read_lines(&mut port)?;
        for line in &sync_lines {
            println!("{line}");
        }

        sync_ok = sync_lines
            .iter()
            .any(|line| line.starts_with("SYNC OK") || line.starts_with("Time set."));
        if sync_ok {
            break;
        }

        eprintln!("Time sync not confirmed (attempt {attempt}/3), retrying...");
        std::thread::sleep(Duration::from_millis(300));
    }

    if !sync_ok {
        bail!("Device did not confirm time sync");
    }
    eprintln!("Time sync confirmed.");

    eprintln!("Starting recording...");

    let mut led_ok = false;
    let mut start_ok = false;
    for attempt in 1..=3 {
        send_command(&mut port, "start")?;
        let start_lines = read_lines(&mut port)?;
        for line in &start_lines {
            println!("{line}");
        }

        start_ok = start_lines.iter().any(|line| {
            line.starts_with("START OK")
                || line.contains("Recording started")
                || line.contains("already recording")
        });
        led_ok = start_lines
            .iter()
            .any(|line| line.contains("LED: BLUE BLINK"));

        if start_ok {
            break;
        }

        eprintln!("Start not confirmed (attempt {attempt}/3), retrying...");
        std::thread::sleep(Duration::from_millis(300));
    }

    if !start_ok {
        bail!("Device did not confirm recording start");
    }
    if led_ok {
        eprintln!("Start confirmed. LED switched to blue blink.");
    } else {
        eprintln!("Start confirmed.");
    }

    Ok(())
}

// =========================================================================
//  Příkaz: list – výpis sériových portů
//
//  Vypíše všechny dostupné porty s typem (USB/PCI/Bluetooth).
//  Užitečné pro nalezení správného COMx nebo /dev/ttyACMx.
// =========================================================================

fn cmd_list() -> Result<()> {
    let ports = serialport::available_ports().context("Cannot list serial ports")?;
    if ports.is_empty() {
        println!("No serial ports found.");
    } else {
        for p in &ports {
            let info = match &p.port_type {
                serialport::SerialPortType::UsbPort(usb) => {
                    let mfg = usb.manufacturer.as_deref().unwrap_or("unknown");
                    let product = usb.product.as_deref().unwrap_or("unknown");
                    let serial = usb.serial_number.as_deref().unwrap_or("n/a");
                    format!(
                        "USB - {} / {} | VID:PID {:04x}:{:04x} | SN: {}",
                        mfg, product, usb.vid, usb.pid, serial
                    )
                }
                serialport::SerialPortType::PciPort => "PCI".to_string(),
                serialport::SerialPortType::BluetoothPort => "Bluetooth".to_string(),
                serialport::SerialPortType::Unknown => "Unknown".to_string(),
            };
            println!("{} ({})", p.port_name, info);
        }
    }
    Ok(())
}

// =========================================================================
//  Vstupní bod – parsování CLI argumentů a dispatch příkazů
// =========================================================================

fn main() -> Result<()> {
    let cli = Cli::parse();

    if matches!(cli.command, Commands::List) {
        return cmd_list();
    }

    let port_name = resolve_port_name(cli.port.as_deref())?;

    match cli.command {
        Commands::Dump { output } => cmd_dump(&port_name, cli.baud, output),
        Commands::Save => cmd_save(&port_name, cli.baud),
        Commands::Status => cmd_simple(&port_name, cli.baud, "status"),
        Commands::Erase => {
            eprintln!("Erasing flash... this may take a while.");
            cmd_simple(&port_name, cli.baud, "erase")
        }
        Commands::Start => cmd_start(&port_name, cli.baud),
        Commands::Stop => cmd_simple(&port_name, cli.baud, "stop"),
        Commands::List => unreachable!(),
    }
}
