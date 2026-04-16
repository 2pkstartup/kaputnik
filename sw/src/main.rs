//! ==========================================================================
//!  kaputnik-downloader – CLI nástroj pro Kaputnik flight logger
//! ==========================================================================
//!
//!  Komunikuje s RP2040 firmware přes USB CDC sériovou linku.
//!
//!  Podporované příkazy:
//!    dump   – stažení letových dat jako CSV (s progress barem)
//!    status – zobrazení stavu zařízení (MPU, flash, čas, záznam)
//!    erase  – smazání celé flash paměti
//!    start  – spuštění záznamu vzdáleně
//!    stop   – zastavení záznamu vzdáleně
//!    sync   – synchronizace hodin (nastaví epoch čas na zařízení)
//!    list   – výpis dostupných sériových portů
//!
//!  Výchozí port: /dev/ttyACM0, 115200 baud
//!  Změna: kaputnik-downloader -p /dev/ttyACM1 status

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

// =========================================================================
//  CLI definice (clap derive)
// =========================================================================

#[derive(Parser)]
#[command(name = "kaputnik-downloader")]
#[command(about = "Kaputnik flight data downloader")]
struct Cli {
    /// Sériový port (např. /dev/ttyACM0)
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    port: String,

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
    /// Zobrazit stav zařízení
    Status,
    /// Smazat flash paměť
    Erase,
    /// Spustit záznam
    Start,
    /// Zastavit záznam
    Stop,
    /// Synchronizovat hodiny zařízení s PC
    Sync,
    /// Vypsat dostupné sériové porty
    List,
}

// =========================================================================
//  Pomocné funkce pro sériovou komunikaci
// =========================================================================

/// Otevře sériový port s daným názvem a rychlostí.
/// Timeout 5 s – pokud zařízení neodpoví, operace selhá.
fn open_port(port: &str, baud: u32) -> Result<Box<dyn serialport::SerialPort>> {
    serialport::new(port, baud)
        .timeout(Duration::from_secs(5))
        .open()
        .with_context(|| format!("Cannot open serial port {port}"))
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

/// Čte řádky z portu dokud nenarazí na "# END", timeout nebo EOF.
/// Používá se pro jednoduché příkazy (status, erase, start, stop).
fn read_lines(port: &mut Box<dyn serialport::SerialPort>) -> Result<Vec<String>> {
    let mut reader = BufReader::new(port.try_clone()?);
    let mut lines = Vec::new();
    let mut line = String::new();

    loop {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => break,
            Ok(_) => {
                let trimmed = line.trim().to_string();
                if trimmed == "# END" {
                    lines.push(trimmed);
                    break;
                }
                lines.push(trimmed);
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => break,
            Err(e) => return Err(e.into()),
        }
    }

    Ok(lines)
}

// =========================================================================
//  Příkaz: dump – stahování letových dat
//
//  Průběh:
//  1. Pošle "dump" na zařízení
//  2. Přečte hlavičkové řádky (začínají #) – z nich získá počet vzorků
//  3. Přečte CSV hlavičku (epoch_ms,ax,ay,az,gx,gy,gz)
//  4. Čte datové řádky s progress barem dokud nepřijde "# END"
//  5. Zapisuje do souboru (-o) nebo na stdout
// =========================================================================

fn cmd_dump(port_name: &str, baud: u32, output: Option<String>) -> Result<()> {
    let mut port = open_port(port_name, baud)?;

    // Set longer timeout for dump (large data)
    port.set_timeout(Duration::from_secs(2))?;

    eprintln!("Requesting data dump...");
    send_command(&mut port, "dump")?;

    let mut reader = BufReader::new(port.try_clone()?);
    let mut line = String::new();

    // Parsování hlavičky – řádky začínající '#' jsou komentáře firmware,
    // první řádek bez '#' je CSV hlavička (názvy sloupců)
    let mut total_samples: u64 = 0;
    // Hlášky z hlavičky firmware
    let mut header_lines: Vec<String> = Vec::new();

    // Čtení hlavičkových řádků (komentáře # a CSV header)
    loop {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => bail!("Connection closed unexpectedly"),
            Ok(_) => {
                let trimmed = line.trim().to_string();
                if trimmed.starts_with("# Samples:") {
                    if let Some(n) = trimmed.strip_prefix("# Samples:") {
                        total_samples = n.trim().parse().unwrap_or(0);
                    }
                }
                header_lines.push(trimmed.clone());

                if trimmed.starts_with("ERROR") {
                    bail!("{trimmed}");
                }

                // CSV header line (not a comment)
                if !trimmed.starts_with('#') {
                    break;
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                bail!("Timeout waiting for data");
            }
            Err(e) => return Err(e.into()),
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
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => break,
            Ok(_) => {
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
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => break,
            Err(e) => return Err(e.into()),
        }
    }

    if let Some(pb) = pb {
        pb.finish_with_message("done");
    }

    eprintln!("Downloaded {count} samples.");
    Ok(())
}

// =========================================================================
//  Příkaz: jednoduché operace (status, erase, start, stop)
//
//  Pošle příkaz a vypíše všechny řádky odpovědi.
// =========================================================================

fn cmd_simple(port_name: &str, baud: u32, cmd: &str) -> Result<()> {
    let mut port = open_port(port_name, baud)?;
    send_command(&mut port, cmd)?;

    let lines = read_lines(&mut port)?;
    for line in &lines {
        println!("{line}");
    }
    Ok(())
}

// =========================================================================
//  Příkaz: sync – synchronizace hodin
//
//  Získá aktuální epoch sekundy z PC a pošle "settime <epoch>" na zařízení.
//  RP2040 nemá RTC s baterií, takže čas se musí nastavit před každým letem.
// =========================================================================

fn cmd_sync(port_name: &str, baud: u32) -> Result<()> {
    let epoch_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .context("System clock error")?
        .as_secs();

    let cmd = format!("settime {epoch_secs}");
    eprintln!("Setting device clock to epoch {epoch_secs}...");

    let mut port = open_port(port_name, baud)?;
    send_command(&mut port, &cmd)?;

    let lines = read_lines(&mut port)?;
    for line in &lines {
        println!("{line}");
    }
    Ok(())
}

// =========================================================================
//  Příkaz: list – výpis sériových portů
//
//  Vypíše všechny dostupné porty s typem (USB/PCI/Bluetooth).
//  Užitečné pro nalezení správného /dev/ttyACMx.
// =========================================================================

fn cmd_list() -> Result<()> {
    let ports = serialport::available_ports().context("Cannot list serial ports")?;
    if ports.is_empty() {
        println!("No serial ports found.");
    } else {
        for p in &ports {
            let info = match &p.port_type {
                serialport::SerialPortType::UsbPort(usb) => {
                    format!(
                        "USB - {}",
                        usb.product.as_deref().unwrap_or("unknown")
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

    match cli.command {
        Commands::Dump { output } => cmd_dump(&cli.port, cli.baud, output),
        Commands::Status => cmd_simple(&cli.port, cli.baud, "status"),
        Commands::Erase => {
            eprintln!("Erasing flash... this may take a while.");
            cmd_simple(&cli.port, cli.baud, "erase")
        }
        Commands::Start => cmd_simple(&cli.port, cli.baud, "start"),
        Commands::Stop => cmd_simple(&cli.port, cli.baud, "stop"),
        Commands::Sync => cmd_sync(&cli.port, cli.baud),
        Commands::List => cmd_list(),
    }
}
