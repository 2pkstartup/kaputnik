use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[derive(Parser)]
#[command(name = "kaputnik-downloader")]
#[command(about = "Kaputnik flight data downloader")]
struct Cli {
    /// Serial port (e.g. /dev/ttyACM0)
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    port: String,

    /// Baud rate
    #[arg(short, long, default_value_t = 115200)]
    baud: u32,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Download flight data as CSV
    Dump {
        /// Output file (default: stdout)
        #[arg(short, long)]
        output: Option<String>,
    },
    /// Show device status
    Status,
    /// Erase flash memory
    Erase,
    /// Start recording
    Start,
    /// Stop recording
    Stop,
    /// Set device clock to current PC time
    Sync,
    /// List available serial ports
    List,
}

fn open_port(port: &str, baud: u32) -> Result<Box<dyn serialport::SerialPort>> {
    serialport::new(port, baud)
        .timeout(Duration::from_secs(5))
        .open()
        .with_context(|| format!("Cannot open serial port {port}"))
}

fn send_command(port: &mut Box<dyn serialport::SerialPort>, cmd: &str) -> Result<()> {
    port.write_all(cmd.as_bytes())?;
    port.write_all(b"\r\n")?;
    port.flush()?;
    // Small delay to let device process
    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

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

fn cmd_dump(port_name: &str, baud: u32, output: Option<String>) -> Result<()> {
    let mut port = open_port(port_name, baud)?;

    // Set longer timeout for dump (large data)
    port.set_timeout(Duration::from_secs(2))?;

    eprintln!("Requesting data dump...");
    send_command(&mut port, "dump")?;

    let mut reader = BufReader::new(port.try_clone()?);
    let mut line = String::new();

    // Parse header comments to get sample count for progress bar
    let mut total_samples: u64 = 0;
    let mut header_lines: Vec<String> = Vec::new();

    // Read header lines (starting with #) and CSV header
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

    // Open output
    let mut out: Box<dyn Write> = match &output {
        Some(path) => {
            eprintln!("Saving to {path}");
            Box::new(File::create(path).with_context(|| format!("Cannot create {path}"))?)
        }
        None => Box::new(std::io::stdout()),
    };

    // Write header
    for hl in &header_lines {
        writeln!(out, "{hl}")?;
    }

    // Progress bar
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

    // Read data lines
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

fn cmd_simple(port_name: &str, baud: u32, cmd: &str) -> Result<()> {
    let mut port = open_port(port_name, baud)?;
    send_command(&mut port, cmd)?;

    let lines = read_lines(&mut port)?;
    for line in &lines {
        println!("{line}");
    }
    Ok(())
}

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
