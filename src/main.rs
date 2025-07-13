use std::{collections::VecDeque, fs::File, io::{BufRead, BufReader}, sync::{Arc, Mutex}, thread};
use std::ops::RangeInclusive;
use eframe::Frame;
use egui::{Color32, Context, Vec2b, ViewportBuilder};
use std::time::Duration;
use egui_plot::{Line, Plot, PlotPoints};
use serialport::{ErrorKind, SerialPort};

pub const DEFAULT_CAPACITY: usize = 100_000;
pub const DEFAULT_BAUDRATE: u32 = 115_200;
pub const COMMON_BAUDRATES: [u32; 14] = [
    110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
];

pub const EMPTY_PORT: &str = "Select Port";


#[derive(Debug, Clone)]
enum DataInput {
    None,
    SerialPort(Option<String>, u32),
    File(String),
}

#[derive(Debug, Clone, Default, Eq, PartialEq)]
enum DataInputUi {
    #[default]
    None,
    SerialPort,
    File,
}
impl DataInputUi {
    pub fn str(&self) -> &str {
        match self {
            DataInputUi::None => "None",
            DataInputUi::SerialPort => "Serial Port",
            DataInputUi::File => "File",
        }
    }
}

impl PartialEq for DataInput {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (DataInput::None, DataInput::None) => true,
            (DataInput::SerialPort(_,_), DataInput::SerialPort(_,_)) => true,
            (DataInput::File(_), DataInput::File(_)) => true,
            _ => false,
        }
    }
}


#[derive(Debug)]
pub struct Data {
    data_points: usize,
    lc_all: VecDeque<DataRecord>,
    ps_all: VecDeque<DataRecord>,
    data_input: DataInput,
    is_serial_port_open: bool,
    request_stop: bool,
    latest_size: usize,
    lc_stats: Stats,
    ps_stats: Stats,
}

#[derive(Debug, Clone)]
pub struct Stats {
    max: f64,
    min: f64,
    latest_max: f64,
    latest_min: f64,
    latest_avg: f64,
}
impl Default for Stats {
    fn default() -> Self {
        Stats {
            max: f64::NEG_INFINITY,
            min: f64::INFINITY,
            latest_max: f64::NEG_INFINITY,
            latest_min: f64::INFINITY,
            latest_avg: 0.0,
        }
    }
}

#[derive(Debug)]
pub struct DataRecord {
    pub time: u64,
    pub value: f64,
}

#[derive(Default, serde::Serialize, serde::Deserialize, Debug)]
pub struct FileCsvLine {
    pub typ: String,
    pub time: u64,
    pub value: f64,
}

#[derive(Default)]
pub struct UiData {
    file_path: String,
    boundrate: u32,
    port_list: Vec<String>,
    selected_input: DataInputUi,
}

pub struct VisualizerApp {
    data: Arc<Mutex<Data>>,
    ui_data: UiData,
}

impl VisualizerApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let data = Data{
            data_points: DEFAULT_CAPACITY,
            lc_all: VecDeque::with_capacity(DEFAULT_CAPACITY),
            ps_all: VecDeque::with_capacity(DEFAULT_CAPACITY),
            data_input: DataInput::None,
            is_serial_port_open: false,
            request_stop: false,
            latest_size: 1000,
            lc_stats: Stats::default(),
            ps_stats: Stats::default(),
        };
        let mut slf: Self = VisualizerApp {
            data: Arc::new(Mutex::new(data)),
            ui_data: UiData {
                port_list: scan_serial_ports(),
                ..UiData::default()
            }
        };
        slf.spawn_input_consumer();

        slf
    }

    fn spawn_input_consumer(&mut self) {
        let data = Arc::clone(&self.data);
        std::thread::spawn(move || Self::read_file(data));
    }

    fn open_reader(data_input: &DataInput) -> Option<Box<dyn BufRead>> {
        println!("Using data input: {:?}", data_input);
        match data_input {
            DataInput::None => {
                eprintln!("[PORT][ERR]### No port selected");
                None
            }
            DataInput::SerialPort(port, boudrate) => {
                let serial_port = match port {
                    Some(p) => serialport::new(p, *boudrate)
                        .timeout(Duration::from_millis(1000))
                        .open(),
                    None => Err(serialport::Error::new(ErrorKind::InvalidInput, "No port selected")),
                };

                match serial_port {
                    Ok(serial_port) => Some(Box::new(BufReader::new(serial_port))),
                    Err(e) => {
                        eprintln!("[PORT][ERR]### Failed to open port: {}", e);
                        None
                    }
                }
            }
            DataInput::File(path) => {
                let file = File::open(path);
                match file {
                    Ok(file) => Some(Box::new(BufReader::new(file))),
                    Err(e) => {
                        eprintln!("[PORT][ERR]### Failed to open file: {}", e);
                        None
                    }
                }
            }
        }
    }

    fn read_file(data_arc: Arc<Mutex<Data>>) {
        // let file = File::open("/home/patyg/data.csv").unwrap();
        let reader = {
            let mut data = data_arc.lock().unwrap();
            let reader = Self::open_reader(&data.data_input);
            data.is_serial_port_open = reader.is_some();

            reader
        };

        if reader.is_none() {
            eprintln!("[PORT][ERR]### No reader available, exiting.");
            return;
        }
        let mut reader = reader.unwrap();

        let mut buffer = String::new();

        let mut last_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis();

        println!("[PORT][INF]### Starting to read from port...");
        loop {
            buffer.clear();
            if {data_arc.lock().unwrap().request_stop} { // TODO better communication
                eprintln!("[PORT][INF]### Request to stop reading from port.");
                break;
            }

            match reader.read_line(&mut buffer) {
                Ok(0) => {
                    // EOF reached, exit the loop
                    eprintln!("[PORT][INF]### End of file reached.");
                    break;
                }
                Ok(_) => {
                    Self::process_input_line(&data_arc, &buffer);
                    // let now = std::time::SystemTime::now()
                    //     .duration_since(std::time::UNIX_EPOCH)
                    //     .unwrap()
                    //     .as_millis();
                    // if now - last_time > 33 {
                    //     last_time = now;
                    //     thread::sleep(Duration::from_millis(10));
                    // }
                }
                Err(e) => {
                    eprintln!("[PORT][ERR]### Failed to read line: {}", e);
                    break;
                }
            }
        }
        println!("[PORT][INF]### Finished reading from port.");
        drop(reader);
        let mut data = data_arc.lock().unwrap();
        data.request_stop = false;
        data.is_serial_port_open = false;
    }

    fn process_input_line(data_arc: &Arc<Mutex<Data>>, line: &String) {
        //println!("[PORT][PROCESSING]### {}", line);
        // println!("[PORT][PROCESSING]### {}", line);
        if line.is_empty() || (!line.starts_with("lc,") && !line.starts_with("ps,")) {
            return;
        }

        let parts = line.split(',').collect::<Vec<&str>>();
        let t = parts[0];
        let time  = parts[1].parse::<u64>();
        let value = parts[2].trim().parse::<f64>();

        match (time, value) {
            (Ok(time), Ok(value)) => {
                let csv_line = FileCsvLine {
                    typ: t.to_string(),
                    time,
                    value,
                };
                // println!("[PORT][  RECORD  ]### Type: {}, Time: {}, Value: {}", csv_line.typ, csv_line.time, csv_line.value);

                Self::process_file_data(data_arc, csv_line);
            }
            (Err(te), Err(ve)) => {
                eprintln!("[PORT][ERR]### Failed to parse time: {} (Error: {}) and value: {} (Error: {})", parts[1], parts[2], te, ve);
            }
            (Err(e), _) => {
                eprintln!("[PORT][ERR]### Failed to parse time: {}. Error: {}", parts[1], e);
            }
            (_, Err(e)) => {
                eprintln!("[PORT][ERR]### Failed to parse value: {}. Error: {}", parts[2], e);
            }
        }
    }

    fn update_stats(mut stats: Stats, value: f64, all_data: &VecDeque<DataRecord>, latest_size: usize) -> Stats {
        stats.max = stats.max.max(value);
        stats.min = stats.min.min(value);
        stats.latest_max = stats.latest_max.max(value);
        stats.latest_min = stats.latest_min.min(value);
        let corrected_latest_size = all_data.len().min(latest_size);
        stats.latest_avg = all_data.iter()
            .skip(all_data.len().checked_sub(corrected_latest_size).unwrap_or(0))
            .map(|r| r.value)
            .sum::<f64>() / corrected_latest_size as f64;

        stats
    }

    fn process_file_data(data_arc: &Arc<Mutex<Data>>, csv_line: FileCsvLine) {

        let mut data = data_arc.lock().unwrap();
        // eprintln!("{:?}", measurements);
        let record = DataRecord {time: csv_line.time, value: csv_line.value};
        if record.value.is_nan() {
            eprintln!("[PORT][ERR]### Received NaN value, skipping record.");
            return;
        }

        //println!("[PORT][  RECORD  ]### Type: {}, Time: {}, Value: {}", csv_line.typ, record.time, record.value);
        if csv_line.typ == "lc" {
            data.lc_stats = Self::update_stats(data.lc_stats.clone(), record.value, &data.lc_all, data.latest_size);
            data.lc_all.push_back(record);
            if data.lc_all.len() > data.data_points {
                data.lc_all.pop_front();
            }
        } else if csv_line.typ == "ps" {
            data.ps_stats = Self::update_stats(data.ps_stats.clone(), record.value, &data.ps_all, data.latest_size);
            data.ps_all.push_back(record);
            if data.ps_all.len() > data.data_points {
                data.ps_all.pop_front();
            }
        } else {
            eprintln!("[PORT][ERR]### Unknown type: {}", csv_line.typ);
        }
    }

    fn draw_ui(&mut self, ctx: &Context) {
        let data_arc = Arc::clone(&self.data);
        egui::CentralPanel::default().show(ctx, |ui| {
            let data : &mut Data = &mut data_arc.lock().unwrap();
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Data Points: {}", data.data_points));
                    ui.label(format!("LC Count: {}", data.lc_all.len()));
                    ui.label(format!("PS Count: {}", data.ps_all.len()));
                    ui.separator();
                    ui.add(egui::DragValue::new(&mut data.latest_size).range(RangeInclusive::new(0, i32::MAX)));
                    self.draw_input_selector(ui, data);
                });
                ui.horizontal(|ui| {
                    ui.label(format!("LC Stats: Max: {:.2}, Min: {:.2}, Latest Max: {:.2}, Latest Min: {:.2}, Latest Avg: {:.2}",
                        data.lc_stats.max, data.lc_stats.min, data.lc_stats.latest_max, data.lc_stats.latest_min, data.lc_stats.latest_avg));
                    ui.label(format!("PS Stats: Max: {:.2}, Min: {:.2}, Latest Max: {:.2}, Latest Min: {:.2}, Latest Avg: {:.2}",
                        data.ps_stats.max, data.ps_stats.min, data.ps_stats.latest_max, data.ps_stats.latest_min, data.ps_stats.latest_avg));
                });
            });

            // let data : &mut Data = &mut data_arc.lock().unwrap();
            if !data.lc_all.is_empty() {
                draw_chart(ui, data);
            } else {
                ui.label("No lc/ps data Available");
            }
        });
    }


    fn draw_input_selector(&mut self, ui: &mut egui::Ui, data: &mut Data) {
        let mut selected_input = self.ui_data.selected_input.clone();
        egui::ComboBox::from_id_salt("input_selector")
            .selected_text(selected_input.str())
            .show_ui(ui, |ui| {
                let mut value_changed = false;
                value_changed |= ui.selectable_value(&mut selected_input, DataInputUi::None,       DataInputUi::None.str()).clicked();
                value_changed |= ui.selectable_value(&mut selected_input, DataInputUi::File,       DataInputUi::File.str()).clicked();
                value_changed |= ui.selectable_value(&mut selected_input, DataInputUi::SerialPort, DataInputUi::SerialPort.str()).clicked();
                if value_changed {
                    data.data_input = match selected_input {
                        DataInputUi::None => DataInput::None,
                        DataInputUi::File => DataInput::File(self.ui_data.file_path.clone()),
                        DataInputUi::SerialPort => DataInput::SerialPort(None, self.ui_data.boundrate),
                    };
                    println!("[ UI ][INF] Selected input: {:?}", selected_input);
                    self.ui_data.selected_input = selected_input.clone();
                }
            });

        match selected_input {
            DataInputUi::None => return,
            DataInputUi::File => self.draw_input_file_selector(ui, data),
            DataInputUi::SerialPort => self.draw_input_serial_port_selector(ui, data)
        };


        if !data.is_serial_port_open {
            if ui.button("Connect").clicked() {
                self.spawn_input_consumer()
            }
        }
        else if ui.button("Disconnect").clicked() {
            eprintln!("[PORT][INF]### Disconnecting from port (kind of)");
            // TODO: better Disconnect logic
            data.request_stop = true;
        }
    }
    fn draw_input_file_selector(&mut self, ui: &mut egui::Ui, data: &mut Data) {
        if ui.text_edit_singleline(&mut self.ui_data.file_path).changed() {
            data.data_input = DataInput::File(self.ui_data.file_path.clone());
            println!("[ UI ][INF] Selected file: {:?}", self.ui_data.file_path);
        }
    }
    fn draw_input_serial_port_selector(&mut self, ui: &mut egui::Ui, data: &mut Data) {
        let DataInput::SerialPort(mut selected_port,mut selected_baudrate) = data.data_input.clone() else {
            eprintln!("[ UI ][ERR]### drawing serial port selector but Data Input is not a serial port");
            return;
        };

        let ports = self.ui_data.port_list.clone();

        let mut value_changed = false;
        egui::ComboBox::from_id_salt("port_selector")
            .selected_text(selected_port.clone().unwrap_or(EMPTY_PORT.to_string()))
            .show_ui(ui, |ui| {
                value_changed = ui.selectable_value(&mut selected_port, None, EMPTY_PORT.to_string()).clicked();
                for port in ports {
                    value_changed |= ui.selectable_value(&mut selected_port, Some(port.clone()), port).clicked();
                }
            });

        if ui.button("⟳").clicked() {
            self.ui_data.port_list = scan_serial_ports();
        }

        egui::ComboBox::from_id_salt("baudrate_selector")
            .selected_text(selected_baudrate.to_string())
            .show_ui(ui, |ui| {
                for baudrate in COMMON_BAUDRATES {
                    value_changed |= ui.selectable_value(&mut selected_baudrate, baudrate, baudrate.to_string()).clicked();
                }
            });

        if value_changed {
            println!("[ UI ][INF] Selected serial port: {:?} at baudrate: {}", selected_port, selected_baudrate);
            data.data_input = DataInput::SerialPort(selected_port, selected_baudrate);
        }

    }
}

impl eframe::App for VisualizerApp {
    fn update(&mut self, ctx: &Context, _frame: &mut Frame) {
        self.draw_ui(ctx);
        ctx.request_repaint_after(Duration::from_millis(33));
    }
}


fn draw_chart(ui: &mut egui::Ui, data: &Data) {

    egui_extras::StripBuilder::new(ui)
        .sizes(egui_extras::Size::relative(2f32.recip()), 2)
        .vertical(|mut vstrip| {
            vstrip.cell(|ui| {
                egui_extras::StripBuilder::new(ui)
                    .sizes(egui_extras::Size::relative(2f32.recip()), 2)
                    .horizontal(|mut hstrip| {
                        hstrip.cell(|ui| {
                            let lc_points: Vec<[f64; 2]> = data.lc_all.iter()
                                .skip(data.lc_all.len().checked_sub(data.latest_size).unwrap_or(0))
                                .map(|p| [p.time as f64 / 1000.0, p.value])
                                .collect();
                            draw_plot(ui, "latest lc", vec![
                                PlotData { name: "Load Cell", points: lc_points, color: Color32::RED },
                            ]);
                        });
                        hstrip.cell(|ui| {
                            let ps_points: Vec<[f64; 2]> = data.ps_all.iter()
                                .skip(data.ps_all.len().checked_sub(data.latest_size).unwrap_or(0))
                                .map(|p| [p.time as f64 / 1000.0, p.value])
                                .collect();
                            draw_plot(ui, "latest ps", vec![
                                PlotData { name: "Pressure Sensor", points: ps_points, color: Color32::BLUE }
                            ]);
                        });
                    });
            });
            vstrip.cell(|ui| {
                let lc_points: Vec<[f64; 2]> = data.lc_all.iter()
                    .map(|p| [p.time as f64 / 1000.0, p.value])
                    .collect();
                let ps_points: Vec<[f64; 2]> = data.ps_all.iter()
                    .map(|p| [p.time as f64 / 1000.0, p.value])
                    .collect();

                draw_plot(ui,  "All", vec![
                    PlotData { name: "Load Cell", points: lc_points, color: Color32::RED },
                    PlotData { name: "Pressure Sensor", points: ps_points, color: Color32::BLUE }
                ]);
            })
        });
        //.x_axis_formatter(|value, _| timestamp_formatter(value));
}

struct PlotData<'a> {
    name: &'a str,
    points: Vec<[f64; 2]>,
    color: Color32
}
fn draw_plot(ui: &mut egui::Ui, plot_name: &str, lines: Vec<PlotData>) {

    Plot::new(plot_name)
        // .include_y(0)
        // .include_y(100)
        .show(ui, |plot_ui| {
            for plot_data in lines {
                let line = Line::new(plot_data.name, PlotPoints::from(plot_data.points))
                    .color(plot_data.color);
                //     .name(plot_data.name);
                plot_ui.line(line);
            }
            plot_ui.set_auto_bounds(Vec2b::new(true, true));
        });
}


fn scan_serial_ports() -> Vec<String> {
    let ports = serialport::available_ports().unwrap_or_default();
    ports.into_iter().map(|p| p.port_name).collect()
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: ViewportBuilder::default()
            .with_inner_size(egui::Vec2::new(1280.0, 720.0)),
        ..eframe::NativeOptions::default()
    };

    eframe::run_native(
        "Flight Visualizer",
        options,
        Box::new(|cc| Ok(Box::new(VisualizerApp::new(cc)))),
    )
}