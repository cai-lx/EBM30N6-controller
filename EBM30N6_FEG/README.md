# EBM30N6/FEG High Voltage Power Supply Controller

A Python-based GUI application for controlling EBM30N6/FEG electron beam gun high voltage power supplies. This application provides real-time monitoring and control of various parameters including beam voltage, heater current, suppressor voltage, and extractor voltage.

## Features

- Real-time monitoring of beam energy voltage, current, heater parameters, suppressor and extractor values
- Control of beam, heater, suppressor, and extractor outputs
- Safety mechanisms including heater current limiting
- Visual status indicators for system faults and warnings
- Parameter setting for all key operational values
- Connection management with serial communication

## Requirements

- Python 3.6 or higher
- Windows, Linux, or macOS operating system
- Serial connection to EBM30N6/FEG device (typically via USB-to-Serial adapter)

## Installation

1. Clone this repository:
   ```
   git clone https://github.com/yourusername/EBM30N6_FEG.git
   cd EBM30N6_FEG
   ```

2. Install the required packages:
   ```
   pip install -r requirements.txt
   ```

## Usage

1. Connect your computer to the EBM30N6/FEG device via serial connection
2. Run the application:
   ```
   python EBM30_1.py
   ```
3. Use the "Connect" button to establish communication with the device
4. Adjust parameters and enable/disable outputs using the GUI controls

## Safety Notes

- The device includes safety limits for heater current to prevent damage
- Always observe proper electrical safety procedures when working with high voltage equipment
- Monitor system status indicators for fault conditions

## Parameters

- Beam Energy Voltage: Up to 30 kV
- Heater Current: Up to 3000 mA with configurable current limit
- Suppressor Voltage: Up to 1000 V
- Extractor Voltage: Up to 10000 V
- Extractor Trip Current: Configurable from 50 to 770 μA

## Controls

- **Connect**: Establish serial connection to device
- **Reset**: Disable all outputs and reset parameters to defaults
- **Disconnect**: Close serial connection safely
- **Exit**: Quit application and disconnect

## Hardware Connections

By default, the application connects to COM3 at 115200 baud rate. Modify the [DEFAULT_PORT](file:///d:/GitHub/EBM30N6_FEG/EBM30_1.py#L28-L28) constant in the code if your device is on a different port.

## Contributing

Feel free to submit pull requests for bug fixes or improvements. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the MIT License - see the LICENSE file for details.