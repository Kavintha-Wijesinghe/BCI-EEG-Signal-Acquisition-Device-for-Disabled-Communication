# BCI-EEG-Signal-Acquisition-Device-for-Disabled-Communication

**Technologies**: Python, YOLOv9, EasyOCR, OpenCV, PyTorch

## **Here is a demo containing the project output:**
![Smart Surveillance System](https://github.com/Kavintha-Wijesinghe/End-to-End-License-Plate-Detection-Recognition-YOLO-EasyOCR-/blob/main/output_.gif?raw=true)

## Overview

This project focuses on the design and development of an EEG-based Brain-Computer Interface (BCI) system aimed at providing a low-cost solution for individuals with disabilities to communicate and control devices. The system uses Steady-State Visual Evoked Potentials (SSVEP) detected from EEG signals to interact with external devices.

## Features

- **Signal Acquisition**: Captures EEG signals using a low-cost single-channel setup.
- **Signal Processing**: Filters and processes signals to remove noise and artifacts.
- **Feature Extraction**: Uses Fast Fourier Transform (FFT) to extract relevant features.
- **Classification**: Classifies extracted features using a Convolutional Neural Network (CNN) model.
- **Real-Time Interaction**: Provides real-time control of external devices based on EEG signals.
- **Open Source**: All hardware designs and software code are shared under an open-source license to allow for further modification and experimentation.

## Installation

### 1. Clone the repository:

```bash
git clone https://github.com/yourusername/EEG-BCI-System.git
cd EEG-BCI-System
```

### 2. Microcontroller Setup (ESP32)

- Install the Arduino IDE and ESP32 board manager from [ESP32 Arduino](https://github.com/espressif/arduino-esp32).
- Open the `microcontroller_code/BCI_ESP32.ino` in the Arduino IDE.
- Ensure that the correct board and port are selected, and then upload the code to the ESP32.

```cpp
// microcontroller_code/BCI_ESP32.ino

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <ADS1115.h>

ADS1115 ads;  // Create an instance of the ADS1115 ADC

void setup() {
  Serial.begin(115200);
  ads.begin();
}

void loop() {
  int16_t adc0 = ads.readADC_SingleEnded(0);
  int16_t adc1 = ads.readADC_SingleEnded(1);

  // Print raw EEG signal data
  Serial.print("ADC0: ");
  Serial.print(adc0);
  Serial.print("	 ADC1: ");
  Serial.println(adc1);

  delay(10); // Delay for stability
}
```

### 3. Python Dependencies

The software for real-time plotting and the user interface is written in Python. First, ensure that you have Python installed. Then install the required dependencies:

```bash
pip install -r requirements.txt
```

### 4. Running the Software

To visualize the EEG signal data and interact with the BCI system, run the following Python script:

```bash
python eegScope.py
```

### `eegScope.py` (Real-time EEG Plotting)

```python
# eegScope.py

import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

# Serial port for data collection from ESP32
ser = serial.Serial('COM3', 115200)  # Update with your port

app = QtGui.QApplication([])

# Set up the plot
win = pg.GraphicsWindow(title="Real-Time EEG Signal")
plot = win.addPlot(title="EEG Signal")
curve = plot.plot(pen='y')

# Data buffer
data = []

# Update function for real-time plotting
def update():
    global data
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if line:
            data.append(float(line.split(":")[1].strip()))  # Assuming output from ESP32 is in the format "ADC0: value"
            if len(data) > 500:  # Limit data size
                data = data[1:]
            curve.setData(data)

    QtCore.QTimer.singleShot(50, update)

update()
QtGui.QApplication.instance().exec_()
```

### User Interface Code (Control External Devices)

```python
# user_interface.py

from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout
import serial

ser = serial.Serial('COM3', 115200)  # Update with your port

class BCIApp(App):
    def build(self):
        layout = GridLayout(cols=4)

        # Create buttons for controlling external devices
        for i in range(4):
            btn = Button(text=f"Control {i+1}")
            btn.bind(on_press=self.on_button_click)
            layout.add_widget(btn)
        return layout

    def on_button_click(self, instance):
        # Send the control signal based on the button clicked
        control_signal = instance.text.split()[1]
        ser.write(control_signal.encode())  # Sending the control signal to ESP32

if __name__ == '__main__':
    BCIApp().run()
```

### 5. Visualizing the Data

Once the software is running, you can visualize the EEG signal data in real-time. The interface will allow you to focus on specific stimuli, and the system will process and classify the EEG signals to control external devices based on the user's focus.

## Circuit Diagram

For the complete circuit design, refer to the file `circuit_diagrams/EEG_BCI_Circuit_Diagram.pdf`.

## Future Work

- Improve the CNN model to enhance signal classification accuracy.
- Add support for multi-channel EEG data acquisition.
- Integrate additional features such as eye-tracking for enhanced interaction.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

We would like to thank our supervisors and the Department of Electrical and Telecommunication Engineering for their invaluable support throughout this project.

---

### Structure of the Repository:

```plaintext
EEG-BCI-System/
│
├── microcontroller_code/
│   ├── BCI_ESP32.ino           # Code for the ESP32 microcontroller
│
├── python_scripts/
│   ├── eegScope.py              # Real-time EEG data visualization
│   ├── user_interface.py        # GUI for controlling external devices
│   ├── requirements.txt         # Python dependencies
│
├── circuit_diagrams/
│   ├── EEG_BCI_Circuit_Diagram.pdf
│
├── LICENSE
├── README.md                    # This README file
```
