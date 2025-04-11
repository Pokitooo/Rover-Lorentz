import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import numpy as np
import serial
import serial.tools.list_ports
import sys

# === Config ===
#SERIAL_PORT = 'COM9'  # Change to your port
BAUD_RATE = 115200
CHANNELS = 288

# === Qt App Setup ===
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="Spectrometer Live Plot")
plot = win.addPlot(title="Spectrometer Output")

# Map 288 bins linearly to 340–850 nm
nm_min = 340
nm_max = 850

# Customize X-axis labels
axis = plot.getAxis('bottom')
nm_ticks = [(i, f"{int(nm_min + i * (nm_max - nm_min) / (CHANNELS - 1))}")
            for i in range(0, CHANNELS, 36)]  # every 36 bins ≈ 65 nm step
axis.setTicks([nm_ticks])
axis.setLabel(text='Wavelength (nm)')

curve = plot.plot(pen='y')
plot.setYRange(0, 1.2)
plot.setXRange(0, CHANNELS - 1)
win.show()

def find_serial_port():
    # List all available serial ports
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        raise Exception("No serial ports found.")
    
    # Print available ports for debugging/informative purposes
    for port in ports:
        print(f"Found port: {port.device}")

    # Strategy for auto-detection: simply return the first port found
    # You might want to add logic to choose a specific port if multiple are available.
    return ports[0].device

# Get a port automatically
SERIAL_PORT = find_serial_port()
print(f"Using port: {SERIAL_PORT}")

# Now, create your serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    # You can now interact with your device via 'ser'
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")

# === Serial Setup ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# === Update Function ===
def update():
    global curve
    line = ser.readline().decode('ascii', errors='ignore').strip()
    try:
        parts = line.strip().split(',')
        values = [float(v) for v in parts if v != '']
        if len(values) == CHANNELS:
            print(f"Received {len(values)} values")
            curve.setData(values)
    except ValueError:
        pass  # Ignore corrupt lines

# === Timer Loop ===
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1000)  # 20 ms ~ 50 FPS

# === Run App ===
if __name__ == '__main__':
    if (sys.flags.interactive != 1) or not hasattr(QtWidgets, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec()