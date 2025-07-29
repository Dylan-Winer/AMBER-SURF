import serial
import csv

import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
for port in ports:
    print(port.device, port.description)

ser = serial.Serial('COM4 USB Serial Device (COM4)', 9600)  # Replace COM3 with your port
filename = "fsr_forces.csv"

with open(filename, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Force1_N", "Force2_N"])
    
    while True:
        line = ser.readline().decode().strip()
        if line == "":
            continue
        try:
            f1, f2 = map(float, line.split(","))
            writer.writerow([f1, f2])
            print(f1, f2)
        except ValueError:
            continue