# seplos_v3_sniffer

[www.seplos.com](https://www.seplos.com/)

This package is designed to read the Seplos V3 BMS in a multipack configuration. In the V3 generation, the first BMS acts as a Modbus master while all other BMS act as slaves. In this configuration, it is no longer possible to access the BMS from a third device via Modbus, since two master devices cannot exist in an RS-485 Modbus system. The package passively detects the communication between the devices, which does not disrupt the communication of the individual BMS.

In the YAML, all required data must be inserted/modified.
Every 200 ms, the BMS transmits a new data set. The update interval can be changed (default: 5 seconds).

![seplos 4x](https://github.com/user-attachments/assets/9d710287-069d-44b6-acda-e96764642a33)

To establish a connection, pins 1/8 (B), 2/7 (A) and 5 (GND) must be connected to the RS485 adapter of the ESP8622/ESP32. Various RS485 to TTL adapters can be used.

During my tests, I found that the 120 Ohm terminator in the adapter is not necessary. There is also no terminator in the original Seplos V3 USB adapter. If only one BMS is to be read, it is necessary to connect pin 6 (B) to pin 5 (GND) so that the master can send data independently.

![pinout](https://github.com/user-attachments/assets/1c8ec271-d20f-4a5d-baf4-87e5a98fc35a)

The following data points are currently read out:
```
pack_voltage
current
remaining_capacity
total_capacity
total_discharge_capacity
soc
soh
cycle_count
average_cell_voltage
average_cell_temp
max_cell_voltage
min_cell_voltage
delta_cell_voltage
max_cell_temp
min_cell_temp
maxdiscurt
maxchgcurt
cell_1_voltage
cell_2_voltage
cell_3_voltage
cell_4_voltage
cell_5_voltage
cell_6_voltage
cell_7_voltage
cell_8_voltage
cell_9_voltage
cell_10_voltage
cell_11_voltage
cell_12_voltage
cell_13_voltage
cell_14_voltage
cell_15_voltage
cell_16_voltage
cell_temp_1
cell_temp_2
cell_temp_3
cell_temp_4
case_temp
power_temp
system_status
active_balancing_cells
cell_temperature_alarms
cell_voltage_alarms
FET_status
active_alarms
active_protections
```
