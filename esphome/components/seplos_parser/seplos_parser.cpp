#include "seplos_parser.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#include <unordered_map>
#include <sstream>

namespace esphome {
namespace seplos_parser {

static const char *TAG = "seplos_parser.component";

void SeplosParser::setup() {
   // Initialisierung der Sensorvektoren
   std::vector<std::vector<sensor::Sensor *> *> sensor_vectors = {
       &pack_voltage_, &current_, &remaining_capacity_, &total_capacity_,
       &total_discharge_capacity_, &soc_, &soh_, &cycle_count_,
       &average_cell_voltage_, &average_cell_temp_, &max_cell_voltage_,
       &min_cell_voltage_, &delta_cell_voltage_, &max_cell_temp_, &min_cell_temp_,
       &maxdiscurt_, &maxchgcurt_, &cell_1_, &cell_2_, &cell_3_, &cell_4_,
       &cell_5_, &cell_6_, &cell_7_, &cell_8_, &cell_9_, &cell_10_,
       &cell_11_, &cell_12_, &cell_13_, &cell_14_, &cell_15_, &cell_16_,
       &cell_temp_1_, &cell_temp_2_, &cell_temp_3_, &cell_temp_4_,
       &case_temp_, &power_temp_
   };

   for (auto *vec : sensor_vectors) {
       vec->resize(bms_count_, nullptr);
   }

   std::vector<std::vector<text_sensor::TextSensor *> *> text_sensor_vectors = {
       &system_status_, &active_balancing_cells_, &cell_temperature_alarms_,
       &cell_voltage_alarms_, &FET_status_, &active_alarms_, &active_protections_
   };

   for (auto *vec : text_sensor_vectors) {
       vec->resize(bms_count_, nullptr);
   }

   // Zuordnung der Sensornamen zu den jeweiligen Vektoren
   std::unordered_map<std::string, std::vector<sensor::Sensor *> *> sensor_map = {
       {"pack_voltage", &pack_voltage_}, {"current", &current_},
       {"remaining_capacity", &remaining_capacity_}, {"total_capacity", &total_capacity_},
       {"total_discharge_capacity", &total_discharge_capacity_}, {"soc", &soc_},
       {"soh", &soh_}, {"cycle_count", &cycle_count_}, {"average_cell_voltage", &average_cell_voltage_},
       {"average_cell_temp", &average_cell_temp_}, {"max_cell_voltage", &max_cell_voltage_},
       {"min_cell_voltage", &min_cell_voltage_}, {"delta_cell_voltage", &delta_cell_voltage_}, {"max_cell_temp", &max_cell_temp_},
       {"min_cell_temp", &min_cell_temp_}, {"maxdiscurt", &maxdiscurt_},
       {"maxchgcurt", &maxchgcurt_}, {"cell_1", &cell_1_}, {"cell_2", &cell_2_},
       {"cell_3", &cell_3_}, {"cell_4", &cell_4_}, {"cell_5", &cell_5_},
       {"cell_6", &cell_6_}, {"cell_7", &cell_7_}, {"cell_8", &cell_8_},
       {"cell_9", &cell_9_}, {"cell_10", &cell_10_}, {"cell_11", &cell_11_},
       {"cell_12", &cell_12_}, {"cell_13", &cell_13_}, {"cell_14", &cell_14_},
       {"cell_15", &cell_15_}, {"cell_16", &cell_16_}, {"cell_temp_1", &cell_temp_1_},
       {"cell_temp_2", &cell_temp_2_}, {"cell_temp_3", &cell_temp_3_},
       {"cell_temp_4", &cell_temp_4_}, {"case_temp", &case_temp_},
       {"power_temp", &power_temp_}
   };

   std::unordered_map<std::string, std::vector<text_sensor::TextSensor *> *> text_sensor_map = {
       {"system_status", &system_status_}, {"active_balancing_cells", &active_balancing_cells_},
       {"cell_temperature_alarms", &cell_temperature_alarms_}, {"cell_voltage_alarms", &cell_voltage_alarms_},
       {"FET_status", &FET_status_}, {"active_alarms", &active_alarms_},
       {"active_protections", &active_protections_}
   };

   // Zuordnung der Sensor-Objekte
   for (auto &entry : sensor_map) {
       const std::string &name = entry.first;
       std::vector<sensor::Sensor *> *sensor_vector = entry.second;

       for (int i = 0; i < bms_count_; i++) {
           std::string expected_name = "bms" + std::to_string(i) + " " + name;
           for (auto *sensor : this->sensors_) {
               if (sensor->get_name() == expected_name) {
                   (*sensor_vector)[i] = sensor;
               }
           }
       }
   }

   // Zuordnung der Text-Sensor-Objekte
   for (auto &entry : text_sensor_map) {
       const std::string &name = entry.first;
       std::vector<text_sensor::TextSensor *> *text_sensor_vector = entry.second;

       for (int i = 0; i < bms_count_; i++) {
           std::string expected_name = "bms" + std::to_string(i) + " " + name;
           for (auto *sensor : this->text_sensors_) {
               if (sensor->get_name() == expected_name) {
                   (*text_sensor_vector)[i] = sensor;
               }
           }
       }
   }
}


void SeplosParser::loop() {
  while (available()) {
    uint8_t byte = read();
    buffer.push_back(byte);

    if (buffer.size() > 100) {
      buffer.pop_front();
    }
   
    if (buffer.size() >= 5) {
      if (!is_valid_header()) {
        buffer.pop_front();
        continue;
      }

      size_t expected_length = get_expected_length();
      if (buffer.size() >= expected_length) {
        if (validate_crc(expected_length)) {
          process_packet(expected_length);
          //buffer.clear();
          buffer.erase(buffer.begin(), buffer.begin() + expected_length);
          return;  // Nach dem Verarbeiten eines Pakets direkt aus der loop() aussteigen
        } 
        else {
          buffer.pop_front();
        }
      }
    }
  }
}

bool SeplosParser::is_valid_header() {
  return ((buffer[0] >= 0x01 && buffer[0] <= 0x10 && buffer[1] == 0x04 && (buffer[2] == 0x24 || buffer[2] == 0x34)) ||
         (buffer[0] >= 0x01 && buffer[0] <= 0x10 && buffer[1] == 0x01 && buffer[2] == 0x12));
}
size_t SeplosParser::get_expected_length() {
  // +3 Header, +2 CRC, =+5
  if (buffer[2] == 0x24) {return 41;} // (0x24) 36+5=41
  if (buffer[2] == 0x34) {return 57;} // (0x34) 52+5=57
  if (buffer[1] == 0x01 && buffer[2] == 0x12) {return 23;} // (0x12) 18+5=23
  return 0; // If an invalid packet arrives
}
bool SeplosParser::validate_crc(size_t length) {
  uint16_t received_crc = (buffer[length - 1] << 8) | buffer[length - 2];
  uint16_t calculated_crc = calculate_modbus_crc(buffer, length - 2);
  return received_crc == calculated_crc;
}

std::string join_list(const std::vector<int>& list, const std::string& delimiter = ", ") {
  if (list.empty()) return "";
  std::ostringstream oss;
  for (size_t i = 0; i < list.size(); i++) {
    if (i > 0) oss << delimiter;
    oss << list[i];
  }
  return oss.str();
}

std::string join_list(const std::vector<std::string>& list, const std::string& delimiter = ", ") {
  if (list.empty()) return "";
  std::ostringstream oss;
  for (size_t i = 0; i < list.size(); i++) {
    if (i > 0) oss << delimiter;
    oss << list[i];
  }
  return oss.str();
}

void SeplosParser::process_packet(size_t length) {
  int bms_index = buffer[0] - 0x01;
  if (bms_index < 0 || bms_index >= bms_count_) {
    ESP_LOGW("seplos", "Ungültige BMS-ID: %d", buffer[0]);
    return;
  }

  if (buffer[2] == 0x24) {  // 36-Byte-Paket
    //ESP_LOGI("DEBUG", "buffer[3]: 0x%02X, buffer[4]: 0x%02X", buffer[3], buffer[4]);
    std::vector<std::pair<sensor::Sensor*, float>> updates;

    updates.emplace_back(pack_voltage_[bms_index], (buffer[3] << 8 | buffer[4]) / 100.0f);
    updates.emplace_back(current_[bms_index], (int16_t(buffer[5] << 8 | buffer[6])) / 100.0f);
    updates.emplace_back(remaining_capacity_[bms_index], (buffer[7] << 8 | buffer[8]) / 100.0f);
    updates.emplace_back(total_capacity_[bms_index], (buffer[9] << 8 | buffer[10]) / 100.0f);
    updates.emplace_back(total_discharge_capacity_[bms_index], (buffer[11] << 8 | buffer[12]) / 0.1f);
    updates.emplace_back(soc_[bms_index], (buffer[13] << 8 | buffer[14]) / 10.0f);
    updates.emplace_back(soh_[bms_index], (buffer[15] << 8 | buffer[16]) / 10.0f);
    updates.emplace_back(cycle_count_[bms_index], (buffer[17] << 8 | buffer[18]));
    updates.emplace_back(average_cell_voltage_[bms_index], (buffer[19] << 8 | buffer[20]) / 1000.0f);
    updates.emplace_back(average_cell_temp_[bms_index], (buffer[21] << 8 | buffer[22]) / 10.0f - 273.15f);
    updates.emplace_back(max_cell_voltage_[bms_index], (buffer[23] << 8 | buffer[24]) / 1000.0f);
    updates.emplace_back(min_cell_voltage_[bms_index], (buffer[25] << 8 | buffer[26]) / 1000.0f);
    updates.emplace_back(delta_cell_voltage_[bms_index], ((buffer[23] << 8 | buffer[24]) - (buffer[25] << 8 | buffer[26])));
    updates.emplace_back(max_cell_temp_[bms_index], (buffer[27] << 8 | buffer[28]) / 10.0f - 273.15f);
    updates.emplace_back(min_cell_temp_[bms_index], (buffer[29] << 8 | buffer[30]) / 10.0f - 273.15f);
    updates.emplace_back(maxdiscurt_[bms_index], (buffer[33] << 8 | buffer[34]) / 1.0f);
    updates.emplace_back(maxchgcurt_[bms_index], (buffer[35] << 8 | buffer[36]) / 1.0f);

    for (auto &pair : updates) {
      auto *sensor = pair.first;
      auto value = pair.second;
      if (sensor != nullptr) {
        sensor->publish_state(value);
      }
    }
  }

  if (buffer[2] == 0x34) {
    std::vector<std::pair<sensor::Sensor*, float>> updates;

    updates.emplace_back(cell_1_[bms_index], (buffer[3] << 8 | buffer[4]) / 1000.0f);
    updates.emplace_back(cell_2_[bms_index], (buffer[5] << 8 | buffer[6]) / 1000.0f);
    updates.emplace_back(cell_3_[bms_index], (buffer[7] << 8 | buffer[8]) / 1000.0f);
    updates.emplace_back(cell_4_[bms_index], (buffer[9] << 8 | buffer[10]) / 1000.0f);
    updates.emplace_back(cell_5_[bms_index], (buffer[11] << 8 | buffer[12]) / 1000.0f);
    updates.emplace_back(cell_6_[bms_index], (buffer[13] << 8 | buffer[14]) / 1000.0f);
    updates.emplace_back(cell_7_[bms_index], (buffer[15] << 8 | buffer[16]) / 1000.0f);
    updates.emplace_back(cell_8_[bms_index], (buffer[17] << 8 | buffer[18]) / 1000.0);
    updates.emplace_back(cell_9_[bms_index], (buffer[19] << 8 | buffer[20]) / 1000.0f);
    updates.emplace_back(cell_10_[bms_index], (buffer[21] << 8 | buffer[22]) / 1000.0f);
    updates.emplace_back(cell_11_[bms_index], (buffer[23] << 8 | buffer[24]) / 1000.0f);
    updates.emplace_back(cell_12_[bms_index], (buffer[25] << 8 | buffer[26]) / 1000.0f);
    updates.emplace_back(cell_13_[bms_index], (buffer[27] << 8 | buffer[28]) / 1000.0f);
    updates.emplace_back(cell_14_[bms_index], (buffer[29] << 8 | buffer[30]) / 1000.0f);
    updates.emplace_back(cell_15_[bms_index], (buffer[31] << 8 | buffer[32]) / 1000.0f);
    updates.emplace_back(cell_16_[bms_index], (buffer[33] << 8 | buffer[34]) / 1000.0f);
    updates.emplace_back(cell_temp_1_[bms_index], (buffer[35] << 8 | buffer[36]) / 10.0f - 273.15f);
    updates.emplace_back(cell_temp_2_[bms_index], (buffer[37] << 8 | buffer[38]) / 10.0f - 273.15f);
    updates.emplace_back(cell_temp_3_[bms_index], (buffer[39] << 8 | buffer[40]) / 10.0f - 273.15f);
    updates.emplace_back(cell_temp_4_[bms_index], (buffer[41] << 8 | buffer[42]) / 10.0f - 273.15f);
    updates.emplace_back(case_temp_[bms_index], (buffer[51] << 8 | buffer[52]) / 10.0f - 273.15f);
    updates.emplace_back(power_temp_[bms_index], (buffer[53] << 8 | buffer[54]) / 10.0f - 273.15f);

    for (auto &pair : updates) {
      auto *sensor = pair.first;
      auto value = pair.second;
      if (sensor != nullptr) {
        sensor->publish_state(value);
      }
    }
  }
  if (buffer[2] == 0x12) {
    //ESP_LOGW("seplos", "BMS-ID 0x12: %d", buffer[0]);
    std::vector<std::string> active_alarms;
    std::vector<std::string> active_protections;
    std::vector<int> low_voltage_cells, high_voltage_cells;
    std::vector<int> low_temp_cells, high_temp_cells;
    std::vector<int> balancing_cells;
    std::vector<std::string> system_status;
    std::vector<std::string> fet_status;

    auto parse_bits = [](uint8_t byte, int offset) {
      std::vector<int> result;
      for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
          result.push_back(i + offset);
        }
      }
      return result;
    };

    low_voltage_cells = parse_bits(buffer[3], 1);
    auto low_v2 = parse_bits(buffer[4], 9);
    low_voltage_cells.insert(low_voltage_cells.end(), low_v2.begin(), low_v2.end());

    high_voltage_cells = parse_bits(buffer[5], 1);
    auto high_v2 = parse_bits(buffer[6], 9);
    high_voltage_cells.insert(high_voltage_cells.end(), high_v2.begin(), high_v2.end());

    low_temp_cells = parse_bits(buffer[7], 1);
    high_temp_cells = parse_bits(buffer[8], 1);

    balancing_cells = parse_bits(buffer[9], 1);
    auto bal2 = parse_bits(buffer[10], 9);
    balancing_cells.insert(balancing_cells.end(), bal2.begin(), bal2.end());

    if (buffer[11] & 0x01) system_status.push_back("Discharge");
    if (buffer[11] & 0x02) system_status.push_back("Charge");
    if (buffer[11] & 0x04) system_status.push_back("Floating Charge");
    if (buffer[11] & 0x08) system_status.push_back("Full Charge");
    if (buffer[11] & 0x10) system_status.push_back("Standby Mode");
    if (buffer[11] & 0x20) system_status.push_back("Turn Off");

    if (buffer[12] & 0x01) active_alarms.push_back("Cell High Voltage Alarm");
    if (buffer[12] & 0x02) active_protections.push_back("Cell Over Voltage Protection");
    if (buffer[12] & 0x04) active_alarms.push_back("Cell Low Voltage Alarm");
    if (buffer[12] & 0x08) active_protections.push_back("Cell Under Voltage Protection");
    if (buffer[12] & 0x10) active_alarms.push_back("Pack High Voltage Alarm");
    if (buffer[12] & 0x20) active_protections.push_back("Pack Over Voltage Protection");
    if (buffer[12] & 0x40) active_alarms.push_back("Pack Low Voltage Alarm");
    if (buffer[12] & 0x80) active_protections.push_back("Pack Under Voltage Protection");

    if (buffer[13] & 0x01) active_alarms.push_back("Charge High Temperature Alarm");
    if (buffer[13] & 0x02) active_protections.push_back("Charge High Temperature Protection");
    if (buffer[13] & 0x04) active_alarms.push_back("Charge Low Temperature Alarm");
    if (buffer[13] & 0x08) active_protections.push_back("Charge Under Temperature Protection");
    if (buffer[13] & 0x10) active_alarms.push_back("Discharge High Temperature Alarm");
    if (buffer[13] & 0x20) active_protections.push_back("Discharge Over Temperature Protection");
    if (buffer[13] & 0x40) active_alarms.push_back("Discharge Low Temperature Alarm");
    if (buffer[13] & 0x80) active_protections.push_back("Discharge Under Temperature Protection");

    if (buffer[14] & 0x01) active_alarms.push_back("High Environment Temperature Alarm");
    if (buffer[14] & 0x02) active_protections.push_back("Over Environment Temperature Protection");
    if (buffer[14] & 0x04) active_alarms.push_back("Low Environment Temperature Alarm");
    if (buffer[14] & 0x08) active_protections.push_back("Under Environment Temperature Protection");
    if (buffer[14] & 0x10) active_alarms.push_back("High Power Temperature Alarm");
    if (buffer[14] & 0x20) active_protections.push_back("Over Power Temperature Protection");
    if (buffer[14] & 0x40) active_alarms.push_back("Cell Temperature Low Heating");

    if (buffer[15] & 0x01) active_alarms.push_back("Charge Current Alarm");
    if (buffer[15] & 0x02) active_protections.push_back("Charge Over Current Protection");
    if (buffer[15] & 0x04) active_protections.push_back("Charge Second Level Current Protection");
    if (buffer[15] & 0x08) active_alarms.push_back("Discharge Current Alarm");
    if (buffer[15] & 0x10) active_protections.push_back("Discharge Over Current Protection");
    if (buffer[15] & 0x20) active_protections.push_back("Discharge Second Level Over Current Protection");
    if (buffer[15] & 0x40) active_protections.push_back("Output Short Circuit Protection");

    if (buffer[16] & 0x01) active_alarms.push_back("Output Short Latch Up");
    if (buffer[16] & 0x04) active_alarms.push_back("Second Charge Latch Up");
    if (buffer[16] & 0x08) active_alarms.push_back("Second Discharge Latch Up");

    if (buffer[17] & 0x04) active_alarms.push_back("SOC Alarm");
    if (buffer[17] & 0x08) active_protections.push_back("SOC Protection");
    if (buffer[17] & 0x10) active_alarms.push_back("Cell Difference Alarm");

    if (buffer[18] & 0x01) fet_status.push_back("Discharge FET On");
    if (buffer[18] & 0x02) fet_status.push_back("Charge FET On");
    if (buffer[18] & 0x04) fet_status.push_back("Current Limiting FET On");
    if (buffer[18] & 0x08) fet_status.push_back("Heating On");

    if (buffer[19] & 0x01) active_alarms.push_back("Low SOC Alarm");
    if (buffer[19] & 0x02) active_alarms.push_back("Intermittent Charge");
    if (buffer[19] & 0x04) active_alarms.push_back("External Switch Conrol");
    if (buffer[19] & 0x08) active_alarms.push_back("Static Standy Sleep Mode");
    if (buffer[19] & 0x10) active_alarms.push_back("History Data Recording");
    if (buffer[19] & 0x20) active_protections.push_back("Under SOC Protections");
    if (buffer[19] & 0x40) active_alarms.push_back("Active Limited Current");
    if (buffer[19] & 0x80) active_alarms.push_back("Passive Limited Current");

    if (buffer[20] & 0x01) active_protections.push_back("NTC Fault");
    if (buffer[20] & 0x02) active_protections.push_back("AFE Fault");
    if (buffer[20] & 0x04) active_protections.push_back("Charge Mosfet Fault");
    if (buffer[20] & 0x08) active_protections.push_back("Discharge Mosfet Fault");
    if (buffer[20] & 0x10) active_protections.push_back("Cell Fault");
    if (buffer[20] & 0x20) active_protections.push_back("Break Line Fault");
    if (buffer[20] & 0x40) active_protections.push_back("Key Fault");
    if (buffer[20] & 0x80) active_protections.push_back("Aerosol Alarm");

    std::string volt_str = join_list(low_voltage_cells, ", ");
    if (!volt_str.empty() && !high_voltage_cells.empty()) volt_str += " | " + join_list(high_voltage_cells, ", ");
    else volt_str += join_list(high_voltage_cells, ", ");

    std::string temp_str = join_list(low_temp_cells, ", ");
    if (!temp_str.empty() && !high_temp_cells.empty()) temp_str += " | " + join_list(high_temp_cells, ", ");
    else temp_str += join_list(high_temp_cells, ", ");

    if (should_update(bms_index)) {
      if (cell_voltage_alarms_[bms_index]) cell_voltage_alarms_[bms_index]->publish_state(volt_str);
      if (cell_temperature_alarms_[bms_index]) cell_temperature_alarms_[bms_index]->publish_state(temp_str);
      if (active_balancing_cells_[bms_index]) active_balancing_cells_[bms_index]->publish_state(join_list(balancing_cells, ", "));
      if (system_status_[bms_index]) system_status_[bms_index]->publish_state(join_list(system_status, ", "));
      if (FET_status_[bms_index]) FET_status_[bms_index]->publish_state(join_list(fet_status, ", "));
      if (active_alarms_[bms_index]) active_alarms_[bms_index]->publish_state(join_list(active_alarms, ", "));
      if (active_protections_[bms_index]) active_protections_[bms_index]->publish_state(join_list(active_protections, ", "));
    }
  }
}

const uint16_t crc_table[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t SeplosParser::calculate_modbus_crc(const std::deque<uint8_t> &data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t index = crc ^ data[i];
    crc = (crc >> 8) ^ crc_table[index];
  }
  return crc;
}

void SeplosParser::dump_config(){
    for (int i = 0; i < bms_count_; i++) {
     last_updates_[i] = millis();
     //ESP_LOGD("SeplosParser", "Initialisiere Timer für BMS %d: %u", i, last_updates_[i]);
    }
    for (auto *sensor : this->sensors_) {
        LOG_SENSOR("  ", "Sensor", sensor);
    }
    
    for(auto *text_sensor : this->text_sensors_){
        LOG_TEXT_SENSOR("  ", "Text sensor", text_sensor);
    }

//    for(auto *binary_sensor : this->binary_sensors_){
//        LOG_BINARY_SENSOR("  ", "Binary sensor", binary_sensor);
//    }
}
void SeplosParser::set_bms_count(int bms_count) {
  this->bms_count_ = bms_count;  // Wert speichern
  last_updates_.resize(bms_count, 0);  // Dynamische Größe
  ESP_LOGI("SeplosParser", "BMS Count gesetzt auf: %d", bms_count);
}
void SeplosParser::set_update_interval(int update_interval) {
  this->update_interval_ = update_interval*1000;
  ESP_LOGI("SeplosParser", "update interval: %d", update_interval);
}
bool SeplosParser::should_update(int bms_index) {
  if (bms_index < 0 || bms_index >= bms_count_) {
    //ESP_LOGW("SeplosParser", "Ungültiger BMS-Index: %d (max: %d)", bms_index, bms_count_);
    return false; // Ungültiger Index
  }

  uint32_t now = millis();
  //ESP_LOGD("SeplosParser", "BMS %d: now=%u, last_update=%u, interval=%u", 
  //          bms_index, now, last_updates_[bms_index], update_interval_);
  if (now - last_updates_[bms_index] >= update_interval_) {
    last_updates_[bms_index] = now; // Setze den Timer für dieses BMS-Gerät zurück
    //ESP_LOGD("SeplosParser", "Update für BMS %d durchgeführt", bms_index);
    return true;
  }
  //ESP_LOGD("SeplosParser", "Kein Update für BMS %d nötig", bms_index);
  return false;
}

}  // namespace seplos_parser
}  // namespace esphome
