#include "seplos_parser.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include <unordered_map>

namespace esphome {
namespace seplos_parser {

static const char *TAG = "seplos_parser.component";

void SeplosParser::setup() {
   pack_voltage_.resize(bms_count_, nullptr);
   current_.resize(bms_count_, nullptr);
   remaining_capacity_.resize(bms_count_, nullptr);
   total_capacity_.resize(bms_count_, nullptr);
   total_discharge_capacity_.resize(bms_count_, nullptr);
   soc_.resize(bms_count_, nullptr);
   soh_.resize(bms_count_, nullptr);
   cycle_count_.resize(bms_count_, nullptr);
   average_cell_voltage_.resize(bms_count_, nullptr);
   average_cell_temp_.resize(bms_count_, nullptr);
   max_cell_voltage_.resize(bms_count_, nullptr);
   min_cell_voltage_.resize(bms_count_, nullptr);
   max_cell_temp_.resize(bms_count_, nullptr);
   min_cell_temp_.resize(bms_count_, nullptr);
   maxdiscurt_.resize(bms_count_, nullptr);
   maxchgcurt_.resize(bms_count_, nullptr);
   cell_1_.resize(bms_count_, nullptr);
   cell_2_.resize(bms_count_, nullptr);
   cell_3_.resize(bms_count_, nullptr);
   cell_4_.resize(bms_count_, nullptr);
   cell_5_.resize(bms_count_, nullptr);
   cell_6_.resize(bms_count_, nullptr);
   cell_7_.resize(bms_count_, nullptr);
   cell_8_.resize(bms_count_, nullptr);
   cell_9_.resize(bms_count_, nullptr);
   cell_10_.resize(bms_count_, nullptr);
   cell_11_.resize(bms_count_, nullptr);
   cell_12_.resize(bms_count_, nullptr);
   cell_13_.resize(bms_count_, nullptr);
   cell_14_.resize(bms_count_, nullptr);
   cell_15_.resize(bms_count_, nullptr);
   cell_16_.resize(bms_count_, nullptr);
   cell_temp_1_.resize(bms_count_, nullptr);
   cell_temp_2_.resize(bms_count_, nullptr);
   cell_temp_3_.resize(bms_count_, nullptr);
   cell_temp_4_.resize(bms_count_, nullptr);
   case_temp_.resize(bms_count_, nullptr);
   power_temp_.resize(bms_count_, nullptr);
   
  std::unordered_map<std::string, std::vector<sensor::Sensor *> *> sensor_map = {
    {"pack_voltage", &pack_voltage_},
    {"current", &current_},
    {"remaining_capacity", &remaining_capacity_},
    {"total_capacity", &total_capacity_},
    {"total_discharge_capacity", &total_discharge_capacity_},
    {"soc", &soc_},
    {"soh", &soh_},
    {"cycle_count", &cycle_count_},
    {"average_cell_voltage", &average_cell_voltage_},
    {"average_cell_temp", &average_cell_temp_},
    {"max_cell_voltage", &max_cell_voltage_},
    {"min_cell_voltage", &min_cell_voltage_},
    {"max_cell_temp", &max_cell_temp_},
    {"min_cell_temp", &min_cell_temp_},
    {"maxdiscurt", &maxdiscurt_},
    {"maxchgcurt", &maxchgcurt_},
    {"cell_1", &cell_1_},
    {"cell_2", &cell_2_},
    {"cell_3", &cell_3_},
    {"cell_4", &cell_4_},
    {"cell_5", &cell_5_},
    {"cell_6", &cell_6_},
    {"cell_7", &cell_7_},
    {"cell_8", &cell_8_},
    {"cell_9", &cell_9_},
    {"cell_10", &cell_10_},
    {"cell_11", &cell_11_},
    {"cell_12", &cell_12_},
    {"cell_13", &cell_13_},
    {"cell_14", &cell_14_},
    {"cell_15", &cell_15_},
    {"cell_16", &cell_16_},
    {"cell_temp_1", &cell_temp_1_},
    {"cell_temp_2", &cell_temp_2_},
    {"cell_temp_3", &cell_temp_3_},
    {"cell_temp_4", &cell_temp_4_},
    {"case_temp", &case_temp_},
    {"power_temp", &power_temp_},
  };

  for (auto it = sensor_map.begin(); it != sensor_map.end(); ++it) {
    const std::string &name = it->first;
    std::vector<sensor::Sensor *> *sensor_vector = it->second;

    for (auto *sensor : this->sensors_) {
      for (int i = 0; i < bms_count_; i++) {
        if (sensor->get_name() == "bms" + std::to_string(i) + " " + name) {
        (*sensor_vector)[i] = sensor;
        }
      }
    } 
  }
}

void SeplosParser::loop() {
  while (available()) {
    uint8_t byte = read();
    buffer.push_back(byte);
   
    if (buffer.size() >= 5) {
      if (!is_valid_header()) {
        buffer.pop_front();
        continue;
      }

      size_t expected_length = get_expected_length();
      if (buffer.size() >= expected_length) {
        if (validate_crc(expected_length)) {
          process_packet(expected_length);
          buffer.clear();
          return;  // Nach dem Verarbeiten eines Pakets direkt aus der loop() aussteigen
        } 
        else {
          buffer.clear();
        }
      }
    }
  }
}

bool SeplosParser::is_valid_header() {
  return (buffer[0] >= 0x01 && buffer[0] <= 0x10) &&
          buffer[1] == 0x04 &&
         (buffer[2] == 0x24 || buffer[2] == 0x34);
}
size_t SeplosParser::get_expected_length() {
  return (buffer[2] == 0x24) ? 36 + 2 + 3 : 52 + 2 + 3;
}
bool SeplosParser::validate_crc(size_t length) {
  uint16_t received_crc = (buffer[length - 1] << 8) | buffer[length - 2];
  uint16_t calculated_crc = calculate_modbus_crc(buffer, length - 2);
  return received_crc == calculated_crc;
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
    for (auto *sensor : this->sensors_) {
        LOG_SENSOR("  ", "Sensor", sensor);
    }
    
//    for(auto *text_sensor : this->text_sensors_){
//        LOG_TEXT_SENSOR("  ", "Text sensor", text_sensor);
//    }
//
//    for(auto *binary_sensor : this->binary_sensors_){
//        LOG_BINARY_SENSOR("  ", "Binary sensor", binary_sensor);
//    }
}
void SeplosParser::set_bms_count(int bms_count) {
  this->bms_count_ = bms_count;  // Wert speichern
  ESP_LOGI("SeplosParser", "BMS Count gesetzt auf: %d", bms_count);
}

}  // namespace seplos_parser
}  // namespace esphome
