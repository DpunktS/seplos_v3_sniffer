#include "esphome/core/application.h"
#include "seplos_parser.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace seplos_parser {

SeplosParser::SeplosParser(UARTComponent *uart, int bms_count, int throttle_interval)
    : UARTDevice(uart), max_bms_count(bms_count), throttle_interval_(throttle_interval) {
  this->bms.resize(max_bms_count);
}
  for (int i = 0; i < max_bms_count; i++) {
    bms.emplace_back(BMSData());
  }
}

void SeplosParser::setup() {
  ESP_LOGI("seplos_parser", "Setting up SeplosParser...");
  for (int i = 0; i < max_bms_count; i++) {
    std::string prefix = "BMS " + std::to_string(i + 1) + " ";

    BMSData bms_data;

    bms_data.pack_voltage = new esphome::sensor::Sensor();
    bms_data.pack_voltage->set_name(prefix + "pack_voltage");
    bms_data.pack_voltage->set_unit_of_measurement("V");
    bms_data.pack_voltage->set_accuracy_decimals(2);
    bms_data.pack_voltage->add_filter(new sensor::ThrottleFilter(throttle_interval_));

    bms_data.current = new esphome::sensor::Sensor();
    bms_data.current->set_name(prefix + "current");
    bms_data.current->set_unit_of_measurement("A");
    bms_data.current->set_accuracy_decimals(2);
    bms_data.current->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.remaining_capacity = new esphome::sensor::Sensor();
    bms_data.remaining_capacity->set_name(prefix + "remaining_capacity");
    bms_data.remaining_capacity->set_unit_of_measurement("Ah");
    bms_data.remaining_capacity->set_accuracy_decimals(2);
    bms_data.remaining_capacity->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.total_capacity = new esphome::sensor::Sensor();
    bms_data.total_capacity->set_name(prefix + "total_capacity");
    bms_data.total_capacity->set_unit_of_measurement("Ah");
    bms_data.total_capacity->set_accuracy_decimals(2);
    bms_data.total_capacity->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.total_discharge_capacity = new esphome::esphome::sensor::Sensor();
    bms_data.total_discharge_capacity->set_name(prefix + "total_discharge_capacity");
    bms_data.total_discharge_capacity->set_unit_of_measurement("Ah");
    bms_data.total_discharge_capacity->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.soc = new esphome::sensor::Sensor();
    bms_data.soc->set_name(prefix + "soc");
    bms_data.soc->set_unit_of_measurement("%");
    bms_data.soc->set_accuracy_decimals(1);
    bms_data.soc->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.soh = new esphome::sensor::Sensor();
    bms_data.soh->set_name(prefix + "soh");
    bms_data.soh->set_unit_of_measurement("%");
    bms_data.soh->set_accuracy_decimals(1);
    bms_data.soh->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cycle = new esphome::sensor::Sensor();
    bms_data.cycle->set_name(prefix + "cycle");
    bms_data.cycle->set_accuracy_decimals(0);
    bms_data.cycle->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.average_cell_voltage = new esphome::sensor::Sensor();
    bms_data.average_cell_voltage->set_name(prefix + "average_cell_voltage");
    bms_data.average_cell_voltage->set_unit_of_measurement("V");
    bms_data.average_cell_voltage->set_accuracy_decimals(3);
    bms_data.average_cell_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.average_cell_temp = new esphome::sensor::Sensor();
    bms_data.average_cell_temp->set_name(prefix + "average_cell_temp");
    bms_data.average_cell_temp->set_unit_of_measurement("°C");
    bms_data.average_cell_temp->set_accuracy_decimals(1);
    bms_data.average_cell_temp->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.max_cell_voltage = new esphome::sensor::Sensor();
    bms_data.max_cell_voltage->set_name(prefix + "max_cell_voltage");
    bms_data.max_cell_voltage->set_unit_of_measurement("V");
    bms_data.max_cell_voltage->set_accuracy_decimals(3);
    bms_data.max_cell_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.min_cell_voltage = new esphome::sensor::Sensor();
    bms_data.min_cell_voltage->set_name(prefix + "min_cell_voltage");
    bms_data.min_cell_voltage->set_unit_of_measurement("V");
    bms_data.min_cell_voltage->set_accuracy_decimals(3);
    bms_data.min_cell_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.max_cell_temp = new esphome::sensor::Sensor();
    bms_data.max_cell_temp->set_name(prefix + "max_cell_temp");
    bms_data.max_cell_temp->set_unit_of_measurement("°C");
    bms_data.max_cell_temp->set_accuracy_decimals(1);
    bms_data.max_cell_temp->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.min_cell_temp = new esphome::sensor::Sensor();
    bms_data.min_cell_temp->set_name(prefix + "min_cell_temp");
    bms_data.min_cell_temp->set_unit_of_measurement("°C");
    bms_data.min_cell_temp->set_accuracy_decimals(1);
    bms_data.min_cell_temp->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.maxdiscurt = new esphome::sensor::Sensor();
    bms_data.maxdiscurt->set_name(prefix + "maxdiscurt");
    bms_data.maxdiscurt->set_unit_of_measurement("A");
    bms_data.maxdiscurt->set_accuracy_decimals(2);
    bms_data.maxdiscurt->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.maxchgcurt = new esphome::sensor::Sensor();
    bms_data.maxchgcurt->set_name(prefix + "maxchgcurt");
    bms_data.maxchgcurt->set_unit_of_measurement("A");
    bms_data.maxchgcurt->set_accuracy_decimals(2);
    bms_data.maxchgcurt->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_1_voltage = new esphome::sensor::Sensor();
    bms_data.cell_1_voltage->set_name(prefix + "cell_1_voltage");
    bms_data.cell_1_voltage->set_unit_of_measurement("V");
    bms_data.cell_1_voltage->set_accuracy_decimals(3);
    bms_data.cell_1_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_2_voltage = new esphome::sensor::Sensor();
    bms_data.cell_2_voltage->set_name(prefix + "cell_2_voltage");
    bms_data.cell_2_voltage->set_unit_of_measurement("V");
    bms_data.cell_2_voltage->set_accuracy_decimals(3);
    bms_data.cell_2_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_3_voltage = new esphome::sensor::Sensor();
    bms_data.cell_3_voltage->set_name(prefix + "cell_3_voltage");
    bms_data.cell_3_voltage->set_unit_of_measurement("V");
    bms_data.cell_3_voltage->set_accuracy_decimals(3);
    bms_data.cell_3_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_4_voltage = new esphome::sensor::Sensor();
    bms_data.cell_4_voltage->set_name(prefix + "cell_4_voltage");
    bms_data.cell_4_voltage->set_unit_of_measurement("V");
    bms_data.cell_4_voltage->set_accuracy_decimals(3);
    bms_data.cell_4_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_5_voltage = new esphome::sensor::Sensor();
    bms_data.cell_5_voltage->set_name(prefix + "cell_5_voltage");
    bms_data.cell_5_voltage->set_unit_of_measurement("V");
    bms_data.cell_5_voltage->set_accuracy_decimals(3);
    bms_data.cell_5_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_6_voltage = new esphome::sensor::Sensor();
    bms_data.cell_6_voltage->set_name(prefix + "cell_6_voltage");
    bms_data.cell_6_voltage->set_unit_of_measurement("V");
    bms_data.cell_6_voltage->set_accuracy_decimals(3);
    bms_data.cell_6_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_7_voltage = new esphome::sensor::Sensor();
    bms_data.cell_7_voltage->set_name(prefix + "cell_7_voltage");
    bms_data.cell_7_voltage->set_unit_of_measurement("V");
    bms_data.cell_7_voltage->set_accuracy_decimals(3);
    bms_data.cell_7_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_8_voltage = new esphome::sensor::Sensor();
    bms_data.cell_8_voltage->set_name(prefix + "cell_8_voltage");
    bms_data.cell_8_voltage->set_unit_of_measurement("V");
    bms_data.cell_8_voltage->set_accuracy_decimals(3);
    bms_data.cell_8_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_9_voltage = new esphome::sensor::Sensor();
    bms_data.cell_9_voltage->set_name(prefix + "cell_9_voltage");
    bms_data.cell_9_voltage->set_unit_of_measurement("V");
    bms_data.cell_9_voltage->set_accuracy_decimals(3);
    bms_data.cell_9_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_10_voltage = new esphome::sensor::Sensor();
    bms_data.cell_10_voltage->set_name(prefix + "cell_10_voltage");
    bms_data.cell_10_voltage->set_unit_of_measurement("V");
    bms_data.cell_10_voltage->set_accuracy_decimals(3);
    bms_data.cell_10_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_11_voltage = new esphome::sensor::Sensor();
    bms_data.cell_11_voltage->set_name(prefix + "cell_11_voltage");
    bms_data.cell_11_voltage->set_unit_of_measurement("V");
    bms_data.cell_11_voltage->set_accuracy_decimals(3);
    bms_data.cell_11_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_12_voltage = new esphome::sensor::Sensor();
    bms_data.cell_12_voltage->set_name(prefix + "cell_12_voltage");
    bms_data.cell_12_voltage->set_unit_of_measurement("V");
    bms_data.cell_12_voltage->set_accuracy_decimals(3);
    bms_data.cell_12_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_13_voltage = new esphome::sensor::Sensor();
    bms_data.cell_13_voltage->set_name(prefix + "cell_13_voltage");
    bms_data.cell_13_voltage->set_unit_of_measurement("V");
    bms_data.cell_13_voltage->set_accuracy_decimals(3);
    bms_data.cell_13_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_14_voltage = new esphome::sensor::Sensor();
    bms_data.cell_14_voltage->set_name(prefix + "cell_14_voltage");
    bms_data.cell_14_voltage->set_unit_of_measurement("V");
    bms_data.cell_14_voltage->set_accuracy_decimals(3);
    bms_data.cell_14_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_15_voltage = new esphome::sensor::Sensor();
    bms_data.cell_15_voltage->set_name(prefix + "cell_15_voltage");
    bms_data.cell_15_voltage->set_unit_of_measurement("V");
    bms_data.cell_15_voltage->set_accuracy_decimals(3);
    bms_data.cell_15_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_16_voltage = new esphome::sensor::Sensor();
    bms_data.cell_16_voltage->set_name(prefix + "cell_16_voltage");
    bms_data.cell_16_voltage->set_unit_of_measurement("V");
    bms_data.cell_16_voltage->set_accuracy_decimals(3);
    bms_data.cell_16_voltage->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_temp_1 = new esphome::sensor::Sensor();
    bms_data.cell_temp_1->set_name(prefix + "cell_temp_1");
    bms_data.cell_temp_1->set_unit_of_measurement("°C");
    bms_data.cell_temp_1->set_accuracy_decimals(1);
    bms_data.cell_temp_1->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_temp_2 = new esphome::sensor::Sensor();
    bms_data.cell_temp_2->set_name(prefix + "cell_temp_2");
    bms_data.cell_temp_2->set_unit_of_measurement("°C");
    bms_data.cell_temp_2->set_accuracy_decimals(1);
    bms_data.cell_temp_2->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_temp_3 = new esphome::sensor::Sensor();
    bms_data.cell_temp_3->set_name(prefix + "cell_temp_3");
    bms_data.cell_temp_3->set_unit_of_measurement("°C");
    bms_data.cell_temp_3->set_accuracy_decimals(1);
    bms_data.cell_temp_3->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.cell_temp_4 = new esphome::sensor::Sensor();
    bms_data.cell_temp_4->set_name(prefix + "cell_temp_4");
    bms_data.cell_temp_4->set_unit_of_measurement("°C");
    bms_data.cell_temp_4->set_accuracy_decimals(1);
    bms_data.cell_temp_4->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.case_temp = new esphome::sensor::Sensor();
    bms_data.case_temp->set_name(prefix + "case_temp");
    bms_data.case_temp->set_unit_of_measurement("°C");
    bms_data.case_temp->set_accuracy_decimals(1);
    bms_data.case_temp->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    bms_data.power_temp = new esphome::sensor::Sensor();
    bms_data.power_temp->set_name(prefix + "power_temp");
    bms_data.power_temp->set_unit_of_measurement("°C");
    bms_data.power_temp->set_accuracy_decimals(1);
    bms_data.power_temp->add_filter(new esphome::sensor::ThrottleFilter(throttle_interval_));

    this->bms.push_back(bms_data);
  }
  ESP_LOGI("seplos_parser", "Setup completed for %d BMS devices", this->max_bms_count);
}

void SeplosParser::loop() {
  while (this->available()) {
    uint8_t byte = this->read();
    this->buffer.push_back(byte);

    if (this->buffer.size() > 100) {
      buffer.erase(buffer.begin(), buffer.end() - 100);
    }

    while (this->buffer.size() >= 5) {
      if (this->is_valid_header()) {
        size_t expected_length = this->get_expected_length();
        if (buffer.size() < expected_length) break;

        if (this->validate_crc(expected_length)) {
          this->process_packet(expected_length);
          buffer.erase(buffer.begin(), buffer.begin() + expected_length);
        } else {
          ESP_LOGW("seplos_parser", "Invalid CRC, dropping packet.");
          buffer.erase(buffer.begin());
        }
      } else {
        buffer.erase(buffer.begin());
      }
    }
  }
}

bool SeplosParser::is_valid_header() {
  return (this->buffer[0] >= 0x01 && this->buffer[0] <= 0x10) && this->buffer[1] == 0x04 &&
         (this->buffer[2] == 0x24 || this->buffer[2] == 0x34);
}

size_t SeplosParser::get_expected_length() {
  return (this->buffer[2] == 0x24) ? 36 + 2 + 3 : 52 + 2 + 3;
}

bool SeplosParser::validate_crc(size_t length) {
  uint16_t received_crc = (this->buffer[length - 1] << 8) | this->buffer[length - 2];
  uint16_t calculated_crc = this->calculate_modbus_crc(this->buffer, length - 2);
  return received_crc == calculated_crc;
}

void SeplosParser::process_packet(size_t length) {
  int bms_index = this->buffer[0] - 0x01;
  if (bms_index < 0 || bms_index >= this->max_bms_count) {
    ESP_LOGW("seplos_parser", "Invalid BMS ID: %d", buffer[0]);
    return;
  }

  if (buffer[2] == 0x24) {
    // Verarbeite das 36-Byte-Paket und publiziere die Daten
    uint16_t pack_voltage = (this->buffer[4] << 8) | this->buffer[3];
    int16_t current = (this->buffer[6] << 8) | this->buffer[5];
    uint16_t remaining_capacity = (this->buffer[8] << 8) | this->buffer[7];
    uint16_t total_capacity = (this->buffer[10] << 8) | this->buffer[9];
    uint16_t total_discharge_capacity = (this->buffer[12] << 8) | this->buffer[11];
    uint16_t soc = (this->buffer[14] << 8) | this->buffer[13];
    uint16_t soh = (this->buffer[16] << 8) | this->buffer[15];
    uint16_t cycle = (this->buffer[18] << 8) | this->buffer[17];
    uint16_t average_cell_voltage = (this->buffer[20] << 8) | this->buffer[19];
    uint16_t average_cell_temp = (this->buffer[22] << 8) | this->buffer[21];
    uint16_t max_cell_voltage = (this->buffer[24] << 8) | this->buffer[23];
    uint16_t min_cell_voltage = (this->buffer[26] << 8) | this->buffer[25];
    uint16_t max_cell_temp = (this->buffer[28] << 8) | this->buffer[27];
    uint16_t min_cell_temp = (this->buffer[30] << 8) | this->buffer[29];
    uint16_t maxdiscurt = (this->buffer[34] << 8) | this->buffer[33];
    uint16_t maxchgcurt = (this->buffer[36] << 8) | this->buffer[35];

    this->bms[bms_index].pack_voltage->publish_state(pack_voltage / 100);
    this->bms[bms_index].current->publish_state(current / 100);
    this->bms[bms_index].remaining_capacity->publish_state(remaining_capacity / 1000);
    this->bms[bms_index].total_capacity->publish_state(total_capacity / 100);
    this->bms[bms_index].total_discharge_capacity->publish_state(total_discharge_capacity / 0.1);
    this->bms[bms_index].soc->publish_state(soc / 10.0);
    this->bms[bms_index].soh->publish_state(soh / 10.0);
    this->bms[bms_index].cycle->publish_state(cycle);
    this->bms[bms_index].average_cell_voltage->publish_state(average_cell_voltage / 1000);
    this->bms[bms_index].average_cell_temp->publish_state(average_cell_temp / 10 - 273.15);
    this->bms[bms_index].max_cell_voltage->publish_state(max_cell_voltage / 1000);
    this->bms[bms_index].min_cell_voltage->publish_state(min_cell_voltage / 1000);
    this->bms[bms_index].max_cell_temp->publish_state(max_cell_temp / 10 - 273.15);
    this->bms[bms_index].min_cell_temp->publish_state(min_cell_temp / 10 - 273.15);
    this->bms[bms_index].maxdiscurt->publish_state(maxdiscurt / 100);
    this->bms[bms_index].maxchgcurt->publish_state(maxchgcurt / 100);

  } else if (buffer[2] == 0x34) {
    // Verarbeite das 52-Byte-Paket und publiziere die Daten
    uint16_t cell_1 = (this->buffer[4] << 8) | this->buffer[3];
    uint16_t cell_2 = (this->buffer[6] << 8) | this->buffer[5];
    uint16_t cell_3 = (this->buffer[8] << 8) | this->buffer[7];
    uint16_t cell_4 = (this->buffer[10] << 8) | this->buffer[9];
    uint16_t cell_5 = (this->buffer[12] << 8) | this->buffer[11];
    uint16_t cell_6 = (this->buffer[14] << 8) | this->buffer[13];
    uint16_t cell_7 = (this->buffer[16] << 8) | this->buffer[15];
    uint16_t cell_8 = (this->buffer[18] << 8) | this->buffer[17];
    uint16_t cell_9 = (this->buffer[20] << 8) | this->buffer[19];
    uint16_t cell_10 = (this->buffer[22] << 8) | this->buffer[21];
    uint16_t cell_11 = (this->buffer[24] << 8) | this->buffer[23];
    uint16_t cell_12 = (this->buffer[26] << 8) | this->buffer[25];
    uint16_t cell_13 = (this->buffer[28] << 8) | this->buffer[27];
    uint16_t cell_14 = (this->buffer[30] << 8) | this->buffer[29];
    uint16_t cell_15 = (this->buffer[32] << 8) | this->buffer[31];
    uint16_t cell_16 = (this->buffer[34] << 8) | this->buffer[33];
    uint16_t cell_temp_1 = (this->buffer[36] << 8) | this->buffer[35];
    uint16_t cell_temp_2 = (this->buffer[38] << 8) | this->buffer[37];
    uint16_t cell_temp_3 = (this->buffer[40] << 8) | this->buffer[39];
    uint16_t cell_temp_4 = (this->buffer[42] << 8) | this->buffer[41];
    uint16_t case_temp = (this->buffer[52] << 8) | this->buffer[51];
    uint16_t power_temp = (this->buffer[54] << 8) | this->buffer[53];

    this->bms[bms_index].cell_1->publish_state(cell_1 / 1000);
    this->bms[bms_index].cell_2->publish_state(cell_2 / 1000);
    this->bms[bms_index].cell_3->publish_state(cell_3 / 1000);
    this->bms[bms_index].cell_4->publish_state(cell_4 / 1000);
    this->bms[bms_index].cell_5->publish_state(cell_5 / 1000);
    this->bms[bms_index].cell_6->publish_state(cell_6 / 1000);
    this->bms[bms_index].cell_7->publish_state(cell_7 / 1000);
    this->bms[bms_index].cell_8->publish_state(cell_8 / 1000);
    this->bms[bms_index].cell_9->publish_state(cell_9 / 1000);
    this->bms[bms_index].cell_10->publish_state(cell_10 / 1000);
    this->bms[bms_index].cell_11->publish_state(cell_11 / 1000);
    this->bms[bms_index].cell_12->publish_state(cell_12 / 1000);
    this->bms[bms_index].cell_13->publish_state(cell_13 / 1000);
    this->bms[bms_index].cell_14->publish_state(cell_14 / 1000);
    this->bms[bms_index].cell_15->publish_state(cell_15 / 1000);
    this->bms[bms_index].cell_16->publish_state(cell_16 / 1000);
    this->bms[bms_index].cell_temp_1->publish_state(cell_temp_1 / 10 - 273.15);
    this->bms[bms_index].cell_temp_2->publish_state(cell_temp_2 / 10 - 273.15);
    this->bms[bms_index].cell_temp_3->publish_state(cell_temp_3 / 10 - 273.15);
    this->bms[bms_index].cell_temp_4->publish_state(cell_temp_4 / 10 - 273.15);
    this->bms[bms_index].case_temp->publish_state(case_temp / 10 - 273.15);
    this->bms[bms_index].power_temp->publish_state(power_temp / 10 - 273.15);
  }
}

uint16_t SeplosParser::calculate_modbus_crc(const std::vector<uint8_t> &data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < length; pos++) {
    crc ^= static_cast<uint16_t>(data[pos]);
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

}  // namespace seplos_parser
}  // namespace esphome
