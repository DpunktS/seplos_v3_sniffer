#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <deque>

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
//#ifdef USE_BINARY_SENSOR
//#include "esphome/components/binary_sensor/binary_sensor.h"
//#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace seplos_parser {

class SeplosParser : public uart::UARTDevice, public Component {
#ifdef USE_SENSOR
 protected:
  std::vector<sensor::Sensor *> sensors_;

 public:
  void register_sensor(sensor::Sensor *obj) { this->sensors_.push_back(obj); }
#endif
//#ifdef USE_BINARY_SENSOR
// protected:
//  std::vector<binary_sensor::BinarySensor *> binary_sensors_;
//
// public:
//  void register_binary_sensor(binary_sensor::BinarySensor *obj) { this->binary_sensors_.push_back(obj); }
//#endif
#ifdef USE_TEXT_SENSOR
 protected:
  std::vector<text_sensor::TextSensor *> text_sensors_;

 public:
  void register_text_sensor(text_sensor::TextSensor *obj) { this->text_sensors_.push_back(obj); }
#endif

  void set_bms_count(int bms_count);
  void set_update_interval(int update_interval);
  void setup() override;
  void loop() override;
  void dump_config() override;
  bool is_valid_header();
  bool should_update(int bms_index);
  size_t get_expected_length();
  bool validate_crc(size_t length);
  void process_packet(size_t length);
  uint16_t calculate_modbus_crc(const std::deque<uint8_t> &data, size_t length);

private:
  int bms_count_;  // Variable zur Speicherung von bms_count
  uint32_t update_interval_;
  std::vector<uint32_t> last_updates_; // Timer für jedes BMS-Gerät
  //std::vector<uint8_t> buffer;
  std::deque<uint8_t> buffer;

protected:
  std::vector<sensor::Sensor *> pack_voltage_;
  std::vector<sensor::Sensor *> current_;
  std::vector<sensor::Sensor *> remaining_capacity_;
  std::vector<sensor::Sensor *> total_capacity_;
  std::vector<sensor::Sensor *> total_discharge_capacity_;
  std::vector<sensor::Sensor *> soc_;
  std::vector<sensor::Sensor *> soh_;
  std::vector<sensor::Sensor *> cycle_count_;
  std::vector<sensor::Sensor *> average_cell_voltage_;
  std::vector<sensor::Sensor *> average_cell_temp_;
  std::vector<sensor::Sensor *> max_cell_voltage_;
  std::vector<sensor::Sensor *> min_cell_voltage_;
  std::vector<sensor::Sensor *> delta_cell_voltage_;
  std::vector<sensor::Sensor *> max_cell_temp_;
  std::vector<sensor::Sensor *> min_cell_temp_;
  std::vector<sensor::Sensor *> maxdiscurt_;
  std::vector<sensor::Sensor *> maxchgcurt_;
  std::vector<sensor::Sensor *> cell_1_;
  std::vector<sensor::Sensor *> cell_2_;
  std::vector<sensor::Sensor *> cell_3_;
  std::vector<sensor::Sensor *> cell_4_;
  std::vector<sensor::Sensor *> cell_5_;
  std::vector<sensor::Sensor *> cell_6_;
  std::vector<sensor::Sensor *> cell_7_;
  std::vector<sensor::Sensor *> cell_8_;
  std::vector<sensor::Sensor *> cell_9_;
  std::vector<sensor::Sensor *> cell_10_;
  std::vector<sensor::Sensor *> cell_11_;
  std::vector<sensor::Sensor *> cell_12_;
  std::vector<sensor::Sensor *> cell_13_;
  std::vector<sensor::Sensor *> cell_14_;
  std::vector<sensor::Sensor *> cell_15_;
  std::vector<sensor::Sensor *> cell_16_;
  std::vector<sensor::Sensor *> cell_temp_1_;
  std::vector<sensor::Sensor *> cell_temp_2_;
  std::vector<sensor::Sensor *> cell_temp_3_;
  std::vector<sensor::Sensor *> cell_temp_4_;
  std::vector<sensor::Sensor *> case_temp_;
  std::vector<sensor::Sensor *> power_temp_;
  std::vector<text_sensor::TextSensor *> system_status_;
  std::vector<text_sensor::TextSensor *> active_balancing_cells_;
  std::vector<text_sensor::TextSensor *> cell_temperature_alarms_;
  std::vector<text_sensor::TextSensor *> cell_voltage_alarms_;
  std::vector<text_sensor::TextSensor *> FET_status_;
  std::vector<text_sensor::TextSensor *> active_alarms_;
  std::vector<text_sensor::TextSensor *> active_protections_;
};

}  // namespace seplos_parser
}  // namespace esphome
