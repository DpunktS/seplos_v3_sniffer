#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/uart/uart.h"
#include <vector>

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
//#ifdef USE_BINARY_SENSOR
//#include "esphome/components/binary_sensor/binary_sensor.h"
//#endif
//#ifdef USE_TEXT_SENSOR
//#include "esphome/components/text_sensor/text_sensor.h"
//#endif

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
//#ifdef USE_TEXT_SENSOR
// protected:
//  std::vector<text_sensor::TextSensor *> text_sensors_;
//
// public:
//  void register_text_sensor(text_sensor::TextSensor *obj) { this->text_sensors_.push_back(obj); }
//#endif
  void set_bms_count(int bms_count);
  void setup() override;
  void loop() override;
  void dump_config() override;
// public:
//  void set_bms_count(int bms_count);  // Methode deklarieren
  // Andere bestehende Methoden und Variablen...

 private:
  int bms_count_;  // Variable zur Speicherung von bms_count
 protected:
  //sensor::Sensor *bms0_pack_voltage = nullptr;
  //sensor::Sensor *bms0_current = nullptr;
  std::vector<sensor::Sensor *> pack_voltage_;
  std::vector<sensor::Sensor *> current_;
};

}  // namespace seplos_parser
}  // namespace esphome
