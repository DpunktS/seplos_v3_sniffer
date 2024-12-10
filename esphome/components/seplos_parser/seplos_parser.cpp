#include "seplos_parser.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace seplos_parser {

static const char *TAG = "seplos_parser.component";

void SeplosParser::setup() {
   pack_voltage_.resize(bms_count_, nullptr);
   current_.resize(bms_count_, nullptr);

   for (auto *sensor : this->sensors_) {
    for (int i = 0; i < bms_count_; i++) {
      // Erstelle die erwarteten Namen für Spannung und Strom
      std::string pack_voltage_name = "bms" + std::to_string(i) + " pack_voltage";
      std::string current_name = "bms" + std::to_string(i) + " current";

      if (sensor->get_name() == pack_voltage_name) {
        bms[i]_pack_voltage = sensor;}
      if (sensor->get_name() == current_name) {
        bms[i]_current = sensor;}
    }
   //for (auto *sensor : this->sensors_) {
   //   if (sensor->get_name() == "bms0 pack_voltage") {
   //         bms0_pack_voltage = sensor;}
   //   if (sensor->get_name() == "bms0 current") {
   //         bms0_current = sensor;}
   //}
}

void SeplosParser::loop() {
  bms0_pack_voltage->publish_state(5.0);
  bms0_current->publish_state(5.0);
  bms1_pack_voltage->publish_state(5.0);
  bms1_current->publish_state(5.0);
  //ESP_LOGI("seplos", "Polling BMS data...");
  //id(bms0_pack_voltage).publish_state(5.0);
  //this->sensors_ "bms0 pack_voltage"->publish_state(5);
  //for (auto *sensor : this->sensors_) {
  //  if (sensor->get_name() == "bms0 pack_voltage") { // Vergleiche den Namen
  //      sensor->publish_state(5.0);                 // Setze den Wert
  //      break;                                      // Beende die Schleife
  //  }
//  }
//  for (int i = 0; i < this->bms_count_; i++) {
//      ESP_LOGI("seplos", "Simulating data for BMS %d", i);
 //     // Hier kann die echte Lese- und Verarbeitungslogik implementiert werden.
 // }
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
