#ifndef IMPLEMENTATION_AND_EXAMPLES_MICROCONTROLLERBATTERYMANAGER_H
#define IMPLEMENTATION_AND_EXAMPLES_MICROCONTROLLERBATTERYMANAGER_H

#include <tuple>

class Agent;

class MicroControllerBatteryManager {

public:
    MicroControllerBatteryManager() = default;

    int bytesPerNode = 36; //Number of bytes in a message (average). It depends on the sign of the x and y coordinate, and the LConfidence.
    int targetSenderIDBytes = 19; //Number of bytes prepended for the target and sender ID

    //Based on the esp32
    // From: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
    //Will change with different output powers (dBm), but we will use normal power for now
//    float bluetoothTransmitConsumption_mA = 130.0; //Consumption of the microcontroller when transmitting via bluetooth
//    float bluetoothReceiveConsumption_mA = 100.0; //Consumption of the microcontroller when receiving via bluetooth
//
//    float wifiTransmitConsumption_mA = 180.0; //Consumption of the microcontroller when transmitting via wifi (Transmit 802.11n, OFDM MCS7, POUT = +14 dBm)
//    float wifiReceiveConsumption_mA = 100.0; //Consumption of the microcontroller when receiving via wifi (Receive 802.11b/g/n)
//
//    float modemSleepConsumption240MHz_ma = 68.0; // 30 mA ~ 68 mA , take worst case. Consumption of the microcontroller when the esp is in modem sleep mode (no RF) at 240MHz

    //Based on the pico 2 w
    //https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
    //hello_serial (max typical user case)
    float regularOperation_250MHz_mA = 38.75; //Consumption of the microcontroller when in regular operation at 200MHz

    //Pico's wifi chip
    //https://www.mouser.com/datasheet/2/196/Infineon_CYW43439_DataSheet_v03_00_EN-3074791.pdf?srsltid=AfmBOoqbvp183iKToLTonbX_Zj0hb7Lp-svrc_1Ce22uR-Iw6ve1raZv

    //Measured it is much less https://www.youtube.com/watch?v=GqmnV_T4yAU
//    float wifiTransmitConsumption_mA = regularOperation_250MHz_mA + 320.0f; //Consumption of the microcontroller when transmitting via wifi
//    float wifiReceiveConsumption_mA = regularOperation_250MHz_mA + 43.0f; //Consumption of the microcontroller when receiving via wifi

    //Measured it is much less https://www.youtube.com/watch?v=GqmnV_T4yAU
    //https://www.jeffgeerling.com/blog/2022/raspberry-pi-pico-w-brings-wifi-6?utm_source=chatgpt.com
    float wifiTransmitConsumption_mA = 80.0f; //Consumption of the microcontroller when transmitting via wifi
    float wifiReceiveConsumption_mA = 80.0f; //Consumption of the microcontroller when receiving via wifi


    float bluetoothTransferSpeed_Mbps = 0.01; //Speed of the bluetooth transfer in Mbps

    //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-throughput
    //https://www.jeffgeerling.com/blog/2022/raspberry-pi-pico-w-brings-wifi-6
    float wifiTransferSpeed_Mbps = 7; //Speed of the wifi transfer in Mbps

    float estimateCommunicationConsumption(Agent* agent, float seconds) const;
    std::pair<float, float>  estimateTransmitConsumption(Agent* agent, float seconds) const;
    std::pair<float, float>  estimateReceiveConsumption(Agent* agent, float seconds) const;

};


#endif //IMPLEMENTATION_AND_EXAMPLES_MICROCONTROLLERBATTERYMANAGER_H