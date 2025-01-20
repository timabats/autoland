#pragma once

#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <MAVLink_ardupilotmega.h>
#include <SoftwareSerial.h>

class MAVBridge
{
public:
  MAVBridge(SoftwareSerial* FC, bool ser_debug = false);

  void init();

  void update();

  uint32_t get_flightmode() { return flightmode; }

  bool is_landing() { return flightmode == PLANE_MODE::PLANE_MODE_QLAND; }
  
  uint16_t get_distance() { return distance; }

private:
  WiFiUDP udp;
  IPAddress gcs(255, 255, 255, 255);
  IPAddress broadcast(255, 255, 255, 255);
  bool serialDebug = true;
  uint32_t flightmode;
  uint16_t distance = 0;

  void parse_debug(uint8_t* buf, int packetSize, int chan);

  void fc_to_gcs();

  void gcs_to_fc();

  void send_heartbeat_to_gcs();

};
