#include <mavbridge.h>

MAVBridge::MAVBridge(SoftwareSerial* FC, bool ser_debug)
{
  serialDebug = ser_debug;
  fc = FC;
  gcs = IPAddress(255, 255, 255, 255);
  broadcast = IPAddress(255, 255, 255, 255);
}

void MAVBridge::init()
{
  if (serialDebug)
    Serial.begin(115200);
  WiFi.softAP("MAVLink", "23232323");
  udp.begin(14550);
  if (fc != nullptr)
    fc->begin(38400);
}

void MAVBridge::update()
{
  if (fc == nullptr)
    send_heartbeat_to_gcs();

  gcs_to_fc();
  fc_to_gcs();
}

void MAVBridge::parse_debug(uint8_t* buf, int packetSize, int chan)
{
  mavlink_message_t msg;
  mavlink_status_t status;
  for (int i = 0; i < packetSize; i++)
  {
    if (mavlink_parse_char(chan, buf[i], &msg, &status))
    {
      if (serialDebug)
      {
        if (chan == 0)
          Serial.print("Plane to GCS: ");
        else
          Serial.print("GCS to Plane: ");
      }
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:
          if (chan == 0)
          {
            flightmode = mavlink_msg_heartbeat_get_custom_mode(&msg);
            if (serialDebug)
            {
              Serial.print("Received HEARTBEAT in mode ");
              Serial.println(flightmode);
            }
          }
          else
          {
            if (serialDebug)
            {
              Serial.println("Received HEARTBEAT");
            }
          }
          break;
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
          if (chan == 0)
          {
            uint8_t id = mavlink_msg_distance_sensor_get_id(&msg);
            uint16_t cur_dist = mavlink_msg_distance_sensor_get_current_distance(&msg);
            if (id == 0)
            {
              distance = cur_dist;
            }
            else if (id == 1 && cur_dist >= 50 && cur_dist <= 600)
            {
              distance = cur_dist;
            }
            if (serialDebug)
            {
              Serial.print("Received DISTANCE_SENSOR with distance ");
              Serial.print(cur_dist);
              Serial.print("cm and ID ");
              Serial.println(id);
            }
          }
          break;
        default:
          if (serialDebug)
          {
            Serial.print("Received message with ID ");
            Serial.println(msg.msgid);
          }
          break;
      }
    }
  }
}

void MAVBridge::fc_to_gcs()
{
  if (fc == nullptr)
    return;

  int size = fc->available();
  if (!size)
    return;

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  fc->read(buf, size);

  parse_debug(buf, size, 0);

  udp.beginPacket(gcs, 14550);
  udp.write(buf, size);
  udp.endPacket();
}

void MAVBridge::gcs_to_fc()
{
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  gcs = udp.remoteIP();
  //Serial.println(gcs);

  // Read UDP packet
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  udp.read(buf, MAVLINK_MAX_PACKET_LEN);

  if (fc != nullptr)
    fc->write(buf, packetSize);

  if (serialDebug)
    parse_debug(buf, packetSize, 1);
}

void MAVBridge::send_heartbeat_to_gcs()
{
  static uint last_sent = millis();
  if (millis() - last_sent < 1000)
    return;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_UART_BRIDGE, &msg, MAV_TYPE_VTOL_TILTROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_STANDBY);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  udp.beginPacket(gcs, 14550);
  udp.write(buf, len);
  udp.endPacket();

  last_sent = millis();
}
