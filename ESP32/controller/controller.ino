#include <autoland.h>
#include <mavbridge.h>
#include <SoftwareSerial.h>

SoftwareSerial fc(12, 13);
MAVBridge bridge(&fc, false);
LandingController& landing_controller = LandingController::get_instance();

void setup()
{
  Serial.begin(115200);
  bridge.init();
  if (!landing_controller.init(&fc))
  {
    Serial.println("Landing controller init fail");
    while (1) {};
  }
}

void loop()
{
  bridge.update();
  if (bridge.is_landing())
    landing_controller.handle_landing(bridge.get_distance());
}
