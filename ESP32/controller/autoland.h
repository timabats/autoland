#ifndef AUTOLAND_H
#define AUTOLAND_H

#include "esp_camera.h"
#include <MAVLink_ardupilotmega.h>
#include <math.h>
#include <SoftwareSerial.h>

class Camera
{
public:
  bool init();
  void deinit();
  void set_mask(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max);
  void free_snapshot_buf();
  void free_mask_buf();
  bool get_img_and_mask(uint8_t*& img, uint8_t*& mask);
  int get_width() { return frame_width; }
  int get_height() { return frame_height; }

private:
  bool is_initialized = false;
  const int frame_width = 320;
  const int frame_height = 240;
  const int frame_color = 3;
  uint8_t* snapshot_buf;
  uint8_t* mask_buf;

  camera_config_t camera_config = 
  {
    .pin_pwdn = 32,
    .pin_reset = -1,
    .pin_xclk = 0,
    .pin_sscb_sda = 26,
    .pin_sscb_scl = 27,

    .pin_d7 = 35,
    .pin_d6 = 34,
    .pin_d5 = 39,
    .pin_d4 = 36,
    .pin_d3 = 21,
    .pin_d2 = 19,
    .pin_d1 = 18,
    .pin_d0 = 5,
    .pin_vsync = 25,
    .pin_href = 23,
    .pin_pclk = 22,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
  };

  struct Mask
  {
    int min_h = 0;
    int max_h = 0;
    int min_s = 0;
    int max_s = 0;
    int min_v = 0;
    int max_v = 0;
  };

  Mask hsv_mask;

  bool capture_rgb888();
  uint8_t max(uint8_t a, uint8_t b, uint8_t c);
  uint8_t min(uint8_t a, uint8_t b, uint8_t c);
  void rgb2hsv(uint8_t& r, uint8_t& g, uint8_t& b, float& h, float& s, float& v);
  bool apply_mask();
};

class LandingController
{
public:
  static LandingController& get_instance()
  { 
    static LandingController controller;
    return controller;
  }
  bool init(SoftwareSerial* FC);
  void handle_landing(uint16_t dist_cm);
  int get_data(size_t offset, size_t length, float *out_ptr);

private:
  Camera cam;
  float camera_fov = 60;
  float target_d_cm = 50;
  int width = 320;
  int height = 240;
  uint16_t distance_cm;
  int max_err_to_accept = 400;
  uint8_t* cropped_buf;
  int yes_class_id = 1;
  float threshold = 0.6;
  int max_land_dist_cm = 400;
  SoftwareSerial* fc;
  bool debug_nn = false;

  struct Vector3
  {
    int x = 0;
    int y = 0;
    int z = 0;

    float len()
    {
      return sqrt(x * x + y * y + z * z);
    }
  };

  Vector3 diff;

  int radius_estimation(float multiplier = 1);
  int area_estimation();
  int get_pix_ind(uint16_t x, uint16_t y);
  bool check(uint8_t* mask, uint16_t x, uint16_t y);
  void dfs(uint8_t* mask, uint16_t x, uint16_t y, int& area, float& cx, float& cy);
  int find_area(uint8_t* mask, uint16_t st_x, uint16_t st_y, uint16_t& center_x, uint16_t& center_y);
  bool get_mask_center(uint8_t* mask, int& center_x, int& center_y);
  bool classify(uint8_t* img, int center_x, int center_y);
  void send_landing_target();
};

#endif //AUTOLAND_H
