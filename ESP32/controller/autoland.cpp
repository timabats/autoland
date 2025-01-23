#include <autoland.h>
#include <test_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

bool Camera::init()
{
  if (is_initialized) return true;

  snapshot_buf = new uint8_t[frame_width * frame_height * frame_color];
  mask_buf = new uint8_t[frame_width * frame_height];

  if ((snapshot_buf == nullptr) || (mask_buf == nullptr))
  {
    Serial.println("Memory allocation failed");
    return false;
  }

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    Serial.print("Camera init failed with error 0x");
    Serial.println(err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  
  is_initialized = true;
  return true;
  //TODO param
}

void Camera::deinit()
{
  esp_err_t err = esp_camera_deinit();

  if (err != ESP_OK)
  {
    Serial.println("Camera deinit failed");
  }

  is_initialized = false;
  return;
}

void Camera::set_mask(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max)
{
  hsv_mask.min_h = h_min;
  hsv_mask.max_h = h_max;
  hsv_mask.min_s = s_min;
  hsv_mask.max_s = s_max;
  hsv_mask.min_v = v_min;
  hsv_mask.max_v = v_max;
}

void Camera::free_snapshot_buf()
{
  delete(snapshot_buf);
  snapshot_buf = nullptr;
}

void Camera::free_mask_buf()
{
  delete(mask_buf);
  mask_buf = nullptr;
}

bool Camera::get_img_and_mask(uint8_t*& img, uint8_t*& mask)
{
  if (!is_initialized)
  {
    if (!init())
    {
      Serial.println("Initialization failed");
      return false;
    }
  }

  if (snapshot_buf == nullptr)
    snapshot_buf = new uint8_t[frame_width * frame_height * frame_color];
  if (mask_buf == nullptr)
    mask_buf = new uint8_t[frame_width * frame_height];

  if (!capture_rgb888())
  {
    Serial.println("Failed to capture");
    return false;
  }
  img = snapshot_buf;

  if (!apply_mask())
  {
    Serial.println("Failed to apply mask");
    return false;
  }
  mask = mask_buf;

  return true;
}

bool Camera::capture_rgb888()
{
  if (!is_initialized)
  {
    Serial.println("Camera is not initialized");
    return false;
  }

  camera_fb_t* frame_buf = esp_camera_fb_get();

  if (!frame_buf)
  {
    Serial.println("Camera capture failed");
    return false;
  }

  bool converted = fmt2rgb888(frame_buf->buf, frame_buf->len, PIXFORMAT_JPEG, snapshot_buf);
  esp_camera_fb_return(frame_buf);

  if (!converted)
  {
    Serial.println("Conversion failed");
    return false;
  }

  return true;
}

uint8_t Camera::max(uint8_t a, uint8_t b, uint8_t c)
{
  if (a >= b && a >= c)
    return a;
  else if (b >= c)
    return b;
  return c;
}

uint8_t Camera::min(uint8_t a, uint8_t b, uint8_t c)
{
  if (a <= b && a <= c)
    return a;
  else if (b <= c)
    return b;
  return c;
}

void Camera::rgb2hsv(uint8_t& r, uint8_t& g, uint8_t& b, float& h, float& s, float& v)
{
  uint8_t maximum = max(r, g, b), minimum = min(r, g, b);
  if (maximum == minimum)
  {
    h = 0;
    s = 0;
    v = 0;
    return;
  }
  if (maximum == r)
  {
    h = 60.0f * float(g - b) / float(maximum - minimum);
    if (g < b)
    { 
      h += 360;
    }
  }
  else if (maximum == g)
  {
    h = 60.0f * float(b - r) / float(maximum - minimum) + 120.0f;
  }
  else
  {
    h = 60.0f * float(r - g) / float(maximum - minimum) + 240.0f;
  }
  if (maximum == 0)
    s = 0;
  else
    s = 1 - float(minimum) / float(maximum);
  v = maximum;
  s *= 100.0f;
  v *= 100.0f;
}

bool Camera::apply_mask()
{
  for (int i = 0; i < frame_width * frame_height * frame_color; i += 3)
  {
    uint8_t r = snapshot_buf[i + 2], g = snapshot_buf[i + 1], b = snapshot_buf[i];
    float h, s, v;
    rgb2hsv(r, g, b, h, s, v);
    mask_buf[i / 3] = 0;
    if (h >= hsv_mask.min_h && h <= hsv_mask.max_h)
    {
      if (s >= hsv_mask.min_s && s <= hsv_mask.max_s)
      {
        if (v >= hsv_mask.min_v && v <= hsv_mask.max_v)
        {
          mask_buf[i / 3] = 1;
        }
      }
    }
  }
  return true;
}

bool LandingController::init(SoftwareSerial* FC)
{
  if (!cam.init())
  {
    Serial.println("Camera initialization failed");
    return false;
  }

  int width = cam.get_width();
  int height = cam.get_height();

  fc = FC;

  return true;
}

void LandingController::handle_landing(uint16_t dist_cm)
{
  if (dist_cm > max_land_dist_cm)
    return;
  distance_cm = dist_cm;
  uint8_t *img, *mask;
  cam.get_img_and_mask(img, mask);

  int ctx = 0, cty = 0;
  if (!get_mask_center(mask, ctx, cty))
  {
    Serial.println("Target area not found");
    return;
  }

  if (!classify(img, ctx, cty))
  {
    Serial.println("Found area is not target");
    return;
  }

  diff.x = ctx - (width / 2);
  diff.y = cty - (height / 2);
  diff.z = distance_cm;

  send_landing_target();
}

int LandingController::radius_estimation(float multiplier)
{
  float r_pix = sqrt(width * width + height * height) * (target_d_cm / (2 * distance_cm * tan(camera_fov / 2))) / 2 * multiplier;
  return int(r_pix);
}

int LandingController::area_estimation()
{
  float d_pix = sqrt(width * width + height * height) * (target_d_cm / (2 * distance_cm * tan(camera_fov / 2)));
  float area = d_pix * d_pix * 3.14159 / 4;
  return int(area);
}

int LandingController::get_pix_ind(uint16_t x, uint16_t y)
{
  return int(x) * width + int(y);
}

bool LandingController::check(uint8_t* mask, uint16_t x, uint16_t y)
{
  if (x < 0 || x >= height)
    return false;
  if (y < 0 || y >= width)
    return false;
  return mask[get_pix_ind(x, y)] == 1;
}

void LandingController::dfs(uint8_t* mask, uint16_t x, uint16_t y, int& area, float& cx, float& cy)
{
  mask[get_pix_ind(x, y)] = 2;
  cx -= (cx - x) / (area + 1);
  cy -= (cy - y) / (area + 1);
  ++area;
  for (int dx = -1; dx <= 1; ++dx)
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      if (check(mask, x + dx, y + dy))
        dfs(mask, x + dx, y + dy, area, cx, cy);
    }
  }
}

int LandingController::find_area(uint8_t* mask, uint16_t st_x, uint16_t st_y, uint16_t& center_x, uint16_t& center_y)
{
  float ctx = 0, cty = 0;
  int area = 0;
  dfs(mask, st_x, st_y, area, ctx, cty);
  center_x = uint16_t(ctx);
  center_y = uint16_t(cty);
  return area;
}

bool LandingController::get_mask_center(uint8_t* mask, int& center_x, int& center_y)
{
  int target_area = area_estimation();
  int min_err = int(height) * width;
  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      int pix_ind = i * width + j;
      if (mask[pix_ind] != 1)
        continue;

      uint16_t ctx = 0, cty = 0;
      int area = find_area(mask, i, j, ctx, cty);
      if (abs(area - target_area) < min_err)
      {
        min_err = abs(area - target_area);
        center_x = ctx;
        center_y = cty;
      }
    }
  }
  if (min_err > max_err_to_accept)
    return false;
  return true;
}

int LandingController::get_data(size_t offset, size_t length, float *out_ptr)
{
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0)
  {
      out_ptr[out_ptr_ix] = (cropped_buf[pixel_ix] << 16) + (cropped_buf[pixel_ix + 1] << 8) + cropped_buf[pixel_ix + 2];

      out_ptr_ix++;
      pixel_ix+=3;
      pixels_left--;
  }
  return 0;
}

int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
  LandingController& cont = LandingController::get_instance();
  return cont.get_data(offset, length, out_ptr);
}

bool LandingController::classify(uint8_t* img, int center_x, int center_y)
{
  int radius = radius_estimation(1.5);
  int start_x = max(0, center_x - radius);
  int start_y = max(0, center_y - radius);
  int finish_x = min(height, center_x + radius);
  int finish_y = min(width, center_y + radius);

  cam.free_mask_buf();

  cropped_buf = new uint8_t[width * height * 3];
  ei::image::processing::crop_image_rgb888_packed(img, width, height,
    start_y, start_x, cropped_buf, finish_y - start_y + 1, finish_x - start_x + 1);
  ei::image::processing::resize_image(cropped_buf, finish_y - start_y + 1, finish_x - start_x + 1,
    cropped_buf, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3);
  
  cam.free_snapshot_buf();

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK)
  {
    Serial.println("Classifier error");
    return false;
  }

  delete(cropped_buf);

  if (result.classification[yes_class_id].value > threshold)
    return true;
  return false;
}

void LandingController::send_landing_target()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  float zero_quat[4];
  zero_quat[0] = 1; zero_quat[1] = 0; zero_quat[2] = 0; zero_quat[3] = 0;
  mavlink_msg_landing_target_pack(1, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, micros(),
      0, MAV_FRAME::MAV_FRAME_BODY_FRD, 0, 0, diff.len(), 0, 0, diff.x, diff.y, diff.z, zero_quat,
      LANDING_TARGET_TYPE::LANDING_TARGET_TYPE_VISION_OTHER, 1);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  fc->write(buf, len);
}
