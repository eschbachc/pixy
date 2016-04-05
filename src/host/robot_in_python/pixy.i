%module pixy

%include "stdint.i"
%include "carrays.i"

%{
#define SWIG_FILE_WITH_INIT
#include "pixy.h"
%}

%array_class(struct Block, BlockArray);

int  pixy_init();
void pixy_close();
void pixy_error(int error_code);
int  pixy_blocks_are_new();
int  pixy_get_blocks(uint16_t max_blocks, struct Block * blocks);
int  pixy_rcs_set_position(uint8_t channel, uint16_t position);
int  pixy_command (const char *name,...);
int  pixy_led_set_RGB (uint8_t red, uint8_t green, uint8_t blue);
int  pixy_led_set_max_current (uint32_t current);
int  pixy_led_get_max_current ();
int  pixy_cam_set_auto_white_balance (uint8_t value);
int  pixy_cam_get_auto_white_balance ();
uint32_t pixy_cam_get_white_balance_value ();
int  pixy_cam_set_white_balance_value (uint8_t red, uint8_t green, uint8_t blue);
int  pixy_cam_set_auto_exposure_compensation (uint8_t enable);
int  pixy_cam_get_auto_exposure_compensation ();
int  pixy_cam_set_exposure_compensation (uint8_t gain, uint16_t compensation);
int  pixy_cam_get_exposure_compensation (uint8_t *gain, uint16_t *compensation);
int  pixy_cam_set_brightness (uint8_t brightness);
int  pixy_cam_get_brightness ();
int  pixy_rcs_get_position (uint8_t channel);
int  pixy_rcs_set_frequency (uint16_t frequency);
int  pixy_get_firmware_version (uint16_t *major, uint16_t *minor, uint16_t *build);

struct Block
{
  uint16_t type;
  uint16_t signature;
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
  int16_t  angle;
};



