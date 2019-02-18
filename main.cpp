#include <stdint.h>
#include <MLX90640_API.h>
#include <ucg.h>
#include <bcm2835.h>
#include <unistd.h>
#include <stdlib.h> 
#include <fstream>
#include <chrono>
#include <thread>
#include <math.h>

#define GPIO_PIN_RST 22
#define GPIO_PIN_CD 23
#define GPIO_PIN_CS 8

#define MLX_I2C_ADDR 0x33

#define IMAGE_SCALE 5

// Valid frame rates are 1, 2, 4, 8, 16, 32 and 64
// The i2c baudrate is set to 1mhz to support these
#define FPS 8
#define FRAME_TIME_MICROS (1000000/FPS)

// Despite the framerate being ostensibly FPS hz
// The frame is often not ready in time
// This offset is added to the FRAME_TIME_MICROS
// to account for this.
#define OFFSET_MICROS 850

ucg_t ucg;

static uint16_t eeMLX90640[832];
static paramsMLX90640 mlx90640;

void spi_send(uint32_t len, unsigned char *data){
  bcm2835_spi_writenb((const char *)data, len);
}

int16_t ucg_com_raspberry_pi_4wire_HW_SPI(
    ucg_t *ucg,
    int16_t msg,
    uint16_t arg,
    uint8_t *data
) {
  switch(msg) {
    case UCG_COM_MSG_POWER_UP:
      /* "data" is a pointer to ucg_com_info_t structure with the following information: */
      /*  ((ucg_com_info_t *)data)->serial_clk_speed value in nanoseconds */
      /*  ((ucg_com_info_t *)data)->parallel_clk_speed value in nanoseconds */
      /* "arg" is not used */

      /* This message is sent once at the uC startup and for power up. */
      /* setup i/o or do any other setup */

      if (!bcm2835_init()){
        printf("Failed bcm2835_init().\n");
        exit(1);
      }

      if (!bcm2835_spi_begin()){
        printf("Failed bcm2835_spi_begin().\n");
        exit(1);
      }

      bcm2835_gpio_fsel(GPIO_PIN_RST, BCM2835_GPIO_FSEL_OUTP);
      bcm2835_gpio_fsel(GPIO_PIN_CD, BCM2835_GPIO_FSEL_OUTP);
      bcm2835_gpio_fsel(GPIO_PIN_CS, BCM2835_GPIO_FSEL_OUTP);

      bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
      bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
      bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8);
      bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
      bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
      break;
    case UCG_COM_MSG_POWER_DOWN:
      /* "data" and "arg" are not used*/
      /* This message is sent for a power down request */
      bcm2835_spi_end();
      bcm2835_close();
      break;
    case UCG_COM_MSG_DELAY:
      /* "data" is not used */
      /* "arg" contains the number of microseconds for the delay */
      /* By receiving this message, the following code should delay by */
      /* "arg" microseconds. One microsecond is 0.000001 second */
      bcm2835_delayMicroseconds(arg);
      break;
    case UCG_COM_MSG_CHANGE_RESET_LINE:
      /* "data" is not used */
      /* "arg" = 1: set the reset output line to 1 */
      /* "arg" = 0: set the reset output line to 0 */
      bcm2835_gpio_write(GPIO_PIN_RST, arg);
      break;
    case UCG_COM_MSG_CHANGE_CD_LINE:
      /* "ucg->com_status"  bit 0 contains the old level for the CD line */
      /* "data" is not used */
      /* "arg" = 1: set the command/data (a0) output line to 1 */
      /* "arg" = 0: set the command/data (a0) output line to 0 */
      bcm2835_gpio_write(GPIO_PIN_CD, arg);
      break;
    case UCG_COM_MSG_CHANGE_CS_LINE:
      /* "ucg->com_status"  bit 1 contains the old level for the CS line */
      /* "data" is not used */
      /* "arg" = 1: set the chipselect output line to 1 */
      /* "arg" = 0: set the chipselect output line to 0 */
      bcm2835_gpio_write(GPIO_PIN_CS, arg);
      break;
    case UCG_COM_MSG_SEND_BYTE:
      /* "data" is not used */
      /* "arg" contains one byte, which should be sent to the display */
      /* The current status of the CD line is available */
      /* in bit 0 of u8g->com_status */
      spi_send(1, (unsigned char *)&arg);
      break;
    case UCG_COM_MSG_REPEAT_1_BYTE:
      /* "data[0]" contains one byte */
      /* repeat sending the byte in data[0] "arg" times */
      /* The current status of the CD line is available */
      /* in bit 0 of u8g->com_status */
      while( arg > 0 ) {
        spi_send(1, (unsigned char *)data);
        arg--;
      }
      break;
    case UCG_COM_MSG_REPEAT_2_BYTES:
      /* "data[0]" contains first byte */
      /* "data[1]" contains second byte */
      /* repeat sending the two bytes "arg" times */
      /* The current status of the CD line is available */
      /* in bit 0 of u8g->com_status */
      while( arg > 0 ) {
        spi_send(2, (unsigned char *)data);
        arg--;
      }
      break;
    case UCG_COM_MSG_REPEAT_3_BYTES:
      /* "data[0]" contains first byte */
      /* "data[1]" contains second byte */
      /* "data[2]" contains third byte */
      /* repeat sending the three bytes "arg" times */
      /* The current status of the CD line is available */
      /* in bit 0 of u8g->com_status */
      while( arg > 0 ) {
        spi_send(3, (unsigned char *)data);
        arg--;
      }
      break;
    case UCG_COM_MSG_SEND_STR:
      /* "data" is an array with "arg" bytes */
      /* send "arg" bytes to the display */
      spi_send(arg, (unsigned char *)data);
      break;
    case UCG_COM_MSG_SEND_CD_DATA_SEQUENCE:
      /* "data" is a pointer to two bytes, which contain the cd line */
      /* status and display data */
      /* "arg" contains the number of these two byte tuples which need to */
      /* be analysed and sent. Bellow is a example sequence */
      /* The content of bit 0 in u8g->com_status is undefined for this message */
      while(arg > 0) {
        if ( *data != 0 ) {
          /* set the data line directly, ignore the setting from UCG_CFG_CD */
          if ( *data == 1 ) {
            bcm2835_gpio_write(GPIO_PIN_CD, 0);
          } else {
            bcm2835_gpio_write(GPIO_PIN_CD, 1);
          }
        }
        data++;
        spi_send(1, (unsigned char *)data);
        data++;
        arg--;
      }
      break;
  }
  return 1;
}

void setup_ucg(){
  ucg_Init(
    &ucg,
    ucg_dev_st7735_18x128x160,
    ucg_ext_st7735_18,
    ucg_com_raspberry_pi_4wire_HW_SPI
  );
  ucg_ClearScreen(&ucg);
  ucg_SetRotate270(&ucg);
}

void setup_mlx90640(){
  MLX90640_SetDeviceMode(MLX_I2C_ADDR, 0);
  MLX90640_SetSubPageRepeat(MLX_I2C_ADDR, 0);

  switch(FPS){
    case 1:
      MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b001);
      break;
    case 2:
      MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b010);
      break;
    case 4:
      MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b011);
      break;
    case 8:
      MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b100);
      break;
    case 16:
      MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b101);
      break;
    case 32:
      MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b110);
      break;
    case 64:
      MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b111);
      break;
    default:
      printf("Unsupported framerate: %d", FPS);
      exit(1);
  }
  MLX90640_SetChessMode(MLX_I2C_ADDR);

  MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
  MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
}


void get_grad_value(double value, double min, double max, int *r, int *g, int *b){
  // Heatmap code borrowed from: http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
  /*
  const int NUM_COLORS = 7;
  static float color[NUM_COLORS][3] = {
    {0,0,0},
    {0,0,1},
    {0,1,0},
    {1,1,0},
    {1,0,0},
    {1,0,1},
    {1,1,1}
  };
  */
  const int NUM_COLORS = 5;
  static float color[NUM_COLORS][3] = {
    {0, 0, 0},
    {32.0/255.0, 0, 140./255.0},
    {204.0/255.0, 0, 119.0/255.0},
    {1, 215.0/255.0, 0},
    {1, 1, 1}
  };
  int idx1, idx2;
  float fractBetween = 0;
  float vrange = max-min;
  value -= min;
  value /= vrange;
  if(value <= 0){
    idx1=idx2=0;
  } else if(value >= 1) {
    idx1=idx2=NUM_COLORS-1;
  } else {
    value *= (NUM_COLORS-1);
    idx1 = floor(value);
    idx2 = idx1+1;
    fractBetween = value - float(idx1);
  }

  *r = (int)((((color[idx2][0] - color[idx1][0]) * fractBetween) + color[idx1][0]) * 255.0);
  *g = (int)((((color[idx2][1] - color[idx1][1]) * fractBetween) + color[idx1][1]) * 255.0);
  *b = (int)((((color[idx2][2] - color[idx1][2]) * fractBetween) + color[idx1][2]) * 255.0);
};


void draw_pixel(int x, int y, double temp, float vmin, float vmax){
  int ir, ig, ib;
  get_grad_value(temp, vmin, vmax, &ir, &ig, &ib);

  ucg_SetColor(&ucg, 0, ir, ig, ib);
  // ucg_DrawFrame(&ucg, x * IMAGE_SCALE + 1, y * IMAGE_SCALE + 1, IMAGE_SCALE - 1, IMAGE_SCALE - 1);
  ucg_DrawBox(&ucg, x * IMAGE_SCALE, y * IMAGE_SCALE, IMAGE_SCALE, IMAGE_SCALE);
  if(temp == vmin){
    ucg_SetColor(&ucg, 0, 255, 255, 255);
    ucg_DrawFrame(&ucg, x * IMAGE_SCALE, y * IMAGE_SCALE, IMAGE_SCALE, IMAGE_SCALE);
  }
  if(temp == vmax){
    ucg_SetColor(&ucg, 0, 0, 0, 0);
    ucg_DrawFrame(&ucg, x * IMAGE_SCALE, y * IMAGE_SCALE, IMAGE_SCALE, IMAGE_SCALE);
  }
}

int main(void) {
  setup_ucg();
  auto frame_time = std::chrono::microseconds(FRAME_TIME_MICROS + OFFSET_MICROS);

  setup_mlx90640();

  while (1){
    float emissivity = 1;
    uint16_t frame[834];
    static float mlx90640To[768];
    float eTa;
    auto start = std::chrono::system_clock::now();
    char * char_buff;
    ucg_int_t grad_x_start;
    ucg_int_t grad_x_end;

    // Fetch frame
    MLX90640_GetFrameData(MLX_I2C_ADDR, frame);
    MLX90640_InterpolateOutliers(frame, eeMLX90640);
    eTa = MLX90640_GetTa(frame, &mlx90640);
    MLX90640_CalculateTo(frame, &mlx90640, emissivity, eTa, mlx90640To);

    // Get max/min
    float max_val = -40;
    float min_val = 300;
    for(int y = 0; y < 24; y++){
      for(int x = 0; x < 32; x++){
        float val = mlx90640To[32 * (23-y) + x];
        if(val > max_val)
          max_val = val;
        if(val < min_val)
          min_val = val;
      }
    }

    // Draw image
    for(int y = 0; y < 24; y++){
      for(int x = 0; x < 32; x++){
        float val = mlx90640To[32 * (23-y) + x];
        draw_pixel(x, y, val, min_val, max_val);
      }
    }

    // Draw text
    ucg_SetColor(&ucg, 0, 0, 0, 0);
    ucg_SetFontMode(&ucg, UCG_FONT_MODE_TRANSPARENT);
    ucg_SetFont(&ucg, ucg_font_amstrad_cpc_8f);
    ucg_SetFontPosBaseline(&ucg);
    
    if(-1 == asprintf(&char_buff, "%.1fC", min_val)){
      printf("Failed to asprintf()\n");
      exit(1);
    }
    grad_x_start = ucg_GetStrWidth(&ucg, char_buff) + 1;
    ucg_SetColor(&ucg, 0, 0, 0, 0);
    ucg_DrawBox(&ucg, 0, 120, grad_x_start, 128);
    ucg_SetColor(&ucg, 0, 255, 255, 255);
    ucg_DrawString(&ucg, 0, 129, 0, char_buff);
    free(char_buff);

    if(-1 == asprintf(&char_buff, "%.1fC", max_val)){
      printf("Failed to asprintf()\n");
      exit(1);
    }
    grad_x_end = ucg_GetWidth(&ucg) - ucg_GetStrWidth(&ucg, char_buff) - 1;
    ucg_SetColor(&ucg, 0, 0, 0, 0);
    ucg_DrawBox(&ucg, grad_x_end, 120, 160, 128);
    ucg_SetColor(&ucg, 0, 255, 255, 255);
    ucg_DrawString(&ucg, grad_x_end + 1, 129, 0, char_buff);
    free(char_buff);

    // Draw scale
    for(int x=grad_x_start ; x <= grad_x_end ; x++){
      int r, g, b;
      get_grad_value(x, grad_x_start, grad_x_end, &r, &g, &b);
      ucg_SetColor(&ucg, 0, r, g, b);
      ucg_DrawLine(&ucg, x, 121, x, 127);
    }

    // Sleep before next frame
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::this_thread::sleep_for(std::chrono::microseconds(frame_time - elapsed));
  }
}
