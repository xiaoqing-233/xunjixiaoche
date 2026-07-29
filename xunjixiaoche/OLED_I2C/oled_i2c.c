#include "oled_i2c.h"

#include <string.h>

#include "oled_i2c_font.h"
#include "soft_i2c.h"

#define OLED_I2C_PAGE_COUNT (OLED_I2C_HEIGHT / 8U)
#define OLED_I2C_COMMAND_CONTROL 0x00U
#define OLED_I2C_DATA_CONTROL 0x40U

static uint8_t oled_i2c_framebuffer[OLED_I2C_WIDTH * OLED_I2C_PAGE_COUNT];
static SoftI2C_Bus oled_i2c_bus = {
  .SDA_Port = GPIOB,
  .SDA_Pin = GPIO_PIN_8,
  .SCL_Port = GPIOB,
  .SCL_Pin = GPIO_PIN_9
};
static uint8_t oled_i2c_address = OLED_I2C_ADDRESS;

static HAL_StatusTypeDef oled_i2c_write(uint8_t control, const uint8_t *data, uint16_t size)
{
  uint16_t index;
  HAL_StatusTypeDef status = HAL_OK;

  if ((data == NULL) || (size == 0U) || (size > OLED_I2C_WIDTH))
  {
    return HAL_ERROR;
  }

  Software_I2C_Start(&oled_i2c_bus);
  Software_I2C_WriteByte(&oled_i2c_bus, (uint8_t)(oled_i2c_address << 1U));
  if (Software_I2C_WaitACK(&oled_i2c_bus) != HAL_OK)
  {
    status = HAL_ERROR;
    goto stop;
  }
  Software_I2C_WriteByte(&oled_i2c_bus, control);
  if (Software_I2C_WaitACK(&oled_i2c_bus) != HAL_OK)
  {
    status = HAL_ERROR;
    goto stop;
  }
  for (index = 0U; index < size; ++index)
  {
    Software_I2C_WriteByte(&oled_i2c_bus, data[index]);
    if (Software_I2C_WaitACK(&oled_i2c_bus) != HAL_OK)
    {
      status = HAL_ERROR;
      goto stop;
    }
  }

stop:
  Software_I2C_Stop(&oled_i2c_bus);
  return status;
}

static HAL_StatusTypeDef oled_i2c_probe(uint8_t address)
{
  HAL_StatusTypeDef status;

  Software_I2C_Start(&oled_i2c_bus);
  Software_I2C_WriteByte(&oled_i2c_bus, (uint8_t)(address << 1U));
  status = Software_I2C_WaitACK(&oled_i2c_bus);
  Software_I2C_Stop(&oled_i2c_bus);
  return status;
}

static HAL_StatusTypeDef oled_i2c_command(const uint8_t *commands, uint16_t size)
{
  return oled_i2c_write(OLED_I2C_COMMAND_CONTROL, commands, size);
}

static HAL_StatusTypeDef oled_i2c_set_page(uint8_t page)
{
  const uint8_t commands[] = { (uint8_t)(0xB0U + page), 0x00U, 0x10U };
  return oled_i2c_command(commands, sizeof(commands));
}

static void oled_i2c_draw_column(uint8_t x, uint8_t y, uint8_t data, uint8_t color)
{
  uint8_t bit;

  for (bit = 0U; bit < 8U; ++bit)
  {
    oled_i2c_draw_point(x, (uint8_t)(y + bit), (data & (1U << bit)) ? color : (uint8_t)!color);
  }
}

static uint32_t oled_i2c_pow10(uint8_t exponent)
{
  uint32_t value = 1U;

  while (exponent-- > 0U)
  {
    value *= 10U;
  }
  return value;
}

HAL_StatusTypeDef oled_i2c_init(void)
{
  static const uint8_t initialization[] = {
    0xAEU, 0x20U, 0x02U, 0x40U, 0x81U, 0xCFU, 0xA1U, 0xC8U,
    0xA6U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0xD5U, 0x80U, 0xD9U,
    0xF1U, 0xDAU, 0x12U, 0xDBU, 0x40U, 0x8DU, 0x14U, 0xA4U, 0xAFU
  };
  HAL_StatusTypeDef status;

  Software_I2C_Init(&oled_i2c_bus);
  oled_i2c_address = OLED_I2C_ADDRESS;
  status = oled_i2c_probe(oled_i2c_address);
  if (status != HAL_OK)
  {
    oled_i2c_address = OLED_I2C_ALTERNATE_ADDRESS;
    status = oled_i2c_probe(oled_i2c_address);
  }
  if (status != HAL_OK)
  {
    return status;
  }

  status = oled_i2c_command(initialization, sizeof(initialization));
  if (status != HAL_OK)
  {
    return status;
  }

  return oled_i2c_clear();
}

HAL_StatusTypeDef oled_i2c_clear(void)
{
  oled_i2c_clear_buffer();
  return oled_i2c_refresh();
}

void oled_i2c_clear_buffer(void)
{
  memset(oled_i2c_framebuffer, 0, sizeof(oled_i2c_framebuffer));
}

HAL_StatusTypeDef oled_i2c_refresh(void)
{
  HAL_StatusTypeDef status;
  uint8_t page;

  for (page = 0U; page < OLED_I2C_PAGE_COUNT; ++page)
  {
    status = oled_i2c_set_page(page);
    if (status != HAL_OK)
    {
      return status;
    }
    status = oled_i2c_write(OLED_I2C_DATA_CONTROL, &oled_i2c_framebuffer[page * OLED_I2C_WIDTH], OLED_I2C_WIDTH);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef oled_i2c_display_on(void)
{
  const uint8_t command = 0xAFU;
  return oled_i2c_command(&command, 1U);
}

HAL_StatusTypeDef oled_i2c_display_off(void)
{
  const uint8_t command = 0xAEU;
  return oled_i2c_command(&command, 1U);
}

HAL_StatusTypeDef oled_i2c_set_inverse(uint8_t enabled)
{
  const uint8_t command = enabled ? 0xA7U : 0xA6U;
  return oled_i2c_command(&command, 1U);
}

HAL_StatusTypeDef oled_i2c_set_rotation(uint8_t rotated_180)
{
  const uint8_t commands[] = { rotated_180 ? 0xA0U : 0xA1U, rotated_180 ? 0xC0U : 0xC8U };
  return oled_i2c_command(commands, sizeof(commands));
}

void oled_i2c_draw_point(uint8_t x, uint8_t y, uint8_t color)
{
  uint16_t index;
  uint8_t mask;

  if ((x >= OLED_I2C_WIDTH) || (y >= OLED_I2C_HEIGHT))
  {
    return;
  }

  index = (uint16_t)(y >> 3U) * OLED_I2C_WIDTH + x;
  mask = (uint8_t)(1U << (y & 0x07U));
  if (color != 0U)
  {
    oled_i2c_framebuffer[index] |= mask;
  }
  else
  {
    oled_i2c_framebuffer[index] &= (uint8_t)~mask;
  }
}

void oled_i2c_draw_char(uint8_t x, uint8_t y, char character, uint8_t size, uint8_t color)
{
  uint8_t column;
  uint8_t index;

  if ((character < ' ') || (character > '~'))
  {
    return;
  }

  index = (uint8_t)(character - ' ');
  if (size <= 8U)
  {
    for (column = 0U; column < 6U; ++column)
    {
      oled_i2c_draw_column((uint8_t)(x + column), y, oled_i2c_font_6x8[index][column], color);
    }
  }
  else
  {
    for (column = 0U; column < 8U; ++column)
    {
      oled_i2c_draw_column((uint8_t)(x + column), y, oled_i2c_font_8x16[index][column], color);
      oled_i2c_draw_column((uint8_t)(x + column), (uint8_t)(y + 8U), oled_i2c_font_8x16[index][column + 8U], color);
    }
  }
}

void oled_i2c_draw_string(uint8_t x, uint8_t y, const char *text, uint8_t size, uint8_t color)
{
  const uint8_t character_width = (size <= 8U) ? 6U : 8U;
  const uint8_t line_height = (size <= 8U) ? 8U : 16U;

  if (text == NULL)
  {
    return;
  }

  while ((*text >= ' ') && (*text <= '~'))
  {
    if ((uint16_t)x + character_width > OLED_I2C_WIDTH)
    {
      x = 0U;
      y = (uint8_t)(y + line_height);
    }
    if (y >= OLED_I2C_HEIGHT)
    {
      return;
    }
    oled_i2c_draw_char(x, y, *text++, size, color);
    x = (uint8_t)(x + character_width);
  }
}

void oled_i2c_draw_uint(uint8_t x, uint8_t y, uint32_t value, uint8_t digits, uint8_t size, uint8_t color)
{
  uint8_t position;
  uint8_t character_width = (size <= 8U) ? 6U : 8U;
  uint32_t divisor;

  if (digits == 0U)
  {
    return;
  }
  if (digits > 10U)
  {
    digits = 10U;
  }

  divisor = oled_i2c_pow10((uint8_t)(digits - 1U));
  for (position = 0U; position < digits; ++position)
  {
    oled_i2c_draw_char((uint8_t)(x + character_width * position), y,
                       (char)('0' + ((value / divisor) % 10U)), size, color);
    if (divisor > 1U)
    {
      divisor /= 10U;
    }
  }
}

void oled_i2c_draw_int(uint8_t x, uint8_t y, int32_t value, uint8_t digits, uint8_t size, uint8_t color)
{
  uint32_t magnitude;
  const uint8_t character_width = (size <= 8U) ? 6U : 8U;

  if (value < 0)
  {
    oled_i2c_draw_char(x, y, '-', size, color);
    x = (uint8_t)(x + character_width);
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)value;
  }

  oled_i2c_draw_uint(x, y, magnitude, digits, size, color);
}
