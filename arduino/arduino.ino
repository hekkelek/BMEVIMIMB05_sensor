#include <M5StickCPlus.h>
#include <Wire.h>
#include "bmm150.h"

#define I2C_ADDRESS (0x10)
#define SPI_CS_PIN 12
#define USEIIC 1

uint8_t dev_addr;
/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations powermode, odr and interrupt mapping.
 *
 *  @param[in] dev       : Structure instance of bmm150_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_config(struct bmm150_dev *dev);

/*!
 *  @brief This internal API is used to get gyro data.
 *
 *  @param[in] dev       : Structure instance of bmm150_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_data(struct bmm150_dev *dev);

/******************************************************************************/
/*!            Functions                                        */

void spi_bmm150_cs_high(void) {
  digitalWrite(SPI_CS_PIN, 1);
}
void spi_bmm150_cs_low(void) {
  digitalWrite(SPI_CS_PIN, 0);
}
void bmm150_user_delay_us(uint32_t period_us, void *intf_ptr) {
  /* Wait for a period amount of microseconds. */
  delayMicroseconds(period_us);
}

/*!
 * @brief This function is for writing the sensor's registers through I2C bus.
 */
int8_t bmm150_user_i2c_reg_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {

  /* Write to registers using I2C. Return 0 for a successful execution. */
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg_addr);
  for (uint8_t i = 0; i < length; i++) {
    Wire.write(reg_data[i]);
  }
  Wire.endTransmission();
  return 0;
}

/*!
 * @brief This function is for reading the sensor's registers through I2C bus.
 */
int8_t bmm150_user_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {

  /* Read from registers using I2C. Return 0 for a successful execution. */
  int i = 0;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg_addr);
  if (Wire.endTransmission() != 0) {
    return -1;
  }
  Wire.requestFrom((uint8_t)I2C_ADDRESS, (uint8_t)length);
  while (Wire.available()) {
    reg_data[i++] = Wire.read();
  }
  return 0;
}

/*!
 * @brief This function is for writing the sensor's registers through SPI bus.
 */
int8_t bmm150_user_spi_reg_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {

  /* Write to registers using SPI. Return 0 for a successful execution. */
  int8_t rslt = 0;

  spi_bmm150_cs_high();
  spi_bmm150_cs_low();
  //SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(reg_addr & 0x7f);
  for(uint8_t i = 0; i < length; i++){
    SPI.transfer(reg_data[i]);
  }
  //SPI.endTransaction();
  spi_bmm150_cs_high();
  return rslt;
}

/*!
 * @brief This function is for reading the sensor's registers through SPI bus.
 */
int8_t bmm150_user_spi_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {

  /* Read from registers using SPI. Return 0 for a successful execution. */
  int8_t rslt = 0;
  spi_bmm150_cs_high();
  spi_bmm150_cs_low();
  //SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(reg_addr | 0x80);
  delay(10);
  for (uint8_t i = 0; i < length; i++) {
    reg_data[i] = SPI.transfer(0xff);
  }
  //SPI.endTransaction();
  spi_bmm150_cs_high();
  return rslt;
}

/*!
 *  @brief This function is to select the interface between SPI and I2C.
 */
int8_t bmm150_interface_selection(struct bmm150_dev *dev) {
  int8_t rslt = BMM150_OK;

  if (dev != NULL) {
    /* Select the interface for execution
         * For I2C : BMM150_I2C_INTF
         * For SPI : BMM150_SPI_INTF
         */
    if(USEIIC){
      dev->intf = BMM150_I2C_INTF;
    }else{
      dev->intf = BMM150_SPI_INTF;
    }
    
    /* Bus configuration : I2C */
    if (dev->intf == BMM150_I2C_INTF) {
      Serial.print("I2C Interface \n");
      Wire.begin(0, 26);
      /* To initialize the user I2C function */
      //bmm150_user_i2c_init();

      dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
      dev->read = bmm150_user_i2c_reg_read;
      dev->write = bmm150_user_i2c_reg_write;
    }
    /* Bus configuration : SPI */
    else if (dev->intf == BMM150_SPI_INTF) {
      pinMode(SPI_CS_PIN,OUTPUT);
      Serial.print("SPI Interface \n");
      SPI.begin(9,10,11,12);
      spi_bmm150_cs_low();
      /* To initialize the user SPI function */
      //bmm150_user_spi_init();

      dev_addr = 0;
      dev->read = bmm150_user_spi_reg_read;
      dev->write = bmm150_user_spi_reg_write;
    }

    /* Assign device address to interface pointer */
    dev->intf_ptr = &dev_addr;

    /* Configure delay in microseconds */
    dev->delay_us = bmm150_user_delay_us;
  } else {
    rslt = BMM150_E_NULL_PTR;
  }

  return rslt;
}

/*!
 * @brief This internal API prints the execution status
 */
void bmm150_error_codes_print_result(const char api_name[], int8_t rslt) {
  //Serial.println(api_name);
  if (rslt != BMM150_OK) {
    //Serial.print(api_name);

    switch (rslt) {
      case BMM150_E_NULL_PTR:
        Serial.print("Error: Null pointer error.");Serial.println(rslt);
        Serial.println("It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
        break;

      case BMM150_E_COM_FAIL:
        Serial.print("Error: Communication failure error.");Serial.println(rslt);
        Serial.println("It occurs due to read/write operation failure and also due to power failure during communication\r\n");
        break;

      case BMM150_E_DEV_NOT_FOUND:
        Serial.println("Error: Device not found error. It occurs when the device chip id is incorrectly read\r\n");Serial.println(rslt);
        break;

      case BMM150_E_INVALID_CONFIG:
        Serial.print("Error: Invalid sensor configuration.");Serial.println(rslt);
        Serial.println(" It occurs when there is a mismatch in the requested feature with the available one\r\n");
        break;

      default:
        Serial.print("Error: Unknown error code\r\n");Serial.println(rslt);
        break;
    }
  }
}

// Globals
int8_t rslt;
struct bmm150_dev dev;

int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;
int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  M5.Imu.Init();
  //Wire.begin(0, 26);
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(40, 65, 2);
  M5.Lcd.println("BMEVIMIMB05 sensor");
  pinMode(M5_BUTTON_HOME, INPUT);

  Serial.begin(115200);

  rslt = bmm150_interface_selection(&dev);
  bmm150_error_codes_print_result("bmm150_interface_selection", rslt);

  if (rslt == BMM150_OK) {
    rslt = bmm150_init(&dev);
    bmm150_error_codes_print_result("bmm150_init", rslt);
    Serial.println(dev.chip_id);
    if (rslt == BMM150_OK) {
      rslt = set_config(&dev);
      bmm150_error_codes_print_result("set_config", rslt);

      if (rslt == BMM150_OK) {
        rslt = get_data(&dev);
        bmm150_error_codes_print_result("get_data", rslt);
      }
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}


/*!
 *  @brief This internal API is used to set configurations like powermode, odr and interrupt mapping.
 */
static int8_t set_config(struct bmm150_dev *dev) {
  /* Status of api are returned to this variable. */
  int8_t rslt;

  struct bmm150_settings settings;

  /* Set powermode as normal mode */
  settings.pwr_mode = BMM150_POWERMODE_NORMAL;
  rslt = bmm150_set_op_mode(&settings, dev);
  bmm150_error_codes_print_result("bmm150_set_op_mode", rslt);

  if (rslt == BMM150_OK) {
    /* Setting the preset mode as high accuracy mode
         * i.e. data rate = 20Hz, XY-rep = 1, Z-rep = 2
         */
    settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
    rslt = bmm150_set_presetmode(&settings, dev);
    bmm150_error_codes_print_result("bmm150_set_presetmode", rslt);

    if (rslt == BMM150_OK) {
      /* Map the data interrupt pin */
      settings.int_settings.drdy_pin_en = 0x01;
      rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev);
      bmm150_error_codes_print_result("bmm150_set_sensor_settings", rslt);
    }
  }

  return rslt;
}

/*!
 *  @brief This internal API is used to get gyro data.
 */
static int8_t get_data(struct bmm150_dev *dev) {
  /* Status of api are returned to this variable. */
  int8_t rslt;
  uint8_t au8Packet[ 6*2 + 3*2 + 2 ];
  //int8_t idx;

  struct bmm150_mag_data mag_data;

  /* Reading the mag data */
  while (1) {
    /* Get the interrupt status */
    rslt = bmm150_get_interrupt_status(dev);

    if (dev->int_status & BMM150_INT_ASSERTED_DRDY) {
      //Serial.println("Data interrupt occurred");
      /* Read mag data */
      rslt = bmm150_read_mag_data(&mag_data, dev);

      /* Unit for magnetometer data is microtesla(uT) */
      //Serial.print("X:");Serial.print(mag_data.x);Serial.print(" uT,Y:");Serial.print(mag_data.y);Serial.print(" uT,Z:");Serial.println(mag_data.z);

      //M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
      //M5.Imu.getAccelData(&accX, &accY, &accZ);
      M5.Imu.getGyroAdc(&gyroX, &gyroY, &gyroZ);
      M5.Imu.getAccelAdc(&accX, &accY, &accZ);
      //Serial.print("Xa:");Serial.print(accX);Serial.print(", Ya:");Serial.print(accY);Serial.print(", Za:");Serial.println(accZ);
      //Serial.print("Xg:");Serial.print(gyroX);Serial.print(", Yg:");Serial.print(gyroY);Serial.print(", Zg:");Serial.println(gyroZ);

      au8Packet[  0 ] = 0x55;
      au8Packet[  1 ] = ((uint8_t*)&accX)[ 0 ];
      au8Packet[  2 ] = ((uint8_t*)&accX)[ 1 ];

      au8Packet[  3 ] = ((uint8_t*)&accY)[ 0 ];
      au8Packet[  4 ] = ((uint8_t*)&accY)[ 1 ];

      au8Packet[  5 ] = ((uint8_t*)&accZ)[ 0 ];
      au8Packet[  6 ] = ((uint8_t*)&accZ)[ 1 ];

      au8Packet[  7 ] = ((uint8_t*)&gyroX)[ 0 ];
      au8Packet[  8 ] = ((uint8_t*)&gyroX)[ 1 ];

      au8Packet[  9 ] = ((uint8_t*)&gyroY)[ 0 ];
      au8Packet[ 10 ] = ((uint8_t*)&gyroY)[ 1 ];

      au8Packet[ 11 ] = ((uint8_t*)&gyroZ)[ 0 ];
      au8Packet[ 12 ] = ((uint8_t*)&gyroZ)[ 1 ];

      au8Packet[ 13 ] = ((uint8_t*)&mag_data.x)[ 0 ];
      au8Packet[ 14 ] = ((uint8_t*)&mag_data.x)[ 1 ];

      au8Packet[ 15 ] = ((uint8_t*)&mag_data.y)[ 0 ];
      au8Packet[ 16 ] = ((uint8_t*)&mag_data.y)[ 1 ];

      au8Packet[ 17 ] = ((uint8_t*)&mag_data.z)[ 0 ];
      au8Packet[ 18 ] = ((uint8_t*)&mag_data.z)[ 1 ];

      au8Packet[ 19 ] = 0xAA;

      Serial.write( au8Packet, sizeof(au8Packet) );
    }
    //delay(500);
    //break;
  }

  return rslt;
}
