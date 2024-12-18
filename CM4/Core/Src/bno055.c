/*
 * bno055_1.c
 *
 *  Created on: Dec 14, 2023
 *      Author: remim
 */
#include <bno055.h>
#include <string.h>

I2C_HandleTypeDef *_bno055_i2c_port;

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1<<14);    // 2^14

// Imposta la pagina di registro del sensore BNO055
void bno055_setPage(uint8_t page) { bno055_writeData(BNO055_PAGE_ID, page); }

// Ottiene la modalità operativa del sensore BNO055
bno055_opmode_t bno055_getOperationMode() {
  bno055_opmode_t mode;
  bno055_readData(BNO055_OPR_MODE, &mode, 1);
  return mode;
}

// Imposta la modalità operativa del sensore BNO055 e attende un ritardo in base alla modalità
void bno055_setOperationMode(bno055_opmode_t mode) {
  bno055_writeData(BNO055_OPR_MODE, mode);
  if (mode == BNO055_OPERATION_MODE_CONFIG) {
    bno055_delay(100);
  } else {
    bno055_delay(80);
  }
}

// NON FUSION MODE

// Imposta la modalità operativa del sensore BNO055 su configurazione
void bno055_setOperationModeConfig() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeACCONLY(){
	bno055_setOperationMode(BNO055_OPERATION_MODE_ACCONLY);
}

void bno055_setOperationModeMAGONLY(){
	bno055_setOperationMode(BNO055_OPERATION_MODE_MAGONLY);
}

void bno055_setOperationModeGYRONLY() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_GYRONLY);
}

void bno055_setOperationModeACCMAG(){
	bno055_setOperationMode(BNO055_OPERATION_MODE_ACCMAG);
}

void bno055_setOperationModeACCGYRO(){
	bno055_setOperationMode(BNO055_OPERATION_MODE_ACCGYRO);
}

void bno055_setOperationModeMAGGYRO(){
	bno055_setOperationMode(BNO055_OPERATION_MODE_MAGGYRO);
}

void bno055_setOperationModeAMG(){
	bno055_setOperationMode(BNO055_OPERATION_MODE_AMG);
}

// FUSION MODE

void bno055_setOperationModeIMU(){
	bno055_setOperationMode(BNO055_OPERATION_MODE_IMU);
}

void bno055_setOperationModeCOMPASS() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_COMPASS);
}

void bno055_setOperationModeM4G(){
	bno055_setOperationMode(BNO055_OPERATION_MODE_M4G);
}

void bno055_setOperationModeNDOF_FMC_OFF(){
	bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF_FMC_OFF);
}

// Imposta la modalità operativa del sensore BNO055 su NDOF (Nine Degrees of Freedom)
void bno055_setOperationModeNDOF() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

// Abilita o disabilita l'uso di un cristallo esterno per il sensore BNO055
void bno055_setExternalCrystalUse(bool state) {
  bno055_setPage(0);
  uint8_t tmp = 0;
  bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
  tmp |= (state == true) ? 0x80 : 0x0;
  bno055_writeData(BNO055_SYS_TRIGGER, tmp);
  bno055_delay(700);
}

// Abilita l'uso di un cristallo esterno per il sensore BNO055
void bno055_enableExternalCrystal() { bno055_setExternalCrystalUse(true); }
// Disabilita l'uso di un cristallo esterno per il sensore BNO055
void bno055_disableExternalCrystal() { bno055_setExternalCrystalUse(false); }

// Esegue una procedura di reset per il sensore BNO055
void bno055_reset() {
  bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
  bno055_delay(700);
}

// Ottiene la temperatura dal sensore BNO055
int8_t bno055_getTemp() {
  bno055_setPage(0);
  uint8_t t;
  bno055_readData(BNO055_TEMP, &t, 1);
  return t;
}

// Inizializza il sensore BNO055 eseguendo una procedura di reset e configurazione iniziale
void bno055_setup() {
  bno055_reset();

  uint8_t id = 0;
  bno055_readData(BNO055_CHIP_ID, &id, 1);
  if (id != BNO055_ID) {
    printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
  }
  bno055_setPage(0);
  bno055_writeData(BNO055_SYS_TRIGGER, 0x0);

  // Seleziona la modalità di configurazione del BNO055
  bno055_setOperationModeConfig();
  bno055_delay(10);
}


// Ottiene la revisione del software del sensore BNO055
int16_t bno055_getSWRevision() {
  bno055_setPage(0);
  uint8_t buffer[2];
  bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

// Ottiene la revisione del bootloader del sensore BNO055
uint8_t bno055_getBootloaderRevision() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

// Ottiene lo stato del sistema dal sensore BNO055
uint8_t bno055_getSystemStatus() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

// Ottiene i risultati del test di autodiagnosi iniziale dal sensore BNO055
bno055_self_test_result_t bno055_getSelfTestResult_start() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_self_test_result_t res = {
      .mcuState = 0,
	  .gyrState = 0,
	  .magState = 0,
	  .accState = 0};
  bno055_readData(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

 //Ottiene i risultati del test di autodiagnosi dal sensore BNO055
bno055_self_test_result_t bno055_getSelfTestResult() {
  bno055_setPage(0);
  //Legge il valore attuale del registro SYS_TRIGGER
  uint8_t sys_trigger_value;
  bno055_readData(BNO055_SYS_TRIGGER, &sys_trigger_value, 1);
  // Stampa il valore del bit meno significativo prima della scrittura
  printf("Bit meno significativo prima della scrittura: %d\r\n", sys_trigger_value & 0x01);
  // Imposta il bit del Self-Test
  sys_trigger_value |= 0x01; // Bit 0: Self-Test
  // Stampa il valore del bit meno significativo dopo la scrittura
  printf("Bit meno significativo dopo la scrittura: %d\r\n", sys_trigger_value & 0x01);
  // Scrive il valore modificato nel registro SYS_TRIGGER
  bno055_writeData(BNO055_SYS_TRIGGER, sys_trigger_value);

  bno055_self_test_result_t res = bno055_getSelfTestResult_start();

  return res;
}

// Ottiene il codice di errore del sistema dal sensore BNO055
uint8_t bno055_getSystemError() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

// Ottiene lo stato di calibrazione del sensore BNO055
bno055_calibration_state_t bno055_getCalibrationState() {
  bno055_setPage(0);
  bno055_calibration_state_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
  uint8_t calState = 0;
  bno055_readData(BNO055_CALIB_STAT, &calState, 1);
  cal.sys = (calState >> 6) & 0x03;
  cal.gyro = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag = calState & 0x03;
  return cal;
}

// Ottiene i dati di calibrazione del sensore BNO055
bno055_calibration_data_t bno055_getCalibrationData() {
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assume un processore little-endian
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);

  bno055_setOperationMode(operationMode);

  return calData;
}

// Imposta i dati di calibrazione per il sensore BNO055
void bno055_setCalibrationData(bno055_calibration_data_t calData) {
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  // Assume un processore little-endian
  memcpy(buffer, &calData.offset.accel, 6);
  memcpy(buffer + 6, &calData.offset.mag, 6);
  memcpy(buffer + 12, &calData.offset.gyro, 6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag, 2);

  for (uint8_t i=0; i < 22; i++) {
    bno055_writeData(BNO055_ACC_OFFSET_X_LSB+i, buffer[i]);
  }

  bno055_setOperationMode(operationMode);
}

//Ottiene un vettore specifico dal sensore BNO055 con la possibilità di specificare la scala del vettore
bno055_vector_t bno055_getVector(uint8_t vec) {
  bno055_setPage(0);
  uint8_t buffer[8];    // I quaternioni richiedono 8 byte

  if (vec == BNO055_VECTOR_QUATERNION)
    bno055_readData(vec, buffer, 8);
  else
    bno055_readData(vec, buffer, 6);

  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER) {
    scale = magScale;
  } else if (vec == BNO055_VECTOR_ACCELEROMETER ||
           vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
    scale = accelScale;
  } else if (vec == BNO055_VECTOR_GYROSCOPE) {
    scale = angularRateScale;
  } else if (vec == BNO055_VECTOR_EULER) {
    scale = eulerScale;
  } else if (vec == BNO055_VECTOR_QUATERNION) {
    scale = quaScale;
  }

  bno055_vector_t xyz = {.w = 0, .x = 0, .y = 0, .z = 0};
  if (vec == BNO055_VECTOR_QUATERNION) {
    xyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    xyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
  } else {
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
  }

  return xyz;
}

// Ottiene un vettore accelerazione dal sensore BNO055
bno055_vector_t bno055_getVectorAccelerometer() {
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
// Ottiene un vettore magnetico dal sensore BNO055
bno055_vector_t bno055_getVectorMagnetometer() {
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
// Ottiene un vettore giroscopico dal sensore BNO055
bno055_vector_t bno055_getVectorGyroscope() {
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
// Ottiene un vettore di angoli di Eulero dal sensore BNO055
bno055_vector_t bno055_getVectorEuler() {
  return bno055_getVector(BNO055_VECTOR_EULER);
}
// Ottiene un vettore di accelerazione lineare dal sensore BNO055
bno055_vector_t bno055_getVectorLinearAccel() {
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
// Ottiene un vettore di gravità dal sensore BNO055
bno055_vector_t bno055_getVectorGravity() {
  return bno055_getVector(BNO055_VECTOR_GRAVITY);
}
// Ottiene un vettore quaternion dal sensore BNO055
bno055_vector_t bno055_getVectorQuaternion() {
  return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

// Imposta la mappatura degli assi per il sensore BNO055
void bno055_setAxisMap(bno055_axis_map_t axis) {
  uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
  uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
  bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
  bno055_writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
}

void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device) {
  _bno055_i2c_port = hi2c_device;
}

void bno055_delay(int time) {
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}

void bno055_writeData(uint8_t reg, uint8_t data) {
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  status = HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1,
                                   txdata, sizeof(txdata), 10);

  if (status == HAL_OK) {
    return;
  }

  if (status == HAL_ERROR) {
    printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
  } else if (status == HAL_TIMEOUT) {
    printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
  } else if (status == HAL_BUSY) {
    printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
  } else {
    printf("Unknown status data %d", status);
  }

  uint32_t error = HAL_I2C_GetError(_bno055_i2c_port);
  if (error == HAL_I2C_ERROR_NONE) {
    return;
  } else if (error == HAL_I2C_ERROR_BERR) {
    printf("HAL_I2C_ERROR_BERR\r\n");
  } else if (error == HAL_I2C_ERROR_ARLO) {
    printf("HAL_I2C_ERROR_ARLO\r\n");
  } else if (error == HAL_I2C_ERROR_AF) {
    printf("HAL_I2C_ERROR_AF\r\n");
  } else if (error == HAL_I2C_ERROR_OVR) {
    printf("HAL_I2C_ERROR_OVR\r\n");
  } else if (error == HAL_I2C_ERROR_DMA) {
    printf("HAL_I2C_ERROR_DMA\r\n");
  } else if (error == HAL_I2C_ERROR_TIMEOUT) {
    printf("HAL_I2C_ERROR_TIMEOUT\r\n");
  }

  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(_bno055_i2c_port);
  if (state == HAL_I2C_STATE_RESET) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_READY) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_BUSY) {
    printf("HAL_I2C_STATE_BUSY\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX) {
    printf("HAL_I2C_STATE_BUSY_TX\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX) {
    printf("HAL_I2C_STATE_BUSY_RX\r\n");
  } else if (state == HAL_I2C_STATE_LISTEN) {
    printf("HAL_I2C_STATE_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_ABORT) {
    printf("HAL_I2C_STATE_ABORT\r\n");
//  } else if (state == HAL_I2C_STATE_TIMEOUT) {
//    printf("HAL_I2C_STATE_TIMEOUT\r\n");
//  } else if (state == HAL_I2C_STATE_ERROR) {
//    printf("HAL_I2C_STATE_ERROR\r\n");
}
  // while (HAL_I2C_GetState(_bno055_i2c_port) != HAL_I2C_STATE_READY) {}
  // return;
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1, &reg, 1,
                          100);
	ret = HAL_I2C_Master_Receive(_bno055_i2c_port, BNO055_I2C_ADDR << 1, data, len,
                         100);
	  if (ret == HAL_OK) {
	    return;
	  }

	  if (ret == HAL_ERROR) {
	    printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
	  } else if (ret == HAL_TIMEOUT) {
	    printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
	  } else if (ret == HAL_BUSY) {
	    printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
	  } else {
	    printf("Unknown status data %d", ret);
	  }
  // HAL_I2C_Mem_Read(_bno055_i2c_port, BNO055_I2C_ADDR_LO<<1, reg,
  // I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

float calculateHeading(float magX, float magY)
{
	float heading = atan2(magY, magX) * 180 / M_PI;

	if (heading < 0)
    {
		heading += 360;
    }

  return heading;
}

void bno055_configureAccelerometer(uint8_t gRange, float bandwidth, uint8_t operationMode) {
	bno055_setPage(1); //il registro si trova in page 1
    uint8_t accConfigReg = 0;

    if (gRange == 2) {
        accConfigReg |= 0 ;  // [ACC_Config]: xxxxxx00b
    } else if (gRange == 4) {
        accConfigReg |= 1 ;  // [ACC_Config]: xxxxxx01b
    } else if (gRange == 8) {
        accConfigReg |= 2 ;  // [ACC_Config]: xxxxxx10b
    } else if (gRange == 16) {
        accConfigReg |= 3 ;  // [ACC_Config]: xxxxxx11b
    } else {
        printf("Errore: G Range acc non valido\r\n");
        return;
    }

    if (bandwidth == 7.81) {
        accConfigReg |= (0 << 2);  // [ACC_Config]: xxx000xxb
    } else if (bandwidth == 15.63) {
        accConfigReg |= (1 << 2);  // [ACC_Config]: xxx001xxb
    } else if (bandwidth == 31.25) {
        accConfigReg |= (2 << 2);  // [ACC_Config]: xxx010xxb
    } else if (bandwidth == 62.5) {
        accConfigReg |= (3 << 2);  // [ACC_Config]: xxx011xxb
    } else if (bandwidth == 125) {
        accConfigReg |= (4 << 2);  // [ACC_Config]: xxx100xxb
    } else if (bandwidth == 250) {
        accConfigReg |= (5 << 2);  // [ACC_Config]: xxx101xxb
    } else if (bandwidth == 500) {
        accConfigReg |= (6 << 2);  // [ACC_Config]: xxx110xxb
    } else if (bandwidth == 1000) {
        accConfigReg |= (7 << 2);  // [ACC_Config]: xxx111xxb
    } else {
        printf("Errore: Bandwidth acc non valido\r\n");
        return;
    }

    if (operationMode == NORMAL) {
        accConfigReg |= 0;  // [ACC_Config]: 000xxxxxb
    } else if (operationMode == SUSPEND) {
        accConfigReg |= (1 << 5);  // [ACC_Config]: 001xxxxxb
    } else if (operationMode == LOW_POWER_1) {
        accConfigReg |= (2 << 5);  // [ACC_Config]: 010xxxxxb
    } else if (operationMode == STANDBY) {
        accConfigReg |= (3 << 5);  // [ACC_Config]: 011xxxxxb
    } else if (operationMode == LOW_POWER_2) {
        accConfigReg |= (4 << 5);  // [ACC_Config]: 100xxxxxb
    } else if (operationMode == DEEP_SUSPEND) {
        accConfigReg |= (5 << 5);  // [ACC_Config]: 101xxxxxb
    } else {
        printf("Errore: Modalità di funzionamento acc non valida\r\n");
        return;
    }

    bno055_setOperationModeConfig();
    bno055_writeData(BNO055_ACC_CONFIG, accConfigReg);
    bno055_setPage(0); //tornare in page 0
}

void bno055_configureGyroscope(uint8_t gRange, uint8_t bandwidth, uint8_t operationMode) {
	bno055_setPage(1); //il registro si trova in page 1
    uint8_t gyrConfigReg_0 = 0;
    uint8_t gyrConfigReg_1 = 0;

    if (gRange == 2000) {
    	gyrConfigReg_0 |= 0 ;  // [GYRO_Config_0]: xxxxx000b
    } else if (gRange == 1000) {
    	gyrConfigReg_0 |= 1 ;  // [GYRO_Config_0]: xxxxx001b
    } else if (gRange == 500) {
    	gyrConfigReg_0 |= 2 ;  // [GYRO_Config_0]: xxxxx010b
    } else if (gRange == 250) {
    	gyrConfigReg_0 |= 3 ;  // [GYRO_Config_0]: xxxxx011b
    } else if (gRange == 125) {
    	gyrConfigReg_0 |= 4 ;  // [GYRO_Config_0]: xxxxx100b
    } else {
        printf("Errore: G Range gyr non valido\r\n");
        return;
    }

    if (bandwidth == 523) {
    	gyrConfigReg_0 |= (0 << 3);  // [GYRO_Config_0]: xx000xxxb
    } else if (bandwidth == 230) {
    	gyrConfigReg_0 |= (1 << 3);  // [GYRO_Config_0]: xx001xxxb
    } else if (bandwidth == 116) {
    	gyrConfigReg_0 |= (2 << 3);  // [GYRO_Config_0]: xx010xxxb
    } else if (bandwidth == 47) {
    	gyrConfigReg_0 |= (3 << 3);  // [GYRO_Config_0]: xx011xxxb
    } else if (bandwidth == 23) {
    	gyrConfigReg_0 |= (4 << 3);  // [GYRO_Config_0]: xx100xxxb
    } else if (bandwidth == 12) {
    	gyrConfigReg_0 |= (5 << 3);  // [GYRO_Config_0]: xx101xxxb
    } else if (bandwidth == 64) {
    	gyrConfigReg_0 |= (6 << 3);  // [GYRO_Config_0]: xx110xxxb
    } else if (bandwidth == 32) {
    	gyrConfigReg_0 |= (7 << 3);  // [GYRO_Config_0]: xx111xxxb
    } else {
        printf("Errore: Bandwidth gyr non valido\r\n");
        return;
    }

    if (operationMode == NORMAL) {
    	gyrConfigReg_1 |= 0;         // [GYRO_Config_1]: xxxxx000b
    } else if (operationMode == FAST_POWER_UP ) {
    	gyrConfigReg_1 |= 1;  // [GYRO_Config_1]: xxxxx001b
    } else if (operationMode == DEEP_SUSPEND) {
    	gyrConfigReg_1 |= 2;  // [GYRO_Config_1]: xxxxx010b
    } else if (operationMode == SUSPEND) {
    	gyrConfigReg_1 |= 3;  // [GYRO_Config_1]: xxxxx011b
    } else if (operationMode == ADVANCED_POWERSAVE) {
    	gyrConfigReg_1 |= 4;  // [GYRO_Config_1]: xxxxx100b
    } else {
        printf("Errore: Modalità di funzionamento gyr non valida\r\n");
        return;
    }

    bno055_setOperationModeConfig();
    bno055_writeData(BNO055_GYRO_CONFIG_0, gyrConfigReg_0);
    bno055_writeData(BNO055_GYRO_CONFIG_1, gyrConfigReg_1);
    bno055_setPage(0); //tornare in page 0
}

void bno055_configureMagnetometer(uint8_t dataOutputRate, uint8_t operationMode, uint8_t powerMode) {
	bno055_setPage(1); //il registro si trova in page 1
    uint8_t magConfigReg = 0;

    if (dataOutputRate == 2) {
    	magConfigReg |= 0 ;  // [MAG_Config]: xxxxx000b
    } else if (dataOutputRate == 6) {
    	magConfigReg |= 1 ;  // [MAG_Config]: xxxxx001b
    } else if (dataOutputRate == 8) {
    	magConfigReg |= 2 ;  // [MAG_Config]: xxxxx010b
    } else if (dataOutputRate == 10) {
    	magConfigReg |= 3 ;  // [MAG_Config]: xxxxx011b
    } else if (dataOutputRate == 15) {
    	magConfigReg |= 4 ;  // [MAG_Config]: xxxxx100b
    } else if (dataOutputRate == 20) {
		magConfigReg |= 5 ;  // [MAG_Config]: xxxxx101b
	} else if (dataOutputRate == 25) {
		magConfigReg |= 6 ;  // [MAG_Config]: xxxxx110b
	} else if (dataOutputRate == 30) {
		magConfigReg |= 7 ;  // [MAG_Config]: xxxxx111b
    } else {
        printf("Errore: Data Output Rate non valido\r\n");
        return;
    }

    if (operationMode == LOW_POWER) {
    	magConfigReg |= (0 << 3);  // [MAG_Config]: xxx00xxxb
    } else if (operationMode == REGULAR) {
    	magConfigReg |= (1 << 3);  // [MAG_Config]: xxx01xxxb
    } else if (operationMode == ENHANCED_REGULAR) {
    	magConfigReg |= (2 << 3);  // [MAG_Config]: xxx10xxxb
    } else if (operationMode == HIGH_ACCURACY) {
    	magConfigReg |= (3 << 3);  // [MAG_Config]: xxx11xxxb
    } else {
        printf("Errore: Operation Mode non valida\r\n");
        return;
    }

    if (powerMode == NORMAL) {
    	magConfigReg |= (0 << 5);  // [MAG_Config]: x00xxxxxb
    } else if (powerMode == SLEEP ) {
    	magConfigReg |= (1 << 5);  // [MAG_Config]: x01xxxxxb
    } else if (powerMode == SUSPEND) {
    	magConfigReg |= (2 << 5);  // [MAG_Config]: x10xxxxxb
    } else if (powerMode == FORCE_MODE) {
    	magConfigReg |= (3 << 5);  // [MAG_Config]: x11xxxxxb
    } else {
        printf("Errore: Power Mode non valida\r\n");
        return;
    }

    bno055_setOperationModeConfig();
    bno055_writeData(BNO055_MAG_CONFIG, magConfigReg);
    bno055_setPage(0); //tornare in page 0
}

// Funzione per impostare la modalità energetica
void bno055_setPowerMode(PowerMode mode) {
    // Leggi il valore corrente di PWR_MODE
    uint8_t currentMode;
    bno055_readData(BNO055_PWR_MODE, &currentMode, 1);

    // Modifica solo i bit relativi alla modalità energetica
    currentMode &= 0xFC; // Azzera i primi due bit
    currentMode |= mode; // Imposta la nuova modalità energetica

    // Scrivi il nuovo valore in PWR_MODE
    bno055_writeData(BNO055_PWR_MODE, currentMode);
}

// Funzione per ottenere la modalità energetica attuale
PowerMode bno055_getPowerMode() {
    // Leggi il valore corrente di PWR_MODE
    uint8_t currentMode;
    bno055_readData(BNO055_PWR_MODE, &currentMode, 1);

    // Estrai la modalità energetica dai primi due bit
    currentMode &= 0x03;

    // Restituisci la modalità energetica come enumerazione PowerMode
    return (PowerMode)currentMode;
}

void bno055_configureUnits(uint8_t accelUnit, uint8_t gyroUnit, uint8_t eulerUnit, uint8_t tempUnit, uint8_t orientMode) {
    uint8_t unitConfig = 0;

    // Configura l'unità di accelerazione
    if (accelUnit == 0) {  //ms^2
        unitConfig |= 0; // xxxxxxx0b
    } else if (accelUnit == 1) {  // mg
        unitConfig |= 1; // xxxxxxx1b
    } else {
        printf("Errore: Acceleration Unit non valida\r\n");
        return;
    }

    // Configura l'unità di velocità angolare
    if (gyroUnit == 0) {  // Dps
        unitConfig |= (0 << 1); // xxxxxx0xb
    } else if (gyroUnit == 1) {  // Rps
        unitConfig |= (1 << 1); // xxxxxx1xb
    } else {
        printf("Errore: Gyro Unit non valida\r\n");
        return;
    }

    // Configura l'unità degli angoli di Eulero
    if (eulerUnit == 0) {  // Gradi
        unitConfig |= (0 << 2); // xxxxx0xxb
    } else if (eulerUnit == 1) {  // Radianti
        unitConfig |= (1 << 2); // xxxxx1xxb
    } else {
        printf("Errore: Euler Unit non valida\r\n");
        return;
    }

    // Configura l'unità di temperatura
    if (tempUnit == 0) {  // gradi celsius
        unitConfig |= (0 << 4); // xxx0xxxxb
    } else if (tempUnit == 1) {  // gradi farenheit
        unitConfig |= (1 << 4); // xxx1xxxxb
    } else {
        printf("Errore: Temperature Unit non valida\r\n");
        return;
    }

    // Configura la modalità di orientamento
	if (orientMode == 0) {  // Windows
		unitConfig |= (0 << 7); // 0xxxxxxxb
	} else if (orientMode == 1) {  // Android
		unitConfig |= (1 << 7); // 1xxxxxxxb
	} else {
		printf("Errore: Orientation Mode non valida\r\n");
		return;
	}

	bno055_setOperationModeConfig();
    bno055_writeData(BNO055_UNIT_SEL, unitConfig);
}

bno055_vector_xyz_int16_t calibrateAccel() {
    double xSum = 0, ySum = 0, zSum = 0;

    // Raccogli i dati di campionamento
    for (int i = 0; i < 100; i++) {
        bno055_vector_t accelData = bno055_getVectorAccelerometer(); // Legge i dati dell'accelerometro
        xSum += accelData.x;
        ySum += accelData.y;
        zSum += accelData.z;
        HAL_Delay(100);
    }

    // Calcola la media
    bno055_vector_xyz_int16_t offset;
    offset.x = xSum / 100;
    offset.y = ySum / 100;
    offset.z = (zSum / 100) - 9.81;

    return offset;
}

void setAccelCalibration(bno055_vector_xyz_int16_t offset) {
    bno055_calibration_data_t calData;

    // Imposta gli offset calcolati
    calData.offset.accel.x = offset.x;
    calData.offset.accel.y = offset.y;
    calData.offset.accel.z = offset.z;

    // Altri valori di offset e raggio possono essere impostati a 0 o a valori preesistenti
    calData.offset.gyro = (bno055_vector_xyz_int16_t){0};
    calData.offset.mag = (bno055_vector_xyz_int16_t){0};
    calData.radius.accel = 0;
    calData.radius.mag = 0;

    // Utilizza i valori calcolati per impostare i dati di calibrazione
    bno055_setCalibrationData(calData); // Assumi che questa funzione scriva i dati di calibrazione nel dispositivo
}

void bno055_set_int_en(uint8_t setAccNM, uint8_t setAccAM, uint8_t setAccHighG, uint8_t setGyrDRDY, uint8_t setGyrHighRate, uint8_t setGyrAM, uint8_t setMagDRDY, uint8_t setAccBSXDRDY){
	bno055_setPage(1);
	uint8_t setInt_en = 0;

	// 0 disabilita l'interrupt, 1 lo abilita
	if(setAccNM == 0){
		setInt_en |= (0 << 7);
	}else{
		setInt_en |= (1 << 7);
	}

	if(setAccAM == 0){
		setInt_en |= (0 << 6);
	}else{
		setInt_en |= (1 << 6);
	}

	if(setAccHighG == 0){
		setInt_en |= (0 << 5);
	}else{
		setInt_en |= (1 << 5);
	}

	if(setGyrDRDY == 0){
		setInt_en |= (0 << 4);
	}else{
		setInt_en |= (1 << 4);
	}

	if(setGyrHighRate == 0){
		setInt_en |= (0 << 3);
	}else{
		setInt_en |= (1 << 3);
	}

	if(setGyrAM == 0){
		setInt_en |= (0 << 2);
	}else{
		setInt_en |= (1 << 2);
	}

	if(setMagDRDY == 0){
		setInt_en |= (0 << 1);
	}else{
		setInt_en |= (1 << 1);
	}

	if(setAccBSXDRDY == 0){
		setInt_en |= (0 << 0);
	}else{
		setInt_en |= (1 << 0);
	}

	bno055_setOperationModeConfig();
	bno055_writeData(BNO055_INT_EN, setInt_en);
	bno055_setPage(0);
}

void bno055_set_int_msk(uint8_t setAccNM, uint8_t setAccAM, uint8_t setAccHighG, uint8_t setGyrDRDY, uint8_t setGyrHighRate, uint8_t setGyrAM, uint8_t setMagDRDY, uint8_t setAccBSXDRDY){
	bno055_setPage(1);
	uint8_t setInt_msk = 0;

	// 0 disabilita l'interrupt, 1 lo abilita
	if(setAccNM == 0){
		setInt_msk |= (0 << 7);
	}else{
		setInt_msk |= (1 << 7);
	}

	if(setAccAM == 0){
		setInt_msk |= (0 << 6);
	}else{
		setInt_msk |= (1 << 6);
	}

	if(setAccHighG == 0){
		setInt_msk |= (0 << 5);
	}else{
		setInt_msk |= (1 << 5);
	}

	if(setGyrDRDY == 0){
		setInt_msk |= (0 << 4);
	}else{
		setInt_msk |= (1 << 4);
	}

	if(setGyrHighRate == 0){
		setInt_msk |= (0 << 3);
	}else{
		setInt_msk |= (1 << 3);
	}

	if(setGyrAM == 0){
		setInt_msk |= (0 << 2);
	}else{
		setInt_msk |= (1 << 2);
	}

	if(setMagDRDY == 0){
		setInt_msk |= (0 << 1);
	}else{
		setInt_msk |= (1 << 1);
	}

	if(setAccBSXDRDY == 0){
		setInt_msk |= (0 << 0);
	}else{
		setInt_msk |= (1 << 0);
	}

	bno055_setOperationModeConfig();
	bno055_writeData(BNO055_INT_MSK, setInt_msk);
	bno055_setPage(0);
}

void bno055_set_acc_int_settings(uint8_t HG_Z_axis, uint8_t HG_Y_axis, uint8_t HG_X_axis, uint8_t AM_NM_Z_axis, uint8_t AM_NM_Y_axis, uint8_t AM_NM_X_axis, uint8_t AM_DUR1, uint8_t AM_DUR0){
	bno055_setPage(1);
	uint8_t setAccIntSet = 0;
	if(HG_Z_axis == 0){
		setAccIntSet |= (0 << 7);
	} else{
		setAccIntSet |= (1 << 7);
	}

	if(HG_Y_axis == 0){
		setAccIntSet |= (0 << 6);
	} else{
		setAccIntSet |= (1 << 6);
	}

	if(HG_X_axis == 0){
		setAccIntSet |= (0 << 5);
	} else{
		setAccIntSet |= (1 << 5);
	}

	if(AM_NM_Z_axis == 0){
		setAccIntSet |= (0 << 4);
	} else{
		setAccIntSet |= (1 << 4);
	}

	if(AM_NM_Y_axis == 0){
		setAccIntSet |= (0 << 3);
	} else{
		setAccIntSet |= (1 << 3);
	}

	if(AM_NM_X_axis == 0){
		setAccIntSet |= (0 << 2);
	} else{
		setAccIntSet |= (1 << 2);
	}

	if(AM_DUR1 == 0){
		setAccIntSet |= (0 << 1);
	} else{
		setAccIntSet |= (1 << 1);
	}

	if(AM_DUR0 == 0){
		setAccIntSet |= (0 << 0);
	} else{
		setAccIntSet |= (1 << 0);
	}

	bno055_setOperationModeConfig();
	bno055_writeData(BNO055_ACC_INT_SETTINGS, setAccIntSet);
	bno055_setPage(0);
}

void bno055_set_gyr_int_settings(uint8_t HR_Filt, uint8_t AM_Filt, uint8_t HR_Z_Axis, uint8_t HR_Y_Axis, uint8_t HR_X_Axis, uint8_t AM_Z_Axis, uint8_t AM_Y_Axis, uint8_t AM_X_Axis){
	bno055_setPage(1);
	uint8_t setGyrIntSet = 0;

	if(HR_Filt == 0){
		setGyrIntSet |= (0 << 7);
	}else{
		setGyrIntSet |= (1 << 7);
	}

	if(AM_Filt == 0){
		setGyrIntSet |= (0 << 6);
	}else{
		setGyrIntSet |= (1 << 6);
	}

	if(HR_Z_Axis == 0){
		setGyrIntSet |= (0 << 5);
	}else{
		setGyrIntSet |= (1 << 5);
	}

	if(HR_Y_Axis == 0){
		setGyrIntSet |= (0 << 4);
	}else{
		setGyrIntSet |= (1 << 4);
	}

	if(HR_X_Axis == 0){
		setGyrIntSet |= (0 << 3);
	}else{
		setGyrIntSet |= (1 << 3);
	}

	if(AM_Z_Axis == 0){
		setGyrIntSet |= (0 << 2);
	}else{
		setGyrIntSet |= (1 << 2);
	}

	if(AM_Y_Axis == 0){
		setGyrIntSet |= (0 << 1);
	}else{
		setGyrIntSet |= (1 << 1);
	}

	if(AM_X_Axis == 0){
		setGyrIntSet |= (0 << 0);
	}else{
		setGyrIntSet |= (1 << 0);
	}

	bno055_setOperationModeConfig();
	bno055_writeData(BNO055_GYR_INT_SETTINGS, setGyrIntSet);
	bno055_setPage(0);
}

void bno055_get_int_en(){
	uint8_t len = 1;
	uint8_t dato[1];

	bno055_setPage(1);
	bno055_readData(BNO055_INT_EN, dato, len);
	bno055_setPage(0);
	printf("Il valore del registro BNO055_INT_EN è: %d\r\n", dato[0]); //valore in esadecimale
}

void bno055_get_int_msk(){
	uint8_t len = 1;
	uint8_t dato[1];

	bno055_setPage(1);
	bno055_readData(BNO055_INT_MSK, dato, len);
	bno055_setPage(0);
	printf("Il valore del registro BNO055_INT_MSK è: %d\r\n", dato[0]); //valore in esadecimale
}

void bno055_get_acc_int_settings(){
	uint8_t len = 1;
	uint8_t dato[1];

	bno055_setPage(1);
	bno055_readData(BNO055_ACC_INT_SETTINGS, dato, len);
	bno055_setPage(0);
	printf("Il valore del registro BNO055_ACC_INT_SETTINGS è: %d\r\n", dato[0]); //valore in esadecimale
}

void bno055_get_gyr_int_settings(){
	uint8_t len = 1;
	uint8_t dato[1];

	bno055_setPage(1);
	bno055_readData(BNO055_GYR_INT_SETTINGS, dato, len);
	bno055_setPage(0);
	printf("Il valore del registro BNO055_GYR_INT_SETTINGS è: %d\r\n", dato[0]); //valore in esadecimale
}
