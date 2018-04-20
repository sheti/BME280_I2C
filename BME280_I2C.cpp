#include "Arduino.h"
#include <Wire.h>
#include <BME280_I2C.h>

BME280_I2C::BME280_I2C() { 
    _i2caddr = BME280_I2C_ADDR;
};

BME280_I2C::BME280_I2C(uint8_t i2caddr) { 
    _i2caddr = i2caddr;
};

bool BME280_I2C::init() {
	if (read8(BME280_REGISTER_CHIPID) != 0x60)
		return false;
        
    // reset the device using soft-reset
    // this makes sure the IIR is off, etc.
    write8(BME280_REGISTER_SOFTRESET, 0xB6);

    // wait for chip to wake up.
    delay(300);
    
    // if chip is still reading calibration, delay
    while (isReadingCalibration())
        delay(100);
        
	readCalibrationData();
    
	setSampling(); // use defaults
    
    delay(100);
    
	return true;
}
/**************************************************************************/
/*!
    @brief return true if chip is busy reading cal data
*/
/**************************************************************************/
bool BME280_I2C::isReadingCalibration(void)
{
  uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

  return (rStatus & (1 << 0)) != 0;
}

void BME280_I2C::setSampling(
        sensor_mode       mode,
		sensor_sampling   tempSampling,
		sensor_sampling   pressSampling,
		sensor_sampling   humSampling,
		sensor_filter     filter,
		standby_duration  duration) {
	
    _measReg.mode     = mode;
    _measReg.osrs_t   = tempSampling;
    _measReg.osrs_p   = pressSampling;
        
    
    _humReg.osrs_h    = humSampling;
    _configReg.filter = filter;
    _configReg.t_sb   = duration;

    
    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
    write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
    write8(BME280_REGISTER_CONFIG, _configReg.get());
    write8(BME280_REGISTER_CONTROL, _measReg.get());
}
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
// t_fine carries fine temperature as global value
int32_t BME280_I2C::getTemperature(void) {
	int32_t var1, var2;

    int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
    //if (adc_T == 0x800000) // value in case temp measurement was disabled
    //    return NAN;
    adc_T >>= 4;

    var1 = ((((adc_T>>3) - ((int32_t)dig_T1 <<1))) *
            ((int32_t)dig_T2)) >> 11;
             
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) *
              ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    
	temperature = (t_fine * 5 + 128) >> 8;
	return temperature;
}
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_I2C::getPressure(bool isTempExist) {
	int64_t var1, var2, p;

	// Must be done first to get the t_fine variable set up
	if(!isTempExist)
        getTemperature();

	int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
    //if (adc_P == 0x800000) // value in case pressure measurement was disabled
    //    return NAN;
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
    var2 = var2 + (((int64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) +
           ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t BME280_I2C::getHumidity(bool isTempExist) {
    int32_t v_x1_u32r;

    if(!isTempExist)
        getTemperature(); // must be done first to get t_fine

    int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
    //if (adc_H == 0x8000) // value in case humidity measurement was disabled
    //    return NAN;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
                    (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    return  (uint32_t)(v_x1_u32r>>12);
}

void BME280_I2C::readCalibrationData(void) {
    dig_T1 = (uint16_t)read16LE(BME280_REGISTER_DIG_T1);
    dig_T2 = (int16_t)read16LE(BME280_REGISTER_DIG_T2);
    dig_T3 = (int16_t)read16LE(BME280_REGISTER_DIG_T3);

    dig_P1 = (uint16_t)read16LE(BME280_REGISTER_DIG_P1);
    dig_P2 = (int16_t)read16LE(BME280_REGISTER_DIG_P2);
    dig_P3 = (int16_t)read16LE(BME280_REGISTER_DIG_P3);
    dig_P4 = (int16_t)read16LE(BME280_REGISTER_DIG_P4);
    dig_P5 = (int16_t)read16LE(BME280_REGISTER_DIG_P5);
    dig_P6 = (int16_t)read16LE(BME280_REGISTER_DIG_P6);
    dig_P7 = (int16_t)read16LE(BME280_REGISTER_DIG_P7);
    dig_P8 = (int16_t)read16LE(BME280_REGISTER_DIG_P8);
    dig_P9 = (int16_t)read16LE(BME280_REGISTER_DIG_P9);

    dig_H1 = (uint8_t)read8(BME280_REGISTER_DIG_H1);
    dig_H2 = (int16_t)read16LE(BME280_REGISTER_DIG_H2);
    dig_H3 = (uint8_t)read8(BME280_REGISTER_DIG_H3);
    dig_H4 = (int16_t)((read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF));
    dig_H5 = (int16_t)((read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4));
    dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

void BME280_I2C::write8(byte _addr, byte _val) {
	Wire.beginTransmission(_i2caddr);   // start transmission to device 
	Wire.write(_addr); // send register address
	Wire.write(_val); // send value to write  
	Wire.endTransmission(); // end transmission
}

byte BME280_I2C::read8(byte _addr) {
	byte *_buff = read(_addr, 1);
	byte _result = _buff[0];
	free(_buff);
	return _result;
}

uint16_t BME280_I2C::read16LE(byte _addr) {
	uint16_t temp = read16(_addr);
	return (temp >> 8) | (temp << 8);
}

uint16_t BME280_I2C::read16(byte _addr) {
	byte *_buff = read(_addr, 2);
	uint16_t _result = ((uint16_t)_buff[0] << 8) | (uint16_t)_buff[1];
	free(_buff);
	return _result;
}

uint32_t BME280_I2C::read24(byte _addr) {
	byte *_buff = read(_addr, 3);
	uint32_t _result = ((uint32_t)_buff[0] << 16) | ((uint32_t)_buff[1] << 8) | (uint32_t)_buff[2];
	free(_buff);
	return _result;
}

uint32_t BME280_I2C::read32(byte _addr) {
	byte *_buff = read(_addr, 4);
	uint32_t _result = ((uint32_t)_buff[0] << 24) | ((uint32_t)_buff[1] << 16) | ((uint32_t)_buff[2] << 8) | (uint32_t)_buff[3];
	free(_buff);
	return _result;
}

byte *BME280_I2C::read(byte _addr, byte n) {
	byte *_buff = (byte *)malloc(sizeof(byte) * n);
	Wire.beginTransmission(_i2caddr); // start transmission to device 
	Wire.write(_addr); // sends register address to read from
	Wire.endTransmission(); // end transmission
	  
	Wire.beginTransmission(_i2caddr); // start transmission to device 
	Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)n);// send data n-bytes read
	byte i = 0;
	while (Wire.available()) {
		_buff[i] = Wire.read(); // receive DATA
		i += 1;
	}
	Wire.endTransmission(); // end transmission
	return _buff;
}
