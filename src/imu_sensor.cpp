/*
 * Project: STM32F429-Gesture-Lock-NYU-RTES
 * Course: ECE-GY 6483 Real Time Embedded Systems
 * School: NYU Tandon School of Engineering
 * 
 * A gesture-based authentication system using STM32F429I-DISC1's I3G4250D gyroscope.
 * Part of the Embedded Sentry security system project.
 */


#include "imu_sensor.hpp"

// 移除全局变量
float ImuSensor::offset_x = 0.0f;
float ImuSensor::offset_y = 0.0f;
float ImuSensor::offset_z = 0.0f;

bool ImuSensor::setup() {
    _spi.format(8, 3);          // 8-bit mode, CPOL=1, CPHA=1
    _spi.frequency(1'000'000);  // 1MHz SPI clock

    // Initialize all registers
    write_reg(CTRL1, CTRL1_VAL);
    write_reg(CTRL2, CTRL2_VAL);
    write_reg(CTRL4, CTRL4_VAL);
    write_reg(CTRL5, CTRL5_VAL);

    return true;
}

void ImuSensor::handle_spi(int event) {
    _events.set(SPI_EVENT);
}

void ImuSensor::write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx_buf[2] = {reg, val};
    uint8_t rx_buf[2];

    _spi.transfer(tx_buf, 2, rx_buf, 2, callback(this, &ImuSensor::handle_spi));
    _events.wait_all(SPI_EVENT);
}

ImuSensor::MotionData ImuSensor::get_motion() {
    uint8_t tx_buf[7] = {DATA_START | 0xC0};  // Read + AutoInc
    uint8_t rx_buf[7];
    MotionData data;

    _spi.transfer(tx_buf, 7, rx_buf, 7, callback(this, &ImuSensor::handle_spi));
    _events.wait_all(SPI_EVENT);

    // 组装数据 - 先高字节后低字节
    for(int i = 0; i < 3; i++) {
        data.raw_data[i] = (int16_t)((rx_buf[2*i + 2] << 8) | rx_buf[2*i + 1]);
        
        // 减去校准值
        if(is_ready) {
            if(i == 0) data.raw_data[i] -= offset_x;
            if(i == 1) data.raw_data[i] -= offset_y;
            if(i == 2) data.raw_data[i] -= offset_z;
        }
        
        // 转换成角速度
        data.angle_speed[i] = (float)data.raw_data[i] * DPS_FACTOR;
    }

    return data;
}

void ImuSensor::zero_calibration() {
    // 采200个样本求平均，消除零点偏移
    const int SAMPLES = 200;
    float sum[3] = {0};
    
    for(int i = 0; i < SAMPLES; i++) {
        auto data = get_motion();
        sum[0] += data.raw_data[0];
        sum[1] += data.raw_data[1];
        sum[2] += data.raw_data[2];
        thread_sleep_for(1);
    }

    offset_x = sum[0] / SAMPLES;
    offset_y = sum[1] / SAMPLES;
    offset_z = sum[2] / SAMPLES;
    
    is_ready = true;
}

bool ImuSensor::calibration_status() {
    return is_ready;
}