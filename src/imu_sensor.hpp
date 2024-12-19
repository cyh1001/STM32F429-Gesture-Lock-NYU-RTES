/*
 * Project: STM32F429-Gesture-Lock-NYU-RTES
 * Course: ECE-GY 6483 Real Time Embedded Systems
 * School: NYU Tandon School of Engineering
 * 
 * A gesture-based authentication system using STM32F429I-DISC1's I3G4250D gyroscope.
 * Part of the Embedded Sentry security system project.
 */
#ifndef IMU_SENSOR_HPP
#define IMU_SENSOR_HPP

#include "mbed.h"

class ImuSensor {
private:
    // 寄存器地址
    static const uint8_t CTRL1 = 0x20;
    static const uint8_t CTRL2 = 0x21;
    static const uint8_t CTRL4 = 0x23;
    static const uint8_t CTRL5 = 0x24;
    static const uint8_t DATA_START = 0x28;

    // SPI Event Flag
    static const int SPI_EVENT = 1;

    // Configuration values
    static const uint8_t CTRL1_VAL = 0x4F;    // ODR 200Hz
    static const uint8_t CTRL2_VAL = 0x05;    // Normal mode + filter
    static const uint8_t CTRL4_VAL = 0x00;    // 245 dps range
    static const uint8_t CTRL5_VAL = 0x10;    // Filter enabled
    
    // Conversion factor
    static constexpr float DPS_FACTOR = 0.00875f;

    // 添加 EventFlags 作为成员变量
    EventFlags _events;
    SPI& _spi;

public:
    static float offset_x, offset_y, offset_z;
    bool is_ready = false;

    // 构造函数，接收 SPI 引用
    ImuSensor(SPI& spi) : _spi(spi) {}

    struct MotionData {
        float angle_speed[3];   // degrees per second
        int16_t raw_data[3];   // raw sensor values
    };

    bool setup();
    void write_reg(uint8_t reg, uint8_t val);
    void handle_spi(int event);
    MotionData get_motion();
    void zero_calibration();
    bool calibration_status();
};

#endif