/*
 * Project: STM32F429-Gesture-Lock-NYU-RTES
 * Course: ECE-GY 6483 Real Time Embedded Systems
 * School: NYU Tandon School of Engineering
 * 
 * A gesture-based authentication system using STM32F429I-DISC1's I3G4250D gyroscope.
 * Part of the Embedded Sentry security system project.
 */

#include "mbed.h"
#include "TS_DISCO_F429ZI.h"
#include "LCD_DISCO_F429ZI.h"
#include "imu_sensor.hpp"

// --- LCD and Touchscreen Initialization ---
LCD_DISCO_F429ZI lcd;
TS_DISCO_F429ZI ts;

// Debug LED
DigitalOut led1(LED1);

// ----- Arrays to store movement sequences -----
constexpr int MAX_SAMPLES = 30;  // Reduced to match reference implementation
constexpr int SAMPLE_INTERVAL_MS = 25;  // Match reference implementation timing

double stored_pattern[MAX_SAMPLES][3] = {0};  // Array to store reference gesture
double current_pattern[MAX_SAMPLES][3] = {0}; // Array to store current gesture

// ----- State variables -----
bool is_capturing = false;
bool is_matching = false;
uint8_t sample_count = 0;
bool pattern_stored = false;

// ----- Function Prototypes -----
void init_display();
void draw_ui();
void show_data(const ImuSensor::MotionData& motion);
void process_touch();
void show_count(uint8_t count);
void update_status(const char* status);
void show_result(bool success);
double compare_patterns();
uint16_t map_touch_x(uint16_t x);
uint16_t map_touch_y(uint16_t y);

// ----- Function Implementations -----
void init_display() {
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.SetFont(&Font16);
}

// 触摸屏坐标映射
uint16_t map_touch_x(uint16_t x) {
    return (x * lcd.GetXSize()) / 240;
}

uint16_t map_touch_y(uint16_t y) {
    return (y * lcd.GetYSize()) / 320;
}

void show_data(const ImuSensor::MotionData& motion) {
    char buf[6][32];
    
    // Convert values to strings with fixed precision
    for(int i = 0; i < 3; i++) {
        int whole = (int)motion.angle_speed[i];
        int frac = (int)abs((motion.angle_speed[i] - whole) * 100);
        snprintf(buf[i], sizeof(buf[i]), "%d.%02d", whole, frac);
        snprintf(buf[i+3], sizeof(buf[i+3]), "%d", motion.raw_data[i]);
    }

    // Clear previous values
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.FillRect(40, 10, 190, 100);
    
    lcd.SetTextColor(LCD_COLOR_WHITE);

    // Display readings
    const char* labels[] = {"X:", "Y:", "Z:"};
    for(int i = 0; i < 3; i++) {
        lcd.DisplayStringAt(10, 20+i*30, (uint8_t*)labels[i], LEFT_MODE);
        lcd.DisplayStringAt(40, 20+i*30, (uint8_t*)buf[i], LEFT_MODE);
        lcd.DisplayStringAt(130, 20+i*30, (uint8_t*)buf[i+3], LEFT_MODE);
    }
}

void draw_ui() {
    lcd.Clear(LCD_COLOR_BLACK);
    
    // Data display area
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DrawRect(5, 5, 230, 130);
    
    // Control buttons
    lcd.SetTextColor(LCD_COLOR_GREEN);
    lcd.FillRect(20, 140, 90, 60);
    lcd.SetTextColor(LCD_COLOR_BLUE);
    lcd.FillRect(130, 140, 90, 60);
    
    lcd.SetBackColor(LCD_COLOR_GREEN);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.DisplayStringAt(35, 160, (uint8_t*)"RECORD", LEFT_MODE);
    
    lcd.SetBackColor(LCD_COLOR_BLUE);
    lcd.DisplayStringAt(145, 160, (uint8_t*)"VERIFY", LEFT_MODE);
    
    lcd.SetBackColor(LCD_COLOR_BLACK);
}

void show_count(uint8_t count) {
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    char countStr[10];
    sprintf(countStr, "%d", count);
    lcd.SetFont(&Font20);
    lcd.DisplayStringAt(0, 210, (uint8_t*)countStr, CENTER_MODE);
    lcd.SetFont(&Font16);
}

void update_status(const char* status) {
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, 240, (uint8_t*)status, CENTER_MODE);
}

double compare_patterns() {
    double mse = 0.0;
    double energy1 = 0.0, energy2 = 0.0;
    double sum_stored[3] = {0.0}, sum_current[3] = {0.0};
    
    // Calculate MSE, energy and direction sums
    for(int i = 0; i < MAX_SAMPLES; i++) {
        for(int j = 0; j < 3; j++) {
            // MSE
            double diff = stored_pattern[i][j] - current_pattern[i][j];
            mse += diff * diff;
            
            // Energy
            energy1 += fabs(stored_pattern[i][j]);
            energy2 += fabs(current_pattern[i][j]);
            
            // Direction sums
            sum_stored[j] += stored_pattern[i][j];
            sum_current[j] += current_pattern[i][j];
        }
    }
    
    mse /= (MAX_SAMPLES * 3);
    double energy_diff = fabs(energy1 - energy2);
    
    // Calculate direction differences
    int sign_diff = 0;
    for(int k = 0; k < 3; k++) {
        if((sum_stored[k] >= 0) != (sum_current[k] >= 0)) {
            sign_diff++;
        }
    }
    
    // Normalize metrics
    double normalized_mse = 1.0 / (1.0 + mse);
    double normalized_energy = 1.0 / (1.0 + energy_diff);
    double normalized_sign = (3.0 - sign_diff) / 3.0;
    
    // Weighted combination
    const double w_mse = 0.6;
    const double w_energy = 0.3;
    const double w_sign = 0.1;
    
    double similarity = (w_mse * normalized_mse) +
                       (w_energy * normalized_energy) +
                       (w_sign * normalized_sign);
    
    printf("MSE: %.6f, Energy diff: %.6f, Sign diff: %d\n", mse, energy_diff, sign_diff);
    printf("Final similarity: %.6f\n", similarity);
    
    return similarity;
}

void show_result(bool success) {
    if (success) {
        for (int i = 0; i < 25; i++) {
            lcd.Clear(LCD_COLOR_GREEN);
            lcd.SetBackColor(LCD_COLOR_GREEN);
            lcd.SetTextColor(LCD_COLOR_WHITE);
            lcd.DisplayStringAt(0, 120, (uint8_t*)"SUCCESS!", CENTER_MODE);
            thread_sleep_for(80);

            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetBackColor(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_WHITE);
            lcd.DisplayStringAt(0, 120, (uint8_t*)"SUCCESS!", CENTER_MODE);
            thread_sleep_for(100);
        }
    } else {
        lcd.Clear(LCD_COLOR_RED);
        lcd.SetBackColor(LCD_COLOR_RED);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, 120, (uint8_t*)"FAILED!", CENTER_MODE);
        thread_sleep_for(1500);
    }
    
    init_display();
    draw_ui();
}

void process_touch() {
    TS_StateTypeDef state;
    ts.GetState(&state);
    
    if(!state.TouchDetected) return;

    uint16_t x = map_touch_x(state.X);
    uint16_t y = map_touch_y(state.Y);
    
    printf("Touch detected at X: %d, Y: %d (mapped)\n", x, y);
    
    // 将屏幕分为左右两部分，中间留出20像素的间隔
    // 左半边 (0-110) = RECORD
    if(x < 110) {
        led1 = !led1;
        is_capturing = true;
        sample_count = 0;
        printf("Record button pressed\n");
        update_status("Recording...");
    }
    // 右半边 (130-240) = VERIFY
    else if(x > 130) {
        if(!pattern_stored) {
            update_status("Record pattern first!");
            thread_sleep_for(1500);
            draw_ui();
            return;
        }
        led1 = !led1;
        is_matching = true;
        sample_count = 0;
        printf("Verify button pressed\n");
        update_status("Verifying...");
    }
}

int main() {
    printf("Starting initialization...\n");
    
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    ImuSensor imu(spi);
    
    if(!imu.setup()) {
        printf("IMU initialization failed\n");
        return -1;
    }
    printf("IMU initialized successfully\n");
    
    ts.Init(lcd.GetXSize(), lcd.GetYSize());
    init_display();
    draw_ui();
    
    printf("Calibrating IMU...\n");
    imu.zero_calibration();
    printf("Calibration complete\n");
    
    while(true) {
        auto motion = imu.get_motion();
        show_data(motion);
        process_touch();
        
        if(is_capturing || is_matching) {
            if(sample_count < MAX_SAMPLES) {
                double* pattern = is_capturing ? stored_pattern[sample_count] : current_pattern[sample_count];
                pattern[0] = motion.raw_data[0] / 32768.0;
                pattern[1] = motion.raw_data[1] / 32768.0;
                pattern[2] = motion.raw_data[2] / 32768.0;
                
                show_count(sample_count);
                sample_count++;
                thread_sleep_for(SAMPLE_INTERVAL_MS);
            } else {
                if(is_capturing) {
                    pattern_stored = true;
                    update_status("Pattern recorded!");
                } else {
                    double similarity = compare_patterns();
                    printf("Similarity: %.3f\n", similarity);
                    show_result(similarity > 0.80);  // Threshold from reference implementation
                }
                is_capturing = false;
                is_matching = false;
                sample_count = 0;
                thread_sleep_for(1500);
                draw_ui();
            }
        }
        
        thread_sleep_for(20);
    }
}