#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"        // Disable brownout problems
#include <soc/rtc_cntl_reg.h>

// New sensor and motor driver includes
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================================
// Mission Control Definitions
// ===================================
enum MissionPhase {
    PHASE_0_0_JOIN_HOTSPOT,
    PHASE_1_0_INITIAL_MOTOR_TEST,
    PHASE_1_1_MOTOR_RAMP_UP,
    // PHASE_1_2_ULTRASONIC_MOTOR_CONTROL, // Removed as per previous instructions
    PHASE_2_1_ALL_MOTORS_50_PERCENT, // This phase will now follow PHASE_1_1 directly
    PHASE_2_2_RECONNECT_CAPTURE_DISPLAY,
    PHASE_2_3_MISSION_COMPLETE, // End of Mission #1 cycle

    // Mission #2 Phases (Updated)
    PHASE_M2_0_0_INIT, // Initial state for Mission 2 (might be skipped if button directly sets next) - Currently not used as button directly sets M2_1_0
    PHASE_M2_1_0_DISCONNECT_WIFI,
    PHASE_M2_2_0_WAIT_10S_A, // MODIFIED: Wait for 10 seconds
    PHASE_M2_3_0_MOTOR_COMPLEX_SEQUENCE, // MODIFIED: New complex motor sequence
    PHASE_M2_4_0_WAIT_10S_CAPTURE_IMAGE,     // MODIFIED: Wait for 10s, then capture image
    PHASE_M2_5_0_MOTOR_D_REVERSE_10S,          // MODIFIED: Motor D reverse for 10s
    PHASE_M2_6_0_TURN_OFF_MOTORS_UPLOAD_IMAGE,
    PHASE_M2_6_1_DISPLAY_IMAGE // Remains the same display logic
};

// =======================================================================
// FORWARD DECLARATIONS for functions and global variables defined in app_httpd.cpp
// These extern declarations tell the compiler that these functions and variables
// are defined elsewhere (likely in `app_httpd.cpp` for the web server).
// =======================================================================
extern "C" void startCameraServer();
extern "C" void stopCameraServer();
extern "C" void store_captured_image(camera_fb_t *fb);
extern "C" void clearCapturedImage();
extern camera_fb_t *g_captured_fb;          // Global frame buffer pointer to store the captured image
extern bool g_mission_started_by_button;    // Flag for Mission #1 start
extern bool g_mission2_started_by_button;   // New global flag for Mission #2 start

// Current mission phase, initialized to join hotspot
MissionPhase currentPhase = PHASE_0_0_JOIN_HOTSPOT;
// =======================================================================

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_AI_THINKER // Has PSRAM        <-- MAKE SURE THIS MATCHES YOUR BOARD
#include "camera_pins.h"        // Contains pin definitions specific to the selected camera model

// ===========================
// Hotspot Credentials (for ESP32-CAM to connect to)
// ===========================
const char *ssid = "ESPServer";
const char *password = "1c3CreamYummy";

// ===================================
// PCA9685 and TB6612 DC Motor Definitions
// ===================================
// Create an Adafruit_PWMServoDriver object, default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PCA9685 Channels for TB6612 (Motor A) - First TB6612, A-side
#define MOTOR_A_PWM_CHANNEL   0   // PWM signal for motor A speed control
#define MOTOR_A_AIN1_CHANNEL  1   // Input 1 for motor A direction
#define MOTOR_A_AIN2_CHANNEL  2   // Input 2 for motor A direction
#define MOTOR_STBY_CHANNEL    3   // Standby pin shared for all motors (active high)

// PCA9685 Channels for TB6612 (Motor B) - First TB6612, B-side
#define MOTOR_B_PWM_CHANNEL   4
#define MOTOR_B_BIN1_CHANNEL  5
#define MOTOR_B_BIN2_CHANNEL  6

// PCA9685 Channels for TB6612 (Motor C) - Second TB6612, A-side
#define MOTOR_C_PWM_CHANNEL   7
#define MOTOR_C_AIN1_CHANNEL  8
#define MOTOR_C_AIN2_CHANNEL  9

// PCA9685 Channels for TB6612 (Motor D) - Second TB6612, B-side
#define MOTOR_D_PWM_CHANNEL   10
#define MOTOR_D_BIN1_CHANNEL  11
#define MOTOR_D_BIN2_CHANNEL  12

#define MOTOR_MAX_SPEED       4095 // Max PWM value (0-4095) for 100% duty cycle with PCA9685
// Macro to convert percentage speed to PWM value
#define SPEED_PERCENT(percent) ((int)(MOTOR_MAX_SPEED * ((float)percent / 100.0)))
#define MOTOR_5_PERCENT_SPEED   SPEED_PERCENT(5)
#define MOTOR_10_PERCENT_SPEED  SPEED_PERCENT(10)
#define MOTOR_20_PERCENT_SPEED  SPEED_PERCENT(20)
#define MOTOR_30_PERCENT_SPEED  SPEED_PERCENT(30)
#define MOTOR_40_PERCENT_SPEED  SPEED_PERCENT(40) // New: 40% speed
#define MOTOR_50_PERCENT_SPEED  SPEED_PERCENT(50)
#define MOTOR_70_PERCENT_SPEED  SPEED_PERCENT(70)
#define MOTOR_75_PERCENT_SPEED  SPEED_PERCENT(75)
#define MOTOR_85_PERCENT_SPEED  SPEED_PERCENT(85) // New: 85% speed
#define MOTOR_95_PERCENT_SPEED  SPEED_PERCENT(95) // New: 95% speed
#define MOTOR_100_PERCENT_SPEED SPEED_PERCENT(100)

// I2C Pins for PCA9685
#define I2C_SDA_PIN 15
#define I2C_SCL_PIN 14

// ===================================
// Mission Control Variables
// ===================================
unsigned long phaseStartTime = 0; // Timestamp when the current phase started

// Flags to track if individual motors are currently commanded ON
bool motorA_IsOn = false;
bool motorB_IsOn = false;
bool motorC_IsOn = false;
bool motorD_IsOn = false;

// --- Phase Durations (in milliseconds) ---
const unsigned long PHASE_1_0_MOTORS_ON_DURATION_MS = 10000;  // 10 seconds
const unsigned long PHASE_1_0_MOTORS_OFF_DURATION_MS = 5000;   // 5 seconds
const unsigned long PHASE_1_1_RAMP_UP_DURATION_MS = 20000;    // 20 seconds for ramp up
const unsigned long PHASE_1_1_HOLD_50_DURATION_MS = 10000;    // 10 seconds hold at 50%
const unsigned long PHASE_1_1_MOTOR_OFF_DELAY_MS = 2000;    // 2 seconds delay between motor stops
const unsigned long PHASE_2_1_MOTORS_ON_DURATION_MS = 15000;  // 15 seconds
const unsigned long PHASE_2_2_IMAGE_DISPLAY_DURATION_MS = 20000; // 20 seconds for image display

// New Mission 2 Phase Durations (UPDATED based on user's latest request)
const unsigned long PHASE_M2_2_0_WAIT_DURATION_MS = 10000; // MODIFIED: 10 seconds wait

// Phase M2_3_0 durations for complex sequence
const unsigned long M2_3_0_ABC_RAMP_DURATION_MS = 15000; // ABC ramp finishes at 15 seconds
const unsigned long M2_3_0_D_START_DELAY_MS = 2000;      // Motor D starts at 2 seconds
const unsigned long M2_3_0_D_RUN_DURATION_MS = 10000;    // Motor D runs for 10 seconds
const unsigned long M2_3_0_STOP_WAIT_DURATION_MS = 300;  // 0.3 seconds wait after ABC ramp and D stop
const unsigned long M2_3_0_TIMEOUT_MS = 30000;           // 30 seconds total timeout for phase 3

const unsigned long PHASE_M2_4_0_WAIT_BEFORE_CAPTURE_MS = 10000; // MODIFIED: 10 seconds wait for initial motor run
const unsigned long M2_4_0_STOP_WAIT_DURATION_MS = 300; // New: 0.3 seconds wait after motor stop in phase 4

const unsigned long PHASE_M2_5_0_MOTOR_D_REVERSE_DURATION_MS = 10000; // MODIFIED: Motor D reverse for 10s

const unsigned long PHASE_M2_6_1_IMAGE_DISPLAY_DURATION_MS = 20000; // 20 seconds for image display
// =======================================================================
// Helper Functions (DEFINED BEFORE SETUP AND LOOP)
// =======================================================================

/**
 * @brief Sets the speed and direction for a single DC motor connected via TB6612 to PCA9685.
 * @param motorPWMChannel The PCA9685 channel for the motor's PWM signal.
 * @param ain1 The PCA9685 channel for the motor's IN1 control pin.
 * @param ain2 The PCA9685 channel for the motor's IN2 control pin.
 * @param speed The desired speed. Positive for forward, negative for reverse, 0 for stop.
 * The absolute value represents the PWM duty cycle (0-MOTOR_MAX_SPEED).
 */
void setMotorSpeed(int motorPWMChannel, int ain1, int ain2, int speed) {
    if (speed > 0) { // Forward rotation
        pwm.setPWM(ain1, 0, MOTOR_MAX_SPEED); // IN1 High
        pwm.setPWM(ain2, 0, 0);               // IN2 Low
        pwm.setPWM(motorPWMChannel, 0, speed); // Set PWM speed
    } else if (speed < 0) { // Reverse rotation
        pwm.setPWM(ain1, 0, 0);               // IN1 Low
        pwm.setPWM(ain2, 0, MOTOR_MAX_SPEED); // IN2 High
        pwm.setPWM(motorPWMChannel, 0, abs(speed)); // Set PWM speed (using absolute value)
    } else { // Stop (short brake for TB6612 - both IN1/IN2 High)
        pwm.setPWM(ain1, 0, MOTOR_MAX_SPEED); // IN1 High
        pwm.setPWM(ain2, 0, MOTOR_MAX_SPEED); // IN2 High
        pwm.setPWM(motorPWMChannel, 0, 0);    // PWM to 0
    }
    // Ensure standby is always active (HIGH) when controlling motors
    pwm.setPWM(MOTOR_STBY_CHANNEL, 0, MOTOR_MAX_SPEED);
}

// Wrapper functions for each motor to simplify control calls
void controlMotorA(int speed) {
    setMotorSpeed(MOTOR_A_PWM_CHANNEL, MOTOR_A_AIN1_CHANNEL, MOTOR_A_AIN2_CHANNEL, speed);
}
void stopMotorA() {
    setMotorSpeed(MOTOR_A_PWM_CHANNEL, MOTOR_A_AIN1_CHANNEL, MOTOR_A_AIN2_CHANNEL, 0);
    motorA_IsOn = false; // Update motor state flag
}
void controlMotorB(int speed) {
    setMotorSpeed(MOTOR_B_PWM_CHANNEL, MOTOR_B_BIN1_CHANNEL, MOTOR_B_BIN2_CHANNEL, speed);
}
void stopMotorB() {
    setMotorSpeed(MOTOR_B_PWM_CHANNEL, MOTOR_B_BIN1_CHANNEL, MOTOR_B_BIN2_CHANNEL, 0);
    motorB_IsOn = false;
}
void controlMotorC(int speed) {
    setMotorSpeed(MOTOR_C_PWM_CHANNEL, MOTOR_C_AIN1_CHANNEL, MOTOR_C_AIN2_CHANNEL, speed);
}
void stopMotorC() {
    setMotorSpeed(MOTOR_C_PWM_CHANNEL, MOTOR_C_AIN1_CHANNEL, MOTOR_C_AIN2_CHANNEL, 0);
    motorC_IsOn = false;
}
void controlMotorD(int speed) {
    setMotorSpeed(MOTOR_D_PWM_CHANNEL, MOTOR_D_BIN1_CHANNEL, MOTOR_D_BIN2_CHANNEL, speed);
}
void stopMotorD() {
    setMotorSpeed(MOTOR_D_PWM_CHANNEL, MOTOR_D_BIN1_CHANNEL, MOTOR_D_BIN2_CHANNEL, 0);
    motorD_IsOn = false;
}

/**
 * @brief Stops all four motors and resets their state flags.
 */
void stopAllMotors() {
    stopMotorA();
    stopMotorB();
    stopMotorC();
    stopMotorD();
    // No need to set motorX_IsOn = false here as individual stopMotorX functions handle it.
    // However, including them ensures full reset for robust state management.
    motorA_IsOn = false;
    motorB_IsOn = false;
    motorC_IsOn = false;
    motorD_IsOn = false;
}

/**
 * @brief Captures an image from the camera and stores it in the global frame buffer
 * for access by the web server.
 */
void captureAndStoreImageForWebServer() {
    camera_fb_t *fb = esp_camera_fb_get(); // Get the camera frame buffer
    if (!fb) {
        Serial.println("Camera capture failed!");
        return;
    }

    Serial.printf("Image captured! Size: %u bytes, Format: %d\n", fb->len, fb->format);
    if (fb->format != PIXFORMAT_JPEG) {
        Serial.println("Captured image is not JPEG! Cannot store/display.");
        esp_camera_fb_return(fb); // Return the frame buffer if not JPEG
    } else {
        store_captured_image(fb); // Store the JPEG image for the web server
        Serial.println("Image stored for web server display.");
    }
}

/**
 * @brief Initializes or re-initializes WiFi connection and starts the camera web server.
 * Includes a timeout for WiFi connection.
 */
void initializeWifiAndWebServer() {
    Serial.println("Attempting to connect to WiFi and start Web Server...");

    // Disconnect any previous WiFi connections and clear settings
    WiFi.disconnect(true, true);
    delay(50); // Small delay for disconnect to take effect

    WiFi.mode(WIFI_STA); // Set WiFi to Station mode (client)
    WiFi.begin(ssid, password); // Connect to the defined hotspot

    Serial.print("Connecting to WiFi ");
    unsigned long wifiConnectStart = millis();
    const unsigned long WIFI_CONNECT_TIMEOUT_MS = 60000; // 60-second timeout for WiFi connection

    // Wait until WiFi is connected or timeout occurs
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); // Wait 0.5 seconds before retrying
        Serial.print(".");
        if (millis() - wifiConnectStart > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.println("\nWiFi connection timed out! Will retry again.");
            stopCameraServer(); // Stop server if connection fails
            return; // Exit function
        }
    }

    Serial.println("\nWiFi Connected!");
    Serial.print("Local IP Address: ");
    Serial.println(WiFi.localIP()); // Print the assigned IP address
    startCameraServer(); // Start the camera web server
    Serial.println("Web Server Ready at local IP!");
}


// =======================================================================
// Setup and Loop Functions
// =======================================================================

void setup() {
    // Disable brownout detector to prevent unexpected resets
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200); // Initialize serial communication
    Serial.setDebugOutput(true); // Enable debug output to serial
    Serial.println("\n\nStarting ESP32-CAM Mission Control System...");

    g_mission_started_by_button = false;    // Initialize Mission #1 flag
    g_mission2_started_by_button = false;   // Initialize Mission #2 flag

    // --- Camera Configuration ---
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;     // XCLK frequency
    config.frame_size = FRAMESIZE_UXGA; // Default frame size
    config.pixel_format = PIXFORMAT_JPEG; // Output JPEG format
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // Grab a fresh frame when the buffer is empty
    config.fb_location = CAMERA_FB_IN_PSRAM; // Store frame buffer in PSRAM
    config.jpeg_quality = 12; // Initial JPEG quality (lower is better quality)
    config.fb_count = 1;      // One frame buffer by default

    // Adjust camera settings if PSRAM is found for higher resolution and multiple buffers
    if (psramFound()) {
        config.jpeg_quality = 10; // Better quality with PSRAM
        config.fb_count = 2;      // Two frame buffers for smoother streaming
        config.grab_mode = CAMERA_GRAB_LATEST; // Grab the latest frame when multiple buffers are used
        config.frame_size = FRAMESIZE_UXGA; // Use UXGA resolution (1600x1200)
        Serial.println("PSRAM found, using UXGA");
    } else {
        config.frame_size = FRAMESIZE_SVGA; // Fallback to SVGA (800x600) if no PSRAM
        config.fb_location = CAMERA_FB_IN_DRAM; // Store frame buffer in DRAM
        Serial.println("PSRAM not found, using SVGA");
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        Serial.println("Restarting...");
        delay(1000);
        ESP.restart(); // Restart ESP32 if camera initialization fails
        return;
    }
    Serial.println("Camera initialized.");

    // Get sensor object to configure advanced settings
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        Serial.println("Adjusting camera sensor settings for better image quality...");
        // Enable automatic exposure control
        s->set_exposure_ctrl(s, 1); // 0 = manual, 1 = auto
        // Increase brightness (range -2 to 2, 0 is default)
        // A value of 1 or 2 can significantly brighten dark images.
        s->set_brightness(s, 1); // Increase brightness by 1 step

        // Set gain ceiling to allow more light amplification in low light conditions.
        // GAINCEILING_8X or GAINCEILING_16X are good starting points for dark environments.
        // Higher gain can introduce more noise.
        s->set_gainceiling(s, GAINCEILING_8X); // Allow gain up to 8x
        // Consider setting AWB (Auto White Balance) to 1 (enabled) if it's not already by default
        s->set_whitebal(s, 1); // Enable AWB
        s->set_awb_gain(s, 1); // Enable AWB gain control
    } else {
        Serial.println("Failed to get camera sensor object, cannot adjust advanced settings.");
    }

    // --- Motor Driver Setup ---
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // Initialize I2C communication for PCA9685
    pwm.begin();                          // Initialize PCA9685
    pwm.setPWMFreq(1000);                 // Set PWM frequency to 1000 Hz (suitable for DC motors)

    stopAllMotors(); // Ensure all motors are stopped at startup
    // Set STBY high to enable the motor drivers (TB6612).
    // This is set once here and then handled within setMotorSpeed to ensure it's always high.
    pwm.setPWM(MOTOR_STBY_CHANNEL, 0, MOTOR_MAX_SPEED);

    Serial.println("DC Motor Drivers initialized.");

    // Set initial mission phase and start time
    currentPhase = PHASE_0_0_JOIN_HOTSPOT;
    phaseStartTime = millis();
    // Attempt to connect to WiFi and start the web server immediately
    initializeWifiAndWebServer();
}

void loop() {
    unsigned long currentTime = millis(); // Get current time for phase duration checks
    static MissionPhase lastLoggedPhase = (MissionPhase)-1; // Static variable to track last logged phase

    // Log phase entry only once when it changes
    if (currentPhase != lastLoggedPhase) {
        Serial.print("\n--- Entering Phase: ");
        switch (currentPhase) {
            case PHASE_0_0_JOIN_HOTSPOT: Serial.println("PHASE_0_0_JOIN_HOTSPOT ---"); break;
            case PHASE_1_0_INITIAL_MOTOR_TEST: Serial.println("PHASE_1_0_INITIAL_MOTOR_TEST ---"); break;
            case PHASE_1_1_MOTOR_RAMP_UP: Serial.println("PHASE_1_1_MOTOR_RAMP_UP ---"); break;
            case PHASE_2_1_ALL_MOTORS_50_PERCENT: Serial.println("PHASE_2_1_ALL_MOTORS_50_PERCENT ---"); break;
            case PHASE_2_2_RECONNECT_CAPTURE_DISPLAY: Serial.println("PHASE_2_2_RECONNECT_CAPTURE_DISPLAY ---"); break;
            case PHASE_2_3_MISSION_COMPLETE: Serial.println("PHASE_2_3_MISSION_COMPLETE ---"); break;
            case PHASE_M2_0_0_INIT: Serial.println("PHASE_M2_0_0_INIT ---"); break;
            case PHASE_M2_1_0_DISCONNECT_WIFI: Serial.println("PHASE_M2_1_0_DISCONNECT_WIFI ---"); break;
            case PHASE_M2_2_0_WAIT_10S_A: Serial.println("PHASE_M2_2_0_WAIT_10S_A ---"); break; // Updated name
            case PHASE_M2_3_0_MOTOR_COMPLEX_SEQUENCE: Serial.println("PHASE_M2_3_0_MOTOR_COMPLEX_SEQUENCE ---"); break; // Updated name
            case PHASE_M2_4_0_WAIT_10S_CAPTURE_IMAGE: Serial.println("PHASE_M2_4_0_WAIT_10S_CAPTURE_IMAGE ---"); break; // Updated name
            case PHASE_M2_5_0_MOTOR_D_REVERSE_10S: Serial.println("PHASE_M2_5_0_MOTOR_D_REVERSE_10S ---"); break; // Updated name
            case PHASE_M2_6_0_TURN_OFF_MOTORS_UPLOAD_IMAGE: Serial.println("PHASE_M2_6_0_TURN_OFF_MOTORS_UPLOAD_IMAGE ---"); break;
            case PHASE_M2_6_1_DISPLAY_IMAGE: Serial.println("PHASE_M2_6_1_DISPLAY_IMAGE ---"); break;
            default: Serial.println("UNKNOWN_PHASE ---"); break;
        }
        lastLoggedPhase = currentPhase;
    }


    // State machine to manage mission phases
    switch (currentPhase) {
        case PHASE_0_0_JOIN_HOTSPOT:
            // Check if Mission #1 has been triggered by the web button
            if (g_mission_started_by_button) {
                Serial.println("Phase 0.0 Complete. Mission 1 started by button. Disconnecting from Hotspot. Moving to Phase 1.0.");
                stopCameraServer(); // Stop the camera streaming
                WiFi.disconnect(true); // Disconnect from WiFi
                delay(100); // Small delay for network cleanup
                Serial.println("Disconnected from WiFi.");
                currentPhase = PHASE_1_0_INITIAL_MOTOR_TEST; // Transition to Mission #1 motor test
                phaseStartTime = currentTime; // Reset phase timer
                stopAllMotors(); // Ensure all motors are off before starting new phase
                g_mission_started_by_button = false; // Reset the flag for next mission trigger
            }
            // Check if Mission #2 has been triggered by the web button
            else if (g_mission2_started_by_button) {
                Serial.println("Phase 0.0 Complete. Mission 2 started by button. Disconnecting from Hotspot. Moving to Phase M2_1_0.");
                stopCameraServer(); // Stop the camera streaming
                WiFi.disconnect(true); // Disconnect from WiFi
                delay(100); // Small delay for network cleanup
                Serial.println("Disconnected from WiFi.");
                currentPhase = PHASE_M2_1_0_DISCONNECT_WIFI; // Start Mission #2 flow
                phaseStartTime = currentTime; // Reset phase timer
                stopAllMotors(); // Ensure all motors are off
                g_mission2_started_by_button = false; // Reset the flag for next mission trigger
            }
            // If no mission started, continuously try to keep WiFi connected and server running
            else {
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("WiFi disconnected in Phase 0.0. Attempting to reconnect/scan.");
                    initializeWifiAndWebServer(); // Reconnect if disconnected
                }
            }
            break;

        case PHASE_1_0_INITIAL_MOTOR_TEST:
            // Motors run for a defined duration
            if (currentTime - phaseStartTime < PHASE_1_0_MOTORS_ON_DURATION_MS) {
                if (!motorA_IsOn) { // Check if motors are not yet commanded ON
                    Serial.println("Phase 1.0: Turning all motors ON at 20% speed.");
                    controlMotorA(MOTOR_20_PERCENT_SPEED);
                    controlMotorB(MOTOR_20_PERCENT_SPEED);
                    controlMotorC(MOTOR_20_PERCENT_SPEED);
                    controlMotorD(MOTOR_20_PERCENT_SPEED);
                    motorA_IsOn = true; motorB_IsOn = true; motorC_IsOn = true; motorD_IsOn = true; // Set flags
                }
            }
            // Motors stop for a defined duration after initial run
            else if (currentTime - phaseStartTime >= PHASE_1_0_MOTORS_ON_DURATION_MS &&
                     currentTime - phaseStartTime < (PHASE_1_0_MOTORS_ON_DURATION_MS + PHASE_1_0_MOTORS_OFF_DURATION_MS)) {
                if (motorA_IsOn) { // Check if motors are still commanded ON
                    Serial.println("Phase 1.0: Turning all motors OFF.");
                    stopAllMotors(); // Stop all motors
                    // motorA_IsOn, etc., are set to false by stopAllMotors()
                }
            }
            // Phase complete, transition to next phase
            else {
                Serial.println("Phase 1.0 Complete. Moving to Phase 1.1.");
                currentPhase = PHASE_1_1_MOTOR_RAMP_UP; // Transition
                phaseStartTime = currentTime; // Reset phase timer
                stopAllMotors(); // Ensure all motors are off before ramp up
            }
            break;

        case PHASE_1_1_MOTOR_RAMP_UP: {
            unsigned long phaseDuration = currentTime - phaseStartTime;

            // Ramp up motors from 5% to 50% over PHASE_1_1_RAMP_UP_DURATION_MS
            if (phaseDuration < PHASE_1_1_RAMP_UP_DURATION_MS) {
                float progress = (float)phaseDuration / PHASE_1_1_RAMP_UP_DURATION_MS;
                int currentSpeed = (int)(MOTOR_5_PERCENT_SPEED + (MOTOR_50_PERCENT_SPEED - MOTOR_5_PERCENT_SPEED) * progress);

                // Only update PWM if the speed changes or motors were off
                if (motorA_IsOn == false || pwm.getPWM(MOTOR_A_PWM_CHANNEL) != currentSpeed) {
                    Serial.printf("Phase 1.1: Ramping up motors to %d PWM\n", currentSpeed);
                    controlMotorA(currentSpeed);
                    controlMotorB(currentSpeed);
                    controlMotorC(currentSpeed);
                    controlMotorD(currentSpeed);
                    motorA_IsOn = true; motorB_IsOn = true; motorC_IsOn = true; motorD_IsOn = true;
                }
            }
            // Hold all motors at 50% speed for PHASE_1_1_HOLD_50_DURATION_MS
            else if (phaseDuration < (PHASE_1_1_RAMP_UP_DURATION_MS + PHASE_1_1_HOLD_50_DURATION_MS)) {
                // Ensure motors are at 50% if they somehow changed or were off
                if (pwm.getPWM(MOTOR_A_PWM_CHANNEL) != MOTOR_50_PERCENT_SPEED) {
                    Serial.println("Phase 1.1: All motors at 50% speed.");
                    controlMotorA(MOTOR_50_PERCENT_SPEED);
                    controlMotorB(MOTOR_50_PERCENT_SPEED);
                    controlMotorC(MOTOR_50_PERCENT_SPEED);
                    controlMotorD(MOTOR_50_PERCENT_SPEED);
                }
            }
            // Gradually stop motors with a delay between each
            else {
                unsigned long timeInShutdown = phaseDuration - (PHASE_1_1_RAMP_UP_DURATION_MS + PHASE_1_1_HOLD_50_DURATION_MS);

                if (timeInShutdown < PHASE_1_1_MOTOR_OFF_DELAY_MS && motorA_IsOn) {
                    Serial.println("Phase 1.1: Stopping Motor A."); stopMotorA(); // motorA_IsOn set to false by stopMotorA()
                } else if (timeInShutdown >= PHASE_1_1_MOTOR_OFF_DELAY_MS && timeInShutdown < (PHASE_1_1_MOTOR_OFF_DELAY_MS * 2) && motorB_IsOn) {
                    Serial.println("Phase 1.1: Stopping Motor B."); stopMotorB();
                } else if (timeInShutdown >= (PHASE_1_1_MOTOR_OFF_DELAY_MS * 2) && timeInShutdown < (PHASE_1_1_MOTOR_OFF_DELAY_MS * 3) && motorC_IsOn) {
                    Serial.println("Phase 1.1: Stopping Motor C."); stopMotorC();
                } else if (timeInShutdown >= (PHASE_1_1_MOTOR_OFF_DELAY_MS * 3) && timeInShutdown < (PHASE_1_1_MOTOR_OFF_DELAY_MS * 4) && motorD_IsOn) {
                    Serial.println("Phase 1.1: Stopping Motor D."); stopMotorD();
                } else if (timeInShutdown >= (PHASE_1_1_MOTOR_OFF_DELAY_MS * 4)) {
                    // All motors stopped, transition to next phase
                    Serial.println("Phase 1.1 Complete. Moving to Phase 2.1 (Skipping 1.2 Ultrasonic).");
                    currentPhase = PHASE_2_1_ALL_MOTORS_50_PERCENT; // Directly transition to 2.1 as 1.2 is removed
                    phaseStartTime = currentTime; // Reset phase timer
                    stopAllMotors(); // Ensure all motors are definitely off for safety
                }
            }
            break;
        }

        case PHASE_2_1_ALL_MOTORS_50_PERCENT:
            // All motors run at 50% speed for PHASE_2_1_MOTORS_ON_DURATION_MS
            if (currentTime - phaseStartTime < PHASE_2_1_MOTORS_ON_DURATION_MS) {
                if (!motorA_IsOn) { // Check if motors are not yet commanded ON
                    Serial.println("Phase 2.1: Turning all motors ON at 50% speed.");
                    controlMotorA(MOTOR_50_PERCENT_SPEED);
                    controlMotorB(MOTOR_50_PERCENT_SPEED);
                    controlMotorC(MOTOR_50_PERCENT_SPEED);
                    controlMotorD(MOTOR_50_PERCENT_SPEED);
                    motorA_IsOn = true; motorB_IsOn = true; motorC_IsOn = true; motorD_IsOn = true;
                }
            }
            // Phase complete, stop motors and transition
            else {
                Serial.println("Phase 2.1 Complete. Moving to Phase 2.2.");
                stopAllMotors(); // Stop all motors
                currentPhase = PHASE_2_2_RECONNECT_CAPTURE_DISPLAY; // Transition
                phaseStartTime = currentTime; // Reset phase timer
                clearCapturedImage(); // Clear any previously captured image
            }
            break;

        case PHASE_2_2_RECONNECT_CAPTURE_DISPLAY:
            // Ensure WiFi is connected
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("Phase 2.2: WiFi disconnected. Attempting to reconnect/scan.");
                initializeWifiAndWebServer(); // Reconnect
            }

            // If WiFi is connected and no image has been captured yet, capture and store one
            if (WiFi.status() == WL_CONNECTED && g_captured_fb == NULL) {
                Serial.println("Phase 2.2: Capturing new image and storing for web server display.");
                captureAndStoreImageForWebServer();
                Serial.println("Image captured and ready for display.");
            }
            
            // If WiFi is connected and an image is available, transition to display phase
            if (g_captured_fb != NULL && WiFi.status() == WL_CONNECTED) {
                Serial.println("Phase 2.2 Complete. Moving to Phase 2.3 (Image Display Phase).");
                currentPhase = PHASE_2_3_MISSION_COMPLETE; // Transition
                phaseStartTime = currentTime; // Start timer for display duration
            }
            break;

        case PHASE_2_3_MISSION_COMPLETE:
            // Ensure motors are off if they were somehow left on
            if (motorA_IsOn || motorB_IsOn || motorC_IsOn || motorD_IsOn) {
                Serial.println("Phase 2.3: Motors were on, stopping them as mission is complete.");
                stopAllMotors();
            }

            // After image display duration, reset system for a new mission
            if (currentTime - phaseStartTime >= PHASE_2_2_IMAGE_DISPLAY_DURATION_MS) {
                Serial.println("Phase 2.3: Image display duration ended. Resetting for new mission.");
                
                clearCapturedImage(); // Release the captured image buffer
                g_mission_started_by_button = false; // Reset Mission #1 flag
                g_mission2_started_by_button = false; // Reset Mission #2 flag

                stopCameraServer(); // Stop current camera server
                delay(500); // Small delay
                initializeWifiAndWebServer(); // Re-initialize WiFi and server for next mission

                Serial.println("System reset. Waiting for new mission start.");
                currentPhase = PHASE_0_0_JOIN_HOTSPOT; // Return to waiting phase
                phaseStartTime = currentTime; // Reset phase timer
            }
            break;

        // ===================================
        // Mission #2 Specific Phases (UPDATED)
        // ===================================

        case PHASE_M2_1_0_DISCONNECT_WIFI:
            Serial.println("Phase M2_1_0: Disconnecting from WiFi hotspot.");
            stopCameraServer(); // Stop the server
            WiFi.disconnect(true); // Disconnect from WiFi
            delay(100); // Allow time for disconnect
            Serial.println("Disconnected from WiFi.");
            currentPhase = PHASE_M2_2_0_WAIT_10S_A; // Transition to new wait phase
            phaseStartTime = currentTime; // Reset phase timer
            break;

        case PHASE_M2_2_0_WAIT_10S_A: // Renamed and duration changed
            Serial.printf("Phase M2_2_0: Waiting for 10 seconds. Time elapsed: %lu ms\n", currentTime - phaseStartTime);
            if (currentTime - phaseStartTime >= PHASE_M2_2_0_WAIT_DURATION_MS) { // Using new 10s duration
                Serial.println("Phase M2_2_0 Complete. Moving to Phase M2_3_0.");
                currentPhase = PHASE_M2_3_0_MOTOR_COMPLEX_SEQUENCE; // Transition to complex motor sequence
                phaseStartTime = currentTime; // Reset phase timer
                stopAllMotors(); // Ensure motors are off before complex sequence
            }
            break;

        case PHASE_M2_3_0_MOTOR_COMPLEX_SEQUENCE: { // New complex motor sequence logic
            unsigned long phaseDuration = currentTime - phaseStartTime;

            // Define internal timestamps/durations for clarity within this complex phase
            const unsigned long D_ACTIVATION_TIME_MS = M2_3_0_D_START_DELAY_MS; // 2000ms (2 seconds)
            const unsigned long D_DEACTIVATION_TIME_MS = D_ACTIVATION_TIME_MS + M2_3_0_D_RUN_DURATION_MS; // 2s + 10s = 12000ms
            const unsigned long ABC_RAMP_END_TIME_MS = M2_3_0_ABC_RAMP_DURATION_MS; // 15000ms (15 seconds)
            const unsigned long POST_ABC_RAMP_STOP_TIME_MS = ABC_RAMP_END_TIME_MS; // Time when ABC motors stop and 0.3s wait begins
            const unsigned long POST_STOP_WAIT_END_TIME_MS = POST_ABC_RAMP_STOP_TIME_MS + M2_3_0_STOP_WAIT_DURATION_MS; // 15s + 0.3s = 15300ms

            // Static flags to control single-shot actions within the phase
            static bool motorsStoppedForDelay = false;
            static unsigned long stopDelayInitiatedTime = 0; // Tracks when 0.3s delay started
            static bool abcSetTo85Percent = false; // Flag for the final ABC speed setting

            // --- Timeout check: if phase exceeds 30 seconds, return to initial state ---
            if (phaseDuration >= M2_3_0_TIMEOUT_MS) {
                Serial.println("Phase M2_3_0: Timeout (30s) reached. Resetting to initial web page.");
                stopAllMotors();
                clearCapturedImage();
                g_mission_started_by_button = false;
                g_mission2_started_by_button = false;
                stopCameraServer();
                delay(500); // Allow server to stop gracefully
                initializeWifiAndWebServer(); // Reconnect and start server
                currentPhase = PHASE_0_0_JOIN_HOTSPOT; // Go back to initial state
                phaseStartTime = currentTime;
                motorsStoppedForDelay = false; // Reset static flags for next mission
                abcSetTo85Percent = false;
                break; // Exit switch case for current loop iteration
            }

            // --- Section 1: ABC ramp (40% to 100%) and D run (100% from 2s to 12s) ---
            if (phaseDuration < ABC_RAMP_END_TIME_MS) { // Covers 0 to 15 seconds
                // Motors A, B, C: Ramp from 40% to 100% over 15 seconds
                float progress = (float)phaseDuration / ABC_RAMP_END_TIME_MS;
                int currentSpeedABC = (int)(MOTOR_40_PERCENT_SPEED + (MOTOR_100_PERCENT_SPEED - MOTOR_40_PERCENT_SPEED) * progress);

                if (pwm.getPWM(MOTOR_A_PWM_CHANNEL) != currentSpeedABC || !motorA_IsOn) {
                    Serial.printf("Phase M2_3_0: Motors A,B,C ramping to %d PWM. Time: %lu ms\n", currentSpeedABC, phaseDuration);
                    controlMotorA(currentSpeedABC);
                    controlMotorB(currentSpeedABC);
                    controlMotorC(currentSpeedABC);
                    motorA_IsOn = true; motorB_IsOn = true; motorC_IsOn = true;
                }

                // Motor D: Start at 2s, run for 10s at 100%
                if (phaseDuration >= D_ACTIVATION_TIME_MS && phaseDuration < D_DEACTIVATION_TIME_MS) {
                    if (!motorD_IsOn || pwm.getPWM(MOTOR_D_PWM_CHANNEL) != MOTOR_100_PERCENT_SPEED) {
                        Serial.printf("Phase M2_3_0: Motor D running at 100%%. Time: %lu ms\n", phaseDuration);
                        controlMotorD(MOTOR_100_PERCENT_SPEED);
                        motorD_IsOn = true;
                    }
                } else if (motorD_IsOn) { // Ensure D stops if its 10-second run is complete within this 15-second block
                    Serial.println("Phase M2_3_0: Stopping Motor D as its run duration is complete.");
                    stopMotorD();
                }
            }
            // --- Section 2: Stop all motors and 0.3s wait ---
            else if (phaseDuration >= ABC_RAMP_END_TIME_MS && phaseDuration < POST_STOP_WAIT_END_TIME_MS) {
                if (!motorsStoppedForDelay) {
                    Serial.println("Phase M2_3_0: All motors stopping for 0.3s delay.");
                    stopAllMotors(); // Ensure all motors are off.
                    stopDelayInitiatedTime = currentTime; // Mark the start of the 0.3s delay
                    motorsStoppedForDelay = true; // Set flag to ensure this block runs only once
                }
                Serial.printf("Phase M2_3_0: Waiting for 0.3s. Elapsed since stop: %lu ms\n", currentTime - stopDelayInitiatedTime);
            }
            // --- Section 3: Set ABC to 85% and transition ---
            else { // This block executes when phaseDuration >= POST_STOP_WAIT_END_TIME_MS
                // The 0.3s wait is over, now set ABC to 85% and transition
                if (!abcSetTo85Percent) { // Ensure this only happens once
                    Serial.println("Phase M2_3_0: Setting Motors A,B,C to 85% speed for transition.");
                    controlMotorA(MOTOR_85_PERCENT_SPEED); // They were 0% during the 0.3s stop
                    controlMotorB(MOTOR_85_PERCENT_SPEED);
                    controlMotorC(MOTOR_85_PERCENT_SPEED);
                    motorA_IsOn = true; motorB_IsOn = true; motorC_IsOn = true;
                    // Motor D should already be off from stopAllMotors() or prior logic
                    abcSetTo85Percent = true; // Mark as set
                }

                Serial.println("Phase M2_3_0 Complete. Moving to Phase M2_4_0 while A,B,C are at 85%.");
                currentPhase = PHASE_M2_4_0_WAIT_10S_CAPTURE_IMAGE; // MODIFIED: Transition to new phase name
                phaseStartTime = currentTime; // Reset timer for the next phase
                motorsStoppedForDelay = false; // Reset static flags for next time M2_3_0 runs
                abcSetTo85Percent = false;
            }
            break;
        }

        case PHASE_M2_4_0_WAIT_10S_CAPTURE_IMAGE: {
            unsigned long phaseDuration = currentTime - phaseStartTime;
            static bool motorsStoppedForImageCapture = false; // Flag to track motor stop for image capture
            static unsigned long motorStopStartTime = 0; // Timestamp for when motors were stopped in this phase
            static bool imageCaptured = false; // Flag to ensure image is taken only once

            // Check if we are in the initial 10-second motor run part
            if (phaseDuration < PHASE_M2_4_0_WAIT_BEFORE_CAPTURE_MS) {
                // Ensure motors A,B,C are running at 85% (as they were set at the end of previous phase)
                if (pwm.getPWM(MOTOR_A_PWM_CHANNEL) != MOTOR_85_PERCENT_SPEED || !motorA_IsOn) {
                    controlMotorA(MOTOR_85_PERCENT_SPEED);
                    controlMotorB(MOTOR_85_PERCENT_SPEED);
                    controlMotorC(MOTOR_85_PERCENT_SPEED);
                    motorA_IsOn = true; motorB_IsOn = true; motorC_IsOn = true;
                }
                if (motorD_IsOn) { stopMotorD(); } // Ensure D is off
                Serial.printf("Phase M2_4_0: Motors A,B,C running at 85%%. Waiting for 10s. Time elapsed: %lu ms\n", phaseDuration);
            }
            // Check if we have just reached or are within the 0.3s stop and wait period
            else if (phaseDuration >= PHASE_M2_4_0_WAIT_BEFORE_CAPTURE_MS &&
                     phaseDuration < (PHASE_M2_4_0_WAIT_BEFORE_CAPTURE_MS + M2_4_0_STOP_WAIT_DURATION_MS)) {

                if (!motorsStoppedForImageCapture) {
                    Serial.println("Phase M2_4_0: 10s reached. Stopping A,B,C motors for 0.3s wait.");
                    stopMotorA(); // Stop individual motors as specified
                    stopMotorB();
                    stopMotorC();
                    motorStopStartTime = currentTime; // Mark the time motors were stopped
                    motorsStoppedForImageCapture = true; // Set flag to ensure this part runs once
                }
                Serial.printf("Phase M2_4_0: Waiting 0.3s after A,B,C motor stop. Elapsed since stop: %lu ms\n", currentTime - motorStopStartTime);
            }
            // After the 0.3s wait, capture the image and transition
            else {
                if (!imageCaptured) { // Capture image only once
                    Serial.println("Phase M2_4_0: 0.3s wait complete. Taking an image.");
                    captureAndStoreImageForWebServer();
                    imageCaptured = true; // Mark image as taken
                }

                Serial.println("Phase M2_4_0 Complete. Moving to Phase M2_5_0.");
                currentPhase = PHASE_M2_5_0_MOTOR_D_REVERSE_10S;
                phaseStartTime = currentTime; // Reset timer for next phase

                // Reset static flags for the next time this phase runs
                motorsStoppedForImageCapture = false;
                imageCaptured = false;
            }
            break;
        }

        case PHASE_M2_5_0_MOTOR_D_REVERSE_10S: { // Renamed to reflect new duration
            unsigned long phaseDuration = currentTime - phaseStartTime;

            if (phaseDuration < PHASE_M2_5_0_MOTOR_D_REVERSE_DURATION_MS) { // Using new 10s duration
                if (!motorD_IsOn) {
                    Serial.println("Phase M2_5_0: Running Motor D in reverse at 100% for 10 seconds."); // Log updated duration
                    controlMotorD(-MOTOR_100_PERCENT_SPEED); // Negative speed for reverse
                    motorD_IsOn = true;
                    // Explicitly ensure A, B, C are off during this phase
                    if (motorA_IsOn) stopMotorA();
                    if (motorB_IsOn) stopMotorB();
                    if (motorC_IsOn) stopMotorC();
                }
            } else {
                Serial.println("Phase M2_5_0 Complete. Stopping Motor D. Moving to Phase M2_6_0.");
                stopMotorD();
                currentPhase = PHASE_M2_6_0_TURN_OFF_MOTORS_UPLOAD_IMAGE;
                phaseStartTime = currentTime;
            }
            break;
        }

        case PHASE_M2_6_0_TURN_OFF_MOTORS_UPLOAD_IMAGE:
            // Ensure all motors are off
            if (motorA_IsOn || motorB_IsOn || motorC_IsOn || motorD_IsOn) {
                Serial.println("Phase M2_6_0: Stopping all motors.");
                stopAllMotors();
            }

            // Reconnect WiFi if disconnected to upload/display image
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("Phase M2_6_0: WiFi disconnected. Attempting to reconnect.");
                initializeWifiAndWebServer();
            }
            
            // If WiFi is connected, transition to image display phase
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("Phase M2_6_0 Complete. WiFi connected. Moving to Phase M2_6_1 for image display.");
                currentPhase = PHASE_M2_6_1_DISPLAY_IMAGE;
                phaseStartTime = currentTime;
            }
            break;

        case PHASE_M2_6_1_DISPLAY_IMAGE:
            // If no image was captured in M2_4_0, skip display and proceed to reset
            if (g_captured_fb == NULL) {
                Serial.println("Phase M2_6_1: No image available for display. Proceeding to reset.");
                currentPhase = PHASE_0_0_JOIN_HOTSPOT;
                phaseStartTime = currentTime;
                clearCapturedImage();
                g_mission_started_by_button = false;
                g_mission2_started_by_button = false;
                stopCameraServer();
                delay(500);
                initializeWifiAndWebServer();
                break;
            }

            Serial.printf("Phase M2_6_1: Displaying image for 20 seconds. Time elapsed: %lu ms\n", currentTime - phaseStartTime);
            // After image display duration, reset system for a new mission
            if (currentTime - phaseStartTime >= PHASE_M2_6_1_IMAGE_DISPLAY_DURATION_MS) {
                Serial.println("Phase M2_6_1 Complete. Image display duration ended. Resetting for new mission.");
                
                clearCapturedImage(); // Release the captured image buffer
                g_mission_started_by_button = false; // Reset Mission #1 flag
                g_mission2_started_by_button = false; // Reset Mission #2 flag

                stopCameraServer(); // Stop current camera server
                delay(500); // Small delay
                initializeWifiAndWebServer(); // Re-initialize WiFi and server for next mission

                currentPhase = PHASE_0_0_JOIN_HOTSPOT; // Return to waiting phase
                phaseStartTime = currentTime; // Reset phase timer
            }
            break;
    }

    delay(100); // Small delay to prevent busy-waiting and allow other tasks (like WiFi/HTTPD) to run
}
