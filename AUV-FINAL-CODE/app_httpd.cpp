#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "driver/ledc.h"
#include "sdkconfig.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

#include "FS.h"
#include "SPIFFS.h"
#include <WiFi.h>

enum MissionPhase {
    PHASE_0_0_JOIN_HOTSPOT,
    PHASE_1_0_INITIAL_MOTOR_TEST,
    PHASE_1_1_MOTOR_RAMP_UP,
    PHASE_1_2_ULTRASONIC_MOTOR_CONTROL,
    PHASE_2_1_ALL_MOTORS_50_PERCENT,
    PHASE_2_2_RECONNECT_CAPTURE_DISPLAY,
    PHASE_2_3_MISSION_COMPLETE,

    // Mission #2 Phases
    PHASE_M2_0_0_INIT, // This might not be explicitly used in switch, but kept for enum consistency
    PHASE_M2_1_0_DISCONNECT_WIFI,
    PHASE_M2_2_0_WAIT_20S_A,
    PHASE_M2_3_0_MOTOR_RAMP_UP_COMPLEX,
    PHASE_M2_4_0_WAIT_20S_B_CAPTURE_IMAGE,
    PHASE_M2_5_0_MOTOR_RAMP_UP_V2,
    PHASE_M2_6_0_TURN_OFF_MOTORS_UPLOAD_IMAGE,
    PHASE_M2_6_1_DISPLAY_IMAGE // New phase for explicit image display duration
};

extern MissionPhase currentPhase;
bool g_mission_started_by_button = false;
bool g_mission2_started_by_button = false; // New global flag

camera_fb_t *g_captured_fb = NULL;
static SemaphoreHandle_t xFrameBufferMutex = NULL;
static httpd_handle_t camera_httpd = NULL; // Global handle for the web server

extern "C" void store_captured_image(camera_fb_t *fb) {
    if (xFrameBufferMutex == NULL) {
        xFrameBufferMutex = xSemaphoreCreateMutex(); // Create if not already
        Serial.println("Mutex created in store_captured_image (should be in startCameraServer).");
    }
    if (xSemaphoreTake(xFrameBufferMutex, portMAX_DELAY) == pdTRUE) {
        if (g_captured_fb) {
            Serial.println("Returning previous frame buffer before storing new one.");
            esp_camera_fb_return(g_captured_fb);
        }
        g_captured_fb = fb;
        xSemaphoreGive(xFrameBufferMutex);
    } else {
        Serial.println("Failed to take mutex in store_captured_image! Releasing new buffer.");
        esp_camera_fb_return(fb);
    }
}

extern "C" void clearCapturedImage() {
    if (xFrameBufferMutex == NULL) {
        Serial.println("Mutex not initialized for clearCapturedImage!");
        return;
    }
    if (xSemaphoreTake(xFrameBufferMutex, portMAX_DELAY) == pdTRUE) {
        if (g_captured_fb) {
            Serial.println("Clearing captured image (returning frame buffer).");
            esp_camera_fb_return(g_captured_fb);
            g_captured_fb = NULL;
        } else {
            Serial.println("No captured image to clear.");
        }
        xSemaphoreGive(xFrameBufferMutex);
    } else {
        Serial.println("Failed to take mutex in clearCapturedImage!");
    }
}

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t captured_image_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    if (xFrameBufferMutex == NULL) {
        httpd_resp_send_500(req);
        Serial.println("Mutex not initialized for image handler!");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(xFrameBufferMutex, portMAX_DELAY) == pdTRUE) {
        if (g_captured_fb) {
            httpd_resp_set_type(req, "image/jpeg");
            httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
            httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
            httpd_resp_set_hdr(req, "Pragma", "no-cache");
            httpd_resp_set_hdr(req, "Expires", "0");
            res = httpd_resp_send(req, (const char *)g_captured_fb->buf, g_captured_fb->len);
        } else {
            httpd_resp_set_type(req, "image/jpeg");
            httpd_resp_set_status(req, "204 No Content");
            httpd_resp_send(req, NULL, 0);
            Serial.println("No image in g_captured_fb to serve, sending 204 No Content.");
            res = ESP_OK;
        }
        xSemaphoreGive(xFrameBufferMutex);
    } else {
        httpd_resp_send_500(req);
        Serial.println("Failed to take mutex in captured_image_handler!");
        res = ESP_FAIL;
    }
    return res;
}

static esp_err_t start_mission_handler(httpd_req_t *req) {
    if (currentPhase == PHASE_0_0_JOIN_HOTSPOT) {
        g_mission_started_by_button = true;
        Serial.println("Received /start_mission request. Setting g_mission_started_by_button to true.");
        httpd_resp_sendstr(req, "Mission 1 start signal received. Initiating shutdown for phase transition.");
    } else {
        Serial.println("Ignoring /start_mission request, not in PHASE_0_0.");
        httpd_resp_sendstr(req, "Mission already started or not in initial phase.");
    }
    return ESP_OK;
}

static esp_err_t start_mission2_handler(httpd_req_t *req) {
    if (currentPhase == PHASE_0_0_JOIN_HOTSPOT) {
        g_mission2_started_by_button = true;
        Serial.println("Received /start_mission2 request. Setting g_mission2_started_by_button to true.");
        httpd_resp_sendstr(req, "Mission 2 start signal received. Initiating shutdown for phase transition.");
    } else {
        Serial.println("Ignoring /start_mission2 request, not in PHASE_0_0.");
        httpd_resp_sendstr(req, "Mission already started or not in initial phase.");
    }
    return ESP_OK;
}


// HTML content for the main web page - Now focuses on dynamic updates based on mission phase
// *** IMPORTANT: Updated JavaScript for robust resetting ***
static const char* STATIC_PAGE = R"rawliteral(
<html>
<head>
    <title>ESP32-CAM Mission Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: Arial, sans-serif; text-align: center; margin: 20px; background-color: #f0f0f0; }
        .card { background-color: white; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); padding: 15px; margin-bottom: 20px; }
        img { width: 100%; max-width: 640px; height: auto; border: 1px solid #ddd; border-radius: 4px; }
        h2 { color: #333; }
        p { color: #666; }
        .message { font-size: 1.5em; color: green; font-weight: bold; }
        .hidden { display: none; }
        .phase-name { font-size: 1.2em; color: #0056b3; font-weight: bold; }
        .start-button {
            background-color: #4CAF50; /* Green */
            color: white;
            padding: 15px 32px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 20px;
            margin: 10px 2px;
            cursor: pointer;
            border: none;
            border-radius: 8px;
            transition: background-color 0.3s ease;
        }
        .start-button:hover {
            background-color: #45a049;
        }
        .mission2-button {
            background-color: #008CBA; /* Blue */
            color: white;
            padding: 15px 32px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 20px;
            margin: 10px 2px;
            cursor: pointer;
            border: none;
            border-radius: 8px;
            transition: background-color 0.3s ease;
        }
        .mission2-button:hover {
            background-color: #007bb5;
        }
    </style>
</head>
<body>
    <div class="card" id="initial-message-card">
        <h2 class="message">ESP CAM connected</h2>
        <p>Waiting for mission to start...</p>
        <button class="start-button" id="start-mission-button">Start Mission #1</button>
        <button class="mission2-button" id="start-mission2-button">Start Mission #2</button>
    </div>

    <div class="card" id="connection-status-card" class="hidden">
        <h2>Mission Status</h2>
        <p id="wifi-status">WiFi Status: Unknown</p>
        <p id="ip-address">Local IP: ---</p>
        <p class="phase-name">Current Phase: <span id="current-phase-display">Waiting...</span></p>
    </div>

    <div class="card" id="captured-image-card" class="hidden">
        <h2>Last Captured Image</h2>
        <p>This image is captured during Phase 2.2 (Mission 1) or M2.4.0 (Mission 2).</p>
        <div id="captured-image-display">
            <img id="captured-img" src="/capture.jpg" alt="Captured Image">
        </div>
        <p id="image-countdown"></p>
    </div>

    <script>
        var imageDisplayStartTime = 0;
        // Image display duration for Mission 1 (Phase 2.3)
        var mission1ImageDisplayDuration = 20000;
        // Image display duration for Mission 2 (Phase M2.6.1)
        var mission2ImageDisplayDuration = 20000;
        var imageIntervalId;
        var wifiCheckInterval;

        const phaseDescriptions = {
            "0.0": "Phase 0.0: Join Hotspot & Display Web Server (Waiting for Start)",
            "1.0": "Phase 1.0: Initial Motor Test (Mission #1)",
            "1.1": "Phase 1.1: Motor Ramp Up (Mission #1)",
            "1.2": "Phase 1.2: Ultrasonic Motor Control (Mission #1)",
            "2.1": "Phase 2.1: All Motors 50% Speed (Mission #1)",
            "2.2": "Phase 2.2: Reconnect & Capture Image (Mission #1)",
            "2.3": "Phase 2.3: Mission #1 Complete (Image Display)",
            "M2.1.0": "Phase M2.1.0: Disconnect from Hotspot (Mission #2)",
            "M2.2.0": "Phase M2.2.0: Wait for 20 seconds (Mission #2)",
            "M2.3.0": "Phase M2.3.0: Complex Motor Ramp Up (Mission #2)",
            "M2.4.0": "Phase M2.4.0: Wait 20 seconds & Capture Image (Mission #2)",
            "M2.5.0": "Phase M2.5.0: Motor Ramp Up V2 (Mission #2)",
            "M2.6.0": "Phase M2.6.0: Turn Off Motors & Reconnect (Mission #2)",
            "M2.6.1": "Phase M2.6.1: Display Image (Mission #2 Complete)" // New description
        };

        // Function to fully reset the UI to the initial state
        function resetUIForPhase0() {
            console.log("Resetting UI for Phase 0.0");
            document.getElementById('initial-message-card').classList.remove('hidden');
            document.getElementById('start-mission-button').classList.remove('hidden');
            document.getElementById('start-mission2-button').classList.remove('hidden'); // Show new button
            document.getElementById('connection-status-card').classList.add('hidden');
            document.getElementById('captured-image-card').classList.add('hidden');

            clearInterval(imageIntervalId);
            imageIntervalId = null; // Clear the interval ID
            imageDisplayStartTime = 0;
            document.getElementById('image-countdown').innerText = "";
            document.getElementById('captured-img').src = ""; // Clear the image visually

            document.getElementById('wifi-status').innerText = "WiFi Status: Unknown";
            document.getElementById('ip-address').innerText = "Local IP: ---";
            document.getElementById('current-phase-display').innerText = "Waiting...";
        }

        document.getElementById('start-mission-button').addEventListener('click', function() {
            // Disable buttons to prevent multiple clicks during transition
            this.disabled = true;
            document.getElementById('start-mission2-button').disabled = true;
            this.innerText = "Starting...";

            fetch('/start_mission', { method: 'POST' })
                .then(response => response.text())
                .then(data => {
                    console.log('Start mission 1 response:', data);
                    alert("Mission 1 start signal sent! The ESP32-CAM will now begin its mission phases.");
                    // Immediately transition UI away from start buttons
                    document.getElementById('initial-message-card').classList.add('hidden');
                    document.getElementById('connection-status-card').classList.remove('hidden');
                })
                .catch(error => {
                    console.error('Error sending start mission 1 signal:', error);
                    alert("Failed to send start mission 1 signal! Please check ESP32 connection and try again.");
                    // Re-enable buttons on failure
                    document.getElementById('start-mission-button').disabled = false;
                    document.getElementById('start-mission2-button').disabled = false;
                    document.getElementById('start-mission-button').innerText = "Start Mission #1";
                });
        });

        document.getElementById('start-mission2-button').addEventListener('click', function() {
            // Disable buttons to prevent multiple clicks during transition
            this.disabled = true;
            document.getElementById('start-mission-button').disabled = true;
            this.innerText = "Starting...";

            fetch('/start_mission2', { method: 'POST' })
                .then(response => response.text())
                .then(data => {
                    console.log('Start mission 2 response:', data);
                    alert("Mission 2 start signal sent! The ESP32-CAM will now begin its mission phases.");
                    // Immediately transition UI away from start buttons
                    document.getElementById('initial-message-card').classList.add('hidden');
                    document.getElementById('connection-status-card').classList.remove('hidden');
                })
                .catch(error => {
                    console.error('Error sending start mission 2 signal:', error);
                    alert("Failed to send start mission 2 signal! Please check ESP32 connection and try again.");
                    // Re-enable buttons on failure
                    document.getElementById('start-mission2-button').disabled = false;
                    document.getElementById('start-mission-button').disabled = false;
                    document.getElementById('start-mission2-button').innerText = "Start Mission #2";
                });
        });


        function updateWifiStatus() {
            fetch('/wifi_status?' + new Date().getTime()) // Add cache-buster
                .then(response => {
                    if (!response.ok) {
                        // If network response is not ok, assume server is down or unreachable
                        throw new Error('Network response was not ok ' + response.statusText);
                    }
                    return response.json();
                })
                .then(data => {
                    document.getElementById('wifi-status').innerText = "WiFi Status: " + data.status;
                    document.getElementById('ip-address').innerText = "Local IP: " + data.ip;
                    document.getElementById('current-phase-display').innerText = phaseDescriptions[data.phase] || "UNKNOWN";

                    if (data.status === "CONNECTED") {
                        if (data.phase === "0.0") {
                            resetUIForPhase0(); // Fully reset UI when back to phase 0.0
                            document.getElementById('start-mission-button').disabled = false; // Re-enable button
                            document.getElementById('start-mission-button').innerText = "Start Mission #1";
                            document.getElementById('start-mission2-button').disabled = false; // Re-enable new button
                            document.getElementById('start-mission2-button').innerText = "Start Mission #2";
                        } else {
                            document.getElementById('initial-message-card').classList.add('hidden');
                            document.getElementById('connection-status-card').classList.remove('hidden');
                            document.getElementById('start-mission-button').classList.add('hidden'); // Hide the button
                            document.getElementById('start-mission2-button').classList.add('hidden'); // Hide the new button

                            // Check for phases where an image is expected or displayed
                            if (data.phase === "2.2" || data.phase === "2.3" || data.phase === "M2.4.0" || data.phase === "M2.6.0" || data.phase === "M2.6.1") {
                                document.getElementById('captured-image-card').classList.remove('hidden');
                                var capturedImg = document.getElementById('captured-img');
                                // Force image reload using timestamp to bust cache
                                capturedImg.src = "/capture.jpg?" + new Date().getTime();

                                if (data.phase === "2.3") {
                                     if (imageDisplayStartTime === 0) {
                                             imageDisplayStartTime = Date.now();
                                             clearInterval(imageIntervalId);
                                             imageIntervalId = setInterval(updateImageCountdownMission1, 1000); // Use Mission 1 countdown
                                     }
                                     updateImageCountdownMission1();
                                } else if (data.phase === "M2.6.1") { // New condition for Mission 2 image display
                                     if (imageDisplayStartTime === 0) {
                                             imageDisplayStartTime = Date.now();
                                             clearInterval(imageIntervalId);
                                             imageIntervalId = setInterval(updateImageCountdownMission2, 1000); // Use Mission 2 countdown
                                     }
                                     updateImageCountdownMission2();
                                } else {
                                    clearInterval(imageIntervalId);
                                    imageIntervalStartTime = 0;
                                    document.getElementById('image-countdown').innerText = "";
                                }


                            } else {
                                document.getElementById('captured-image-card').classList.add('hidden');
                                clearInterval(imageIntervalId);
                                imageDisplayStartTime = 0;
                                document.getElementById('image-countdown').innerText = "";
                                document.getElementById('captured-img').src = "";
                            }
                        }
                    } else {
                        // If WiFi is disconnected according to ESP32
                        document.getElementById('initial-message-card').classList.add('hidden');
                        document.getElementById('connection-status-card').classList.remove('hidden');
                        document.getElementById('captured-image-card').classList.add('hidden');
                        clearInterval(imageIntervalId);
                        imageDisplayStartTime = 0;
                        document.getElementById('image-countdown').innerText = "";
                        document.getElementById('captured-img').src = "";
                        document.getElementById('wifi-status').innerText = "WiFi Status: DISCONNECTED";
                        document.getElementById('ip-address').innerText = "Local IP: ---";
                        document.getElementById('current-phase-display').innerText = "Attempting to reconnect..."; // More informative
                        // Keep the start button hidden until reconnected to avoid multiple presses
                        document.getElementById('start-mission-button').classList.add('hidden');
                        document.getElementById('start-mission2-button').classList.add('hidden');
                    }
                })
                .catch(error => {
                    console.error('Error fetching WiFi status (server unreachable):', error);
                    // This catches network errors when the server isn't responding at all
                    document.getElementById('wifi-status').innerText = "WiFi Status: Error (Server Unreachable)";
                    document.getElementById('ip-address').innerText = "Local IP: ---";
                    document.getElementById('current-phase-display').innerText = "Server Offline/Reconnecting...";
                    // Show a message that the server is likely offline
                    document.getElementById('initial-message-card').classList.add('hidden'); // Keep these hidden
                    document.getElementById('connection-status-card').classList.remove('hidden'); // Show status card
                    document.getElementById('captured-image-card').classList.add('hidden');
                    clearInterval(imageIntervalId);
                    imageDisplayStartTime = 0;
                    document.getElementById('image-countdown').innerText = "";
                    document.getElementById('captured-img').src = "";

                    // If server is unreachable, we probably can't send start signal anyway.
                    // Best to hide the button until connection is re-established.
                    document.getElementById('start-mission-button').classList.add('hidden');
                    document.getElementById('start-mission2-button').classList.add('hidden');
                });
        }

        function updateImageCountdownMission1() {
            var elapsed = Date.now() - imageDisplayStartTime;
            var remaining = Math.max(0, mission1ImageDisplayDuration - elapsed);
            var seconds = Math.ceil(remaining / 1000);
            document.getElementById('image-countdown').innerText = "Displaying for " + seconds + " more seconds (Mission 1)...";

            if (remaining <= 0) {
                clearInterval(imageIntervalId);
                imageIntervalId = null;
                document.getElementById('image-countdown').innerText = "Mission 1 Complete! Image displayed. Returning to start...";
            }
        }

        function updateImageCountdownMission2() { // New countdown function for Mission 2
            var elapsed = Date.now() - imageDisplayStartTime;
            var remaining = Math.max(0, mission2ImageDisplayDuration - elapsed);
            var seconds = Math.ceil(remaining / 1000);
            document.getElementById('image-countdown').innerText = "Displaying for " + seconds + " more seconds (Mission 2)...";

            if (remaining <= 0) {
                clearInterval(imageIntervalId);
                imageIntervalId = null;
                document.getElementById('image-countdown').innerText = "Mission 2 Complete! Image displayed. Returning to start...";
            }
        }

        // Start checking WiFi status immediately
        wifiCheckInterval = setInterval(updateWifiStatus, 1500); // Check every 1.5 seconds
        updateWifiStatus(); // Initial call to update UI state immediately
    </script>
</body>
</html>
)rawliteral";


static httpd_uri_t captured_image_uri = {
    .uri         = "/capture.jpg",
    .method      = HTTP_GET,
    .handler     = captured_image_handler,
    .user_ctx    = NULL
};

static httpd_uri_t start_mission_uri = {
    .uri         = "/start_mission",
    .method      = HTTP_POST,
    .handler     = start_mission_handler,
    .user_ctx    = NULL
};

static httpd_uri_t start_mission2_uri = { // New URI for Mission #2
    .uri         = "/start_mission2",
    .method      = HTTP_POST,
    .handler     = start_mission2_handler,
    .user_ctx    = NULL
};


static httpd_uri_t wifi_status_uri = {
    .uri         = "/wifi_status",
    .method      = HTTP_GET,
    .handler     = [](httpd_req_t *req){
        String status = "DISCONNECTED";
        String ip = "0.0.0.0";
        String current_mission_phase_str;

        if (WiFi.status() == WL_CONNECTED) {
            status = "CONNECTED";
            ip = WiFi.localIP().toString();
        }

        switch (currentPhase) {
            case PHASE_0_0_JOIN_HOTSPOT: current_mission_phase_str = "0.0"; break;
            case PHASE_1_0_INITIAL_MOTOR_TEST: current_mission_phase_str = "1.0"; break;
            case PHASE_1_1_MOTOR_RAMP_UP: current_mission_phase_str = "1.1"; break;
            case PHASE_1_2_ULTRASONIC_MOTOR_CONTROL: current_mission_phase_str = "1.2"; break;
            case PHASE_2_1_ALL_MOTORS_50_PERCENT: current_mission_phase_str = "2.1"; break;
            case PHASE_2_2_RECONNECT_CAPTURE_DISPLAY: current_mission_phase_str = "2.2"; break;
            case PHASE_2_3_MISSION_COMPLETE: current_mission_phase_str = "2.3"; break;
            // Add new Mission #2 cases
            case PHASE_M2_0_0_INIT: current_mission_phase_str = "M2.0.0"; break;
            case PHASE_M2_1_0_DISCONNECT_WIFI: current_mission_phase_str = "M2.1.0"; break;
            case PHASE_M2_2_0_WAIT_20S_A: current_mission_phase_str = "M2.2.0"; break;
            case PHASE_M2_3_0_MOTOR_RAMP_UP_COMPLEX: current_mission_phase_str = "M2.3.0"; break;
            case PHASE_M2_4_0_WAIT_20S_B_CAPTURE_IMAGE: current_mission_phase_str = "M2.4.0"; break;
            case PHASE_M2_5_0_MOTOR_RAMP_UP_V2: current_mission_phase_str = "M2.5.0"; break;
            case PHASE_M2_6_0_TURN_OFF_MOTORS_UPLOAD_IMAGE: current_mission_phase_str = "M2.6.0"; break;
            case PHASE_M2_6_1_DISPLAY_IMAGE: current_mission_phase_str = "M2.6.1"; break; // New phase
            default: current_mission_phase_str = "UNKNOWN"; break;
        }

        char json_response[128];
        snprintf(json_response, sizeof(json_response),
                 "{\"status\":\"%s\", \"ip\":\"%s\", \"phase\":\"%s\"}",
                 status.c_str(), ip.c_str(), current_mission_phase_str.c_str());

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    },
    .user_ctx    = NULL
};


static httpd_uri_t root_uri = {
    .uri         = "/",
    .method      = HTTP_GET,
    .handler     = [](httpd_req_t *req){
        httpd_resp_set_type(req, "text/html");
        return httpd_resp_send(req, STATIC_PAGE, HTTPD_RESP_USE_STRLEN);
    },
    .user_ctx    = NULL
};

extern "C" void startCameraServer(){
    if (camera_httpd != NULL) {
        Serial.println("Web server already running. Not restarting.");
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 16;
    config.max_resp_headers = 8;
    config.uri_match_fn = httpd_uri_match_wildcard;

    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &root_uri);
        httpd_register_uri_handler(camera_httpd, &captured_image_uri);
        httpd_register_uri_handler(camera_httpd, &wifi_status_uri);
        httpd_register_uri_handler(camera_httpd, &start_mission_uri);
        httpd_register_uri_handler(camera_httpd, &start_mission2_uri); // Register new URI
        Serial.println("Web server handlers registered.");
    } else {
        Serial.println("Failed to start web server!");
    }

    if (xFrameBufferMutex == NULL) {
        xFrameBufferMutex = xSemaphoreCreateMutex();
        if (xFrameBufferMutex != NULL) {
            Serial.println("xFrameBufferMutex created (if not already).");
        } else {
            Serial.println("Failed to create xFrameBufferMutex!");
        }
    }
}

extern "C" void stopCameraServer() {
    if (camera_httpd) {
        Serial.println("Stopping web server.");
        httpd_stop(camera_httpd);
        camera_httpd = NULL; // Crucial: Clear the handle after stopping
    }
}
