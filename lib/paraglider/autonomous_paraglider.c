#include "autonomous_paraglider.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "PARAGLIDER";

// Global navigation state
static paraglider_nav_t g_nav_state = {0};
static bool g_initialized = false;

// Servo PWM duty calculation helper
static uint32_t pulse_width_to_duty(uint16_t pulse_width_us) {
    // Calculate duty cycle for LEDC
    // duty = (pulse_width_us * (2^resolution - 1)) / (1000000 / frequency)
    // For 13-bit resolution and 50Hz: period = 20000us
    return (pulse_width_us * 8191) / 20000;
}

esp_err_t paraglider_init(void) {
    esp_err_t ret = ESP_OK;
    
    // Initialize LEDC timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = SERVO_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure left servo channel
    ledc_channel_config_t left_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEFT_SERVO_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEFT_SERVO_PIN,
        .duty = pulse_width_to_duty(SERVO_NEUTRAL_PULSEWIDTH_US),
        .hpoint = 0
    };
    ret = ledc_channel_config(&left_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure left servo channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure right servo channel
    ledc_channel_config_t right_channel = {
        .speed_mode = LEDC_MODE,
        .channel = RIGHT_SERVO_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = RIGHT_SERVO_PIN,
        .duty = pulse_width_to_duty(SERVO_NEUTRAL_PULSEWIDTH_US),
        .hpoint = 0
    };
    ret = ledc_channel_config(&right_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure right servo channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize navigation state
    memset(&g_nav_state, 0, sizeof(g_nav_state));
    g_nav_state.control.left_servo_us = SERVO_NEUTRAL_PULSEWIDTH_US;
    g_nav_state.control.right_servo_us = SERVO_NEUTRAL_PULSEWIDTH_US;
    g_nav_state.navigation_active = false;
    
    g_initialized = true;
    ESP_LOGI(TAG, "Paraglider system initialized successfully");
    
    return ESP_OK;
}

esp_err_t paraglider_set_target(double lat, double lon) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_nav_state.target_position.latitude = lat;
    g_nav_state.target_position.longitude = lon;
    g_nav_state.target_position.altitude = 0.0; // Not used for 2D navigation
    
    ESP_LOGI(TAG, "Target set to: %.6f, %.6f", lat, lon);
    return ESP_OK;
}

esp_err_t paraglider_add_waypoint(const waypoint_t *waypoint) {
    if (!g_initialized || !waypoint) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_nav_state.total_waypoints >= 10) {
        ESP_LOGE(TAG, "Maximum waypoints reached");
        return ESP_ERR_NO_MEM;
    }
    
    memcpy(&g_nav_state.waypoints[g_nav_state.total_waypoints], waypoint, sizeof(waypoint_t));
    g_nav_state.total_waypoints++;
    
    ESP_LOGI(TAG, "Waypoint %d added: %s (%.6f, %.6f) radius: %.1fm", 
             g_nav_state.total_waypoints - 1, waypoint->name,
             waypoint->position.latitude, waypoint->position.longitude,
             waypoint->arrival_radius_m);
    
    return ESP_OK;
}

esp_err_t paraglider_update_position(double lat, double lon, float heading) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_nav_state.current_position.latitude = lat;
    g_nav_state.current_position.longitude = lon;
    g_nav_state.current_heading_deg = heading;
    
    return ESP_OK;
}

esp_err_t paraglider_update_position(double lat, double lon, float heading, float altitude) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_nav_state.current_position.latitude = lat;
    g_nav_state.current_position.longitude = lon;
    g_nav_state.current_heading_deg = heading;
    g_nav_state.current_altitude_m = altitude;
    
    return ESP_OK;
}

esp_err_t paraglider_update_flight_state(flight_state_t state) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_nav_state.flight_state = state;
    return ESP_OK;
}

esp_err_t paraglider_set_loiter_params(double center_lat, double center_lon, float radius_m, int8_t direction) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_nav_state.loiter.center.latitude = center_lat;
    g_nav_state.loiter.center.longitude = center_lon;
    g_nav_state.loiter.radius_m = radius_m;
    g_nav_state.loiter.direction = direction;
    
    ESP_LOGI(TAG, "Loiter params set: Center(%.6f, %.6f), Radius:%.0fm, Direction:%s", 
             center_lat, center_lon, radius_m, 
             (direction == LOITER_DIRECTION_CW) ? "CW" : "CCW");
    
    return ESP_OK;
}

esp_err_t paraglider_start_loiter(void) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Set loiter center to current target if not already set
    if (g_nav_state.loiter.center.latitude == 0.0 && g_nav_state.loiter.center.longitude == 0.0) {
        g_nav_state.loiter.center = g_nav_state.target_position;
        g_nav_state.loiter.radius_m = LOITER_RADIUS_M;
        g_nav_state.loiter.direction = LOITER_DIRECTION_CW;
    }
    
    g_nav_state.loiter.entry_heading_deg = g_nav_state.current_heading_deg;
    g_nav_state.loiter.active = true;
    g_nav_state.nav_mode = NAV_MODE_LOITER;
    
    ESP_LOGI(TAG, "Loitering started at (%.6f, %.6f) with radius %.0fm", 
             g_nav_state.loiter.center.latitude, 
             g_nav_state.loiter.center.longitude,
             g_nav_state.loiter.radius_m);
    
    return ESP_OK;
}

esp_err_t paraglider_stop_loiter(void) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_nav_state.loiter.active = false;
    g_nav_state.nav_mode = NAV_MODE_WAYPOINT;
    
    ESP_LOGI(TAG, "Loitering stopped");
    return ESP_OK;
}

nav_mode_t paraglider_get_nav_mode(void) {
    return g_nav_state.nav_mode;
}

// Helper function to calculate loiter waypoint
static gps_coordinate_t calculate_loiter_waypoint(const gps_coordinate_t *center, 
                                                  const gps_coordinate_t *current_pos,
                                                  float radius_m, 
                                                  int8_t direction) {
    // Calculate current bearing from center to aircraft
    float bearing_to_aircraft = compute_bearing(center->latitude, center->longitude,
                                               current_pos->latitude, current_pos->longitude);
    
    // Add 90 degrees for tangent point (adjust for direction)
    float tangent_bearing = bearing_to_aircraft + (90.0f * direction);
    if (tangent_bearing < 0) tangent_bearing += 360.0f;
    if (tangent_bearing >= 360.0f) tangent_bearing -= 360.0f;
    
    // Calculate tangent point on circle
    gps_coordinate_t waypoint;
    
    // Convert to radians for calculation
    double center_lat_rad = center->latitude * DEG_TO_RAD;
    double center_lon_rad = center->longitude * DEG_TO_RAD;
    double bearing_rad = tangent_bearing * DEG_TO_RAD;
    
    // Calculate new position using spherical geometry
    double angular_distance = radius_m / EARTH_RADIUS_M;
    
    double new_lat_rad = asin(sin(center_lat_rad) * cos(angular_distance) + 
                             cos(center_lat_rad) * sin(angular_distance) * cos(bearing_rad));
    
    double new_lon_rad = center_lon_rad + 
                        atan2(sin(bearing_rad) * sin(angular_distance) * cos(center_lat_rad),
                             cos(angular_distance) - sin(center_lat_rad) * sin(new_lat_rad));
    
    waypoint.latitude = new_lat_rad * RAD_TO_DEG;
    waypoint.longitude = new_lon_rad * RAD_TO_DEG;
    waypoint.altitude = 0.0;
    
    return waypoint;
}

float compute_bearing(double from_lat, double from_lon, double to_lat, double to_lon) {
    double lat1_rad = from_lat * DEG_TO_RAD;
    double lat2_rad = to_lat * DEG_TO_RAD;
    double delta_lon_rad = (to_lon - from_lon) * DEG_TO_RAD;
    
    double y = sin(delta_lon_rad) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon_rad);
    
    double bearing_rad = atan2(y, x);
    double bearing_deg = bearing_rad * RAD_TO_DEG;
    
    // Normalize to 0-360 degrees
    if (bearing_deg < 0) {
        bearing_deg += 360.0;
    }
    
    return (float)bearing_deg;
}

float compute_distance(double from_lat, double from_lon, double to_lat, double to_lon) {
    double lat1_rad = from_lat * DEG_TO_RAD;
    double lat2_rad = to_lat * DEG_TO_RAD;
    double delta_lat_rad = (to_lat - from_lat) * DEG_TO_RAD;
    double delta_lon_rad = (to_lon - from_lon) * DEG_TO_RAD;
    
    double a = sin(delta_lat_rad/2) * sin(delta_lat_rad/2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(delta_lon_rad/2) * sin(delta_lon_rad/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return (float)(EARTH_RADIUS_M * c);
}

float wrap_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

esp_err_t paraglider_set_servos(uint16_t left_us, uint16_t right_us) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp values to safe range
    if (left_us < SERVO_MIN_PULSEWIDTH_US) left_us = SERVO_MIN_PULSEWIDTH_US;
    if (left_us > SERVO_MAX_PULSEWIDTH_US) left_us = SERVO_MAX_PULSEWIDTH_US;
    if (right_us < SERVO_MIN_PULSEWIDTH_US) right_us = SERVO_MIN_PULSEWIDTH_US;
    if (right_us > SERVO_MAX_PULSEWIDTH_US) right_us = SERVO_MAX_PULSEWIDTH_US;
    
    // Set left servo
    esp_err_t ret = ledc_set_duty(LEDC_MODE, LEFT_SERVO_CHANNEL, pulse_width_to_duty(left_us));
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(LEDC_MODE, LEFT_SERVO_CHANNEL);
    if (ret != ESP_OK) return ret;
    
    // Set right servo
    ret = ledc_set_duty(LEDC_MODE, RIGHT_SERVO_CHANNEL, pulse_width_to_duty(right_us));
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(LEDC_MODE, RIGHT_SERVO_CHANNEL);
    if (ret != ESP_OK) return ret;
    
    // Update control state
    g_nav_state.control.left_servo_us = left_us;
    g_nav_state.control.right_servo_us = right_us;
    
    return ESP_OK;
}

esp_err_t paraglider_set_neutral(void) {
    return paraglider_set_servos(SERVO_NEUTRAL_PULSEWIDTH_US, SERVO_NEUTRAL_PULSEWIDTH_US);
}

esp_err_t paraglider_navigation_update(void) {
    if (!g_initialized || !g_nav_state.navigation_active) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check flight state and altitude for mode transitions
    if (g_nav_state.flight_state == FLIGHT_DESCENT && 
        g_nav_state.current_altitude_m < LANDING_ALTITUDE_THRESHOLD_M &&
        g_nav_state.nav_mode == NAV_MODE_LOITER) {
        // Switch to landing mode
        g_nav_state.nav_mode = NAV_MODE_LANDING;
        ESP_LOGI(TAG, "Switching to landing mode - altitude: %.1fm", g_nav_state.current_altitude_m);
    }
    
    // Determine current target based on navigation mode
    gps_coordinate_t *current_target;
    float arrival_radius = 50.0f; // Default 50m radius
    
    switch (g_nav_state.nav_mode) {
        case NAV_MODE_WAYPOINT:
            // Normal waypoint navigation
            if (g_nav_state.total_waypoints > 0 && 
                g_nav_state.current_waypoint_index < g_nav_state.total_waypoints) {
                current_target = &g_nav_state.waypoints[g_nav_state.current_waypoint_index].position;
                arrival_radius = g_nav_state.waypoints[g_nav_state.current_waypoint_index].arrival_radius_m;
            } else {
                current_target = &g_nav_state.target_position;
            }
            break;
            
        case NAV_MODE_LOITER: {
            // Calculate loiter waypoint
            static gps_coordinate_t loiter_waypoint;
            loiter_waypoint = calculate_loiter_waypoint(&g_nav_state.loiter.center,
                                                       &g_nav_state.current_position,
                                                       g_nav_state.loiter.radius_m,
                                                       g_nav_state.loiter.direction);
            current_target = &loiter_waypoint;
            arrival_radius = 30.0f; // Smaller radius for loiter waypoints
            break;
        }
        
        case NAV_MODE_LANDING:
            // Navigate directly to target for landing
            current_target = &g_nav_state.target_position;
            arrival_radius = FINAL_APPROACH_RADIUS_M;
            break;
            
        case NAV_MODE_LANDED:
            // Already landed, maintain position
            paraglider_set_neutral();
            return ESP_OK;
            
        default:
            current_target = &g_nav_state.target_position;
            break;
    }
    
    // Calculate bearing and distance to current target
    float bearing_to_target = compute_bearing(
        g_nav_state.current_position.latitude,
        g_nav_state.current_position.longitude,
        current_target->latitude,
        current_target->longitude
    );
    
    float distance_to_target = compute_distance(
        g_nav_state.current_position.latitude,
        g_nav_state.current_position.longitude,
        current_target->latitude,
        current_target->longitude
    );
    
    // Update control state
    g_nav_state.control.bearing_to_target_deg = bearing_to_target;
    g_nav_state.control.distance_to_target_m = distance_to_target;
    
    // Handle waypoint/target arrival based on mode
    if (distance_to_target <= arrival_radius) {
        switch (g_nav_state.nav_mode) {
            case NAV_MODE_WAYPOINT:
                if (g_nav_state.total_waypoints > 0 && 
                    g_nav_state.current_waypoint_index < g_nav_state.total_waypoints) {
                    // Waypoint reached, move to next
                    ESP_LOGI(TAG, "Waypoint %d reached", g_nav_state.current_waypoint_index);
                    g_nav_state.current_waypoint_index++;
                    
                    if (g_nav_state.current_waypoint_index >= g_nav_state.total_waypoints) {
                        // All waypoints completed, check if should loiter or land
                        float distance_to_final_target = compute_distance(
                            g_nav_state.current_position.latitude,
                            g_nav_state.current_position.longitude,
                            g_nav_state.target_position.latitude,
                            g_nav_state.target_position.longitude
                        );
                        
                        if (distance_to_final_target <= 100.0f && 
                            g_nav_state.flight_state == FLIGHT_DESCENT &&
                            g_nav_state.current_altitude_m > LANDING_ALTITUDE_THRESHOLD_M) {
                            // Start loitering - we're at target but still too high
                            paraglider_start_loiter();
                        } else {
                            ESP_LOGI(TAG, "All waypoints completed, navigating to final target");
                        }
                    }
                } else {
                    // Final target reached
                    float distance_to_final_target = compute_distance(
                        g_nav_state.current_position.latitude,
                        g_nav_state.current_position.longitude,
                        g_nav_state.target_position.latitude,
                        g_nav_state.target_position.longitude
                    );
                    
                    if (distance_to_final_target <= 100.0f && 
                        g_nav_state.flight_state == FLIGHT_DESCENT &&
                        g_nav_state.current_altitude_m > LANDING_ALTITUDE_THRESHOLD_M) {
                        // Start loitering - we're at target but still too high
                        paraglider_start_loiter();
                    } else if (g_nav_state.current_altitude_m <= LANDING_ALTITUDE_THRESHOLD_M) {
                        // Low enough to land
                        g_nav_state.nav_mode = NAV_MODE_LANDED;
                        g_nav_state.waypoint_reached = true;
                        ESP_LOGI(TAG, "Landing completed!");
                        paraglider_stop_navigation();
                        return ESP_OK;
                    }
                }
                break;
                
            case NAV_MODE_LOITER:
                // Continue loitering - waypoints are continuously generated
                ESP_LOGD(TAG, "Loiter waypoint reached, continuing circle");
                break;
                
            case NAV_MODE_LANDING:
                // Landing target reached
                g_nav_state.nav_mode = NAV_MODE_LANDED;
                g_nav_state.waypoint_reached = true;
                ESP_LOGI(TAG, "Landing target reached!");
                paraglider_stop_navigation();
                return ESP_OK;
                
            default:
                break;
        }
    }
    
    // Calculate heading error - your original pseudocode implementation
    float heading_error = wrap_angle(bearing_to_target - g_nav_state.current_heading_deg);
    g_nav_state.control.heading_error_deg = heading_error;
    
    // Determine servo corrections based on heading error
    uint16_t left_servo = SERVO_NEUTRAL_PULSEWIDTH_US;
    uint16_t right_servo = SERVO_NEUTRAL_PULSEWIDTH_US;
    
    // Adaptive correction based on error magnitude and navigation mode
    uint16_t correction_amount = SMALL_PULL_US;
    
    // Use more aggressive corrections during loitering for tighter turns
    if (g_nav_state.nav_mode == NAV_MODE_LOITER) {
        if (fabs(heading_error) > 20.0f) {
            correction_amount = LARGE_PULL_US;
        } else if (fabs(heading_error) > 10.0f) {
            correction_amount = MEDIUM_PULL_US;
        }
    } else {
        // Normal navigation corrections
        if (fabs(heading_error) > 30.0f) {
            correction_amount = LARGE_PULL_US;
        } else if (fabs(heading_error) > 15.0f) {
            correction_amount = MEDIUM_PULL_US;
        }
    }
    
    if (heading_error > HEADING_TOLERANCE_DEG) {
        // Need to turn right
        right_servo = SERVO_NEUTRAL_PULSEWIDTH_US + correction_amount;
        left_servo = SERVO_NEUTRAL_PULSEWIDTH_US;
        ESP_LOGD(TAG, "Turning RIGHT: error=%.1f°, correction=%dus", heading_error, correction_amount);
    } else if (heading_error < -HEADING_TOLERANCE_DEG) {
        // Need to turn left
        left_servo = SERVO_NEUTRAL_PULSEWIDTH_US + correction_amount;
        right_servo = SERVO_NEUTRAL_PULSEWIDTH_US;
        ESP_LOGD(TAG, "Turning LEFT: error=%.1f°, correction=%dus", heading_error, correction_amount);
    } else {
        // Aligned with target
        left_servo = SERVO_NEUTRAL_PULSEWIDTH_US;
        right_servo = SERVO_NEUTRAL_PULSEWIDTH_US;
        ESP_LOGD(TAG, "ON COURSE: error=%.1f°", heading_error);
    }
    
    // Apply servo corrections
    esp_err_t ret = paraglider_set_servos(left_servo, right_servo);
    
    // Log navigation status periodically
    static uint32_t log_counter = 0;
    if (++log_counter % 10 == 0) { // Log every 10 updates
        const char* mode_str[] = {"WAYPOINT", "LOITER", "LANDING", "LANDED"};
        ESP_LOGI(TAG, "Nav[%s]: Pos(%.6f,%.6f) Alt:%.1fm Hdg:%.1f° Target(%.6f,%.6f) Bearing:%.1f° Dist:%.0fm Error:%.1f°",
                 mode_str[g_nav_state.nav_mode],
                 g_nav_state.current_position.latitude, g_nav_state.current_position.longitude,
                 g_nav_state.current_altitude_m, g_nav_state.current_heading_deg,
                 current_target->latitude, current_target->longitude,
                 bearing_to_target, distance_to_target, heading_error);
    }
    
    return ret;
}

esp_err_t paraglider_start_navigation(void) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_nav_state.navigation_active = true;
    g_nav_state.current_waypoint_index = 0;
    g_nav_state.waypoint_reached = false;
    g_nav_state.nav_mode = NAV_MODE_WAYPOINT;
    g_nav_state.flight_state = FLIGHT_PRE_LAUNCH;
    
    // Initialize loiter parameters with defaults
    if (g_nav_state.loiter.radius_m == 0.0f) {
        g_nav_state.loiter.radius_m = LOITER_RADIUS_M;
        g_nav_state.loiter.direction = LOITER_DIRECTION_CW;
    }
    
    ESP_LOGI(TAG, "Navigation started");
    return ESP_OK;
}

esp_err_t paraglider_stop_navigation(void) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    g_nav_state.navigation_active = false;
    esp_err_t ret = paraglider_set_neutral();
    
    ESP_LOGI(TAG, "Navigation stopped");
    return ret;
}

esp_err_t paraglider_get_status(paraglider_nav_t *nav_status) {
    if (!g_initialized || !nav_status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(nav_status, &g_nav_state, sizeof(paraglider_nav_t));
    return ESP_OK;
}

bool paraglider_is_target_reached(void) {
    return g_nav_state.waypoint_reached;
}

esp_err_t paraglider_clear_waypoints(void) {
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memset(g_nav_state.waypoints, 0, sizeof(g_nav_state.waypoints));
    g_nav_state.total_waypoints = 0;
    g_nav_state.current_waypoint_index = 0;
    g_nav_state.waypoint_reached = false;
    
    ESP_LOGI(TAG, "All waypoints cleared");
    return ESP_OK;
}