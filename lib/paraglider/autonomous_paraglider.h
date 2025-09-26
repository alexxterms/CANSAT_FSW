#ifndef AUTONOMOUS_PARAGLIDER_H
#define AUTONOMOUS_PARAGLIDER_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "driver/ledc.h"
#include "esp_err.h"

// Servo configuration constants
#define SERVO_MIN_PULSEWIDTH_US    500     // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US    2500    // Maximum pulse width in microseconds
#define SERVO_NEUTRAL_PULSEWIDTH_US 1500   // Neutral position pulse width
#define SERVO_FREQUENCY_HZ         50      // Servo frequency (50Hz = 20ms period)

// Navigation constants
#define EARTH_RADIUS_M             6371000.0  // Earth radius in meters
#define DEG_TO_RAD                 (M_PI / 180.0)
#define RAD_TO_DEG                 (180.0 / M_PI)
#define HEADING_TOLERANCE_DEG      5.0     // Degrees tolerance for heading alignment
#define SMALL_PULL_US              200     // Microseconds to pull servo for small correction
#define MEDIUM_PULL_US             400     // Microseconds for medium correction
#define LARGE_PULL_US              600     // Microseconds for large correction

// Loitering parameters
#define LOITER_RADIUS_M            100.0   // Default loiter circle radius in meters
#define LOITER_DIRECTION_CW        1       // Clockwise loitering
#define LOITER_DIRECTION_CCW       -1      // Counter-clockwise loitering
#define LANDING_ALTITUDE_THRESHOLD_M 50.0  // Altitude below which to attempt landing
#define FINAL_APPROACH_RADIUS_M    20.0    // Final approach radius for landing

// GPIO pins for servos
#define LEFT_SERVO_PIN             GPIO_NUM_12
#define RIGHT_SERVO_PIN            GPIO_NUM_13

// LEDC channels for PWM
#define LEFT_SERVO_CHANNEL         LEDC_CHANNEL_0
#define RIGHT_SERVO_CHANNEL        LEDC_CHANNEL_1
#define LEDC_TIMER                 LEDC_TIMER_0
#define LEDC_MODE                  LEDC_LOW_SPEED_MODE

// GPS coordinate structure
typedef struct {
    double latitude;   // Latitude in degrees
    double longitude;  // Longitude in degrees
    double altitude;   // Altitude in meters (optional for 2D navigation)
} gps_coordinate_t;

// Navigation waypoint
typedef struct {
    gps_coordinate_t position;
    float arrival_radius_m;  // Radius in meters to consider waypoint reached
    char name[32];          // Optional waypoint name
} waypoint_t;

// Paraglider control parameters
typedef struct {
    uint16_t left_servo_us;    // Left servo pulse width in microseconds
    uint16_t right_servo_us;   // Right servo pulse width in microseconds
    float heading_error_deg;   // Current heading error in degrees
    float distance_to_target_m; // Distance to target in meters
    float bearing_to_target_deg; // Bearing to target in degrees
} paraglider_control_t;

// Navigation modes
typedef enum {
    NAV_MODE_WAYPOINT,      // Navigate to waypoints/target
    NAV_MODE_LOITER,        // Circle around target area
    NAV_MODE_LANDING,       // Final approach and landing
    NAV_MODE_LANDED         // Successfully landed
} nav_mode_t;

// Flight states (matching main.c)
typedef enum {
    FLIGHT_PRE_LAUNCH,
    FLIGHT_ASCENT,
    FLIGHT_APOGEE,
    FLIGHT_DESCENT,
    FLIGHT_LANDING
} flight_state_t;

// Loitering parameters
typedef struct {
    gps_coordinate_t center;    // Center point for loitering
    float radius_m;             // Loiter circle radius
    int8_t direction;           // 1 for CW, -1 for CCW
    float entry_heading_deg;    // Heading when entering loiter
    bool active;                // Is loitering currently active
} loiter_params_t;

// Navigation state
typedef struct {
    gps_coordinate_t current_position;
    gps_coordinate_t target_position;
    float current_heading_deg;
    float current_altitude_m;   // Current altitude for landing decisions
    waypoint_t waypoints[10];  // Support for up to 10 waypoints
    uint8_t current_waypoint_index;
    uint8_t total_waypoints;
    bool navigation_active;
    bool waypoint_reached;
    nav_mode_t nav_mode;        // Current navigation mode
    flight_state_t flight_state; // Current flight state
    loiter_params_t loiter;     // Loitering parameters
    paraglider_control_t control;
} paraglider_nav_t;

// Function declarations

/**
 * Initialize the autonomous paraglider system
 * @return ESP_OK on success
 */
esp_err_t paraglider_init(void);

/**
 * Set the target destination coordinates
 * @param lat Target latitude in degrees
 * @param lon Target longitude in degrees
 * @return ESP_OK on success
 */
esp_err_t paraglider_set_target(double lat, double lon);

/**
 * Add a waypoint to the navigation route
 * @param waypoint Waypoint structure with position and arrival radius
 * @return ESP_OK on success, ESP_ERR_NO_MEM if waypoint list is full
 */
esp_err_t paraglider_add_waypoint(const waypoint_t *waypoint);

/**
 * Update current position, heading, and altitude from GPS and IMU
 * @param lat Current latitude in degrees
 * @param lon Current longitude in degrees
 * @param heading Current heading in degrees (0-360)
 * @param altitude Current altitude in meters
 * @return ESP_OK on success
 */
esp_err_t paraglider_update_position(double lat, double lon, float heading, float altitude);

/**
 * Main navigation update function - call this regularly in your main loop
 * @return ESP_OK on success
 */
esp_err_t paraglider_navigation_update(void);

/**
 * Calculate bearing between two GPS coordinates
 * @param from_lat Starting latitude in degrees
 * @param from_lon Starting longitude in degrees
 * @param to_lat Destination latitude in degrees
 * @param to_lon Destination longitude in degrees
 * @return Bearing in degrees (0-360)
 */
float compute_bearing(double from_lat, double from_lon, double to_lat, double to_lon);

/**
 * Calculate distance between two GPS coordinates using Haversine formula
 * @param from_lat Starting latitude in degrees
 * @param from_lon Starting longitude in degrees
 * @param to_lat Destination latitude in degrees
 * @param to_lon Destination longitude in degrees
 * @return Distance in meters
 */
float compute_distance(double from_lat, double from_lon, double to_lat, double to_lon);

/**
 * Wrap angle to -180 to +180 degree range
 * @param angle Angle in degrees
 * @return Wrapped angle in degrees
 */
float wrap_angle(float angle);

/**
 * Set servo positions manually (for testing)
 * @param left_us Left servo pulse width in microseconds
 * @param right_us Right servo pulse width in microseconds
 * @return ESP_OK on success
 */
esp_err_t paraglider_set_servos(uint16_t left_us, uint16_t right_us);

/**
 * Set servos to neutral position
 * @return ESP_OK on success
 */
esp_err_t paraglider_set_neutral(void);

/**
 * Start autonomous navigation to the set target
 * @return ESP_OK on success
 */
esp_err_t paraglider_start_navigation(void);

/**
 * Stop autonomous navigation and set servos to neutral
 * @return ESP_OK on success
 */
esp_err_t paraglider_stop_navigation(void);

/**
 * Get current navigation status
 * @param nav_status Pointer to navigation structure to fill
 * @return ESP_OK on success
 */
esp_err_t paraglider_get_status(paraglider_nav_t *nav_status);

/**
 * Check if target/waypoint has been reached
 * @return true if reached, false otherwise
 */
bool paraglider_is_target_reached(void);

/**
 * Clear all waypoints and reset navigation
 * @return ESP_OK on success
 */
esp_err_t paraglider_clear_waypoints(void);

/**
 * Update flight state (from main flight software)
 * @param state Current flight state
 * @return ESP_OK on success
 */
esp_err_t paraglider_update_flight_state(flight_state_t state);

/**
 * Configure loitering parameters
 * @param center_lat Loiter center latitude
 * @param center_lon Loiter center longitude
 * @param radius_m Loiter radius in meters
 * @param direction 1 for clockwise, -1 for counter-clockwise
 * @return ESP_OK on success
 */
esp_err_t paraglider_set_loiter_params(double center_lat, double center_lon, float radius_m, int8_t direction);

/**
 * Start loitering mode at current target
 * @return ESP_OK on success
 */
esp_err_t paraglider_start_loiter(void);

/**
 * Stop loitering and resume normal navigation
 * @return ESP_OK on success
 */
esp_err_t paraglider_stop_loiter(void);

/**
 * Get current navigation mode
 * @return Current navigation mode
 */
nav_mode_t paraglider_get_nav_mode(void);

#endif // AUTONOMOUS_PARAGLIDER_H