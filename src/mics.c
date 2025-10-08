#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "communications.h"
#include "gps.h"

// UART configuration
#define UART_MAIN UART_NUM_0
#define UART_TX_PIN 1
#define UART_RX_PIN 3
#define UART_BAUD_RATE 115200
#define UART_BUFFER_SIZE 1024

// Static values for identification packet
static const char* TEAM_ID = "CANSAT014";
static const uint32_t PACKET_ID = 1234;
static const uint8_t SOURCE_ID = 1;
static const uint8_t DEST_ID = 2;
static const uint32_t TIMESTAMP = 5415000;

// Function to send raw bytes to UART
static void send_to_uart(const uint8_t *data, size_t size) {
    uart_write_bytes(UART_MAIN, (const char*)data, size);
}

// Function to send identification packet
static void send_well_packet() {
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    uint8_t payload_buffer[MAX_PAYLOAD_SIZE];
    int payload_offset = 0;

    // Create identification sub-packet
    IdentificationPacket id_packet;
    memset(&id_packet, 0, sizeof(IdentificationPacket));
    id_packet.message_id = MSG_ID_IDENTIFICATION;
    id_packet.packet_id = PACKET_ID;
    strncpy(id_packet.team_id, TEAM_ID, TEAM_ID_MAX_LEN);
    id_packet.timestamp = TIMESTAMP;
    id_packet.source = SOURCE_ID;
    id_packet.destination = DEST_ID;

    uint8_t id_temp_buffer[64];
    size_t id_temp_size = build_identification_packet(id_temp_buffer, &id_packet);

    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    id_temp_buffer, id_temp_size);

    if (payload_offset == -1) {
        return;
    }

    // Create environmental sub-packet
    EnvironmentalPacket env_packet;
    memset(&env_packet, 0, sizeof(EnvironmentalPacket));
    env_packet.message_id = MSG_ID_ENVIRONMENTAL;
    env_packet.packet_id = 1234;
    env_packet.altitude = 10000;
    env_packet.temperature = 2550;
    env_packet.pressure = 10132;
    env_packet.humidity = 450;
    env_packet.voc_concentration = 150;
    env_packet.ethanol_concentration = 5;
    env_packet.h2_concentration = 1;
    env_packet.nh3_concentration = 0.5;
    env_packet.ch4_concentration = 2;
    env_packet.air_quality_index = 50;
  

    uint8_t env_temp_buffer[64];
    size_t env_temp_size = build_environmental_packet(env_temp_buffer, 
                                                      env_packet.altitude,
                                                      env_packet.temperature,
                                                      env_packet.pressure,
                                                      env_packet.humidity,
                                                      env_packet.voc_concentration,
                                                      env_packet.ethanol_concentration,
                                                      env_packet.h2_concentration,
                                                      env_packet.nh3_concentration,
                                                      env_packet.ch4_concentration,
                                                      env_packet.air_quality_index);

    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    env_temp_buffer, env_temp_size);

    if (payload_offset == -1) {
        return;
    }

    AccelerometerPacket acc_packet;
    memset(&acc_packet, 0, sizeof(AccelerometerPacket));
    acc_packet.message_id = MSG_ID_ACCEL;
    acc_packet.packet_id = 1234;
    acc_packet.x_position = 100; // Example scaled value
    acc_packet.y_position = -50; // Example scaled value
    acc_packet.z_position = 0;   // Example scaled value    

    uint8_t acc_temp_buffer[32];
    size_t acc_temp_size = build_accel_packet(acc_temp_buffer, 
                                              acc_packet.x_position,
                                              acc_packet.y_position,
                                              acc_packet.z_position);   
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    acc_temp_buffer, acc_temp_size);
    if (payload_offset == -1) {
        return;    
    }    
     
    GyroscopePacket gyro_packet;
    memset(&gyro_packet, 0, sizeof(GyroscopePacket));
    gyro_packet.message_id = MSG_ID_GYRO;
    gyro_packet.packet_id = 1234;
    gyro_packet.x_rate = 250; // Example scaled value
    gyro_packet.y_rate = 0;   // Example scaled value
    gyro_packet.z_rate = -250; // Example scaled value  

    uint8_t gyro_temp_buffer[32];
    size_t gyro_temp_size = build_gyro_packet(gyro_temp_buffer, 
                                              gyro_packet.x_rate,
                                              gyro_packet.y_rate,
                                              gyro_packet.z_rate);                       
    
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    gyro_temp_buffer, gyro_temp_size);

    if (payload_offset == -1) {
        return;    
    }      
    
    MagnetometerPacket mag_packet;
    memset(&mag_packet, 0, sizeof(MagnetometerPacket));
    mag_packet.message_id = MSG_ID_MAGNETOMETER;
    mag_packet.packet_id = 1234;
    mag_packet.x_field = 300; // Example scaled value
    mag_packet.y_field = 0;   // Example scaled value
    mag_packet.z_field = -300; // Example scaled value  

    uint8_t mag_temp_buffer[32];
    size_t mag_temp_size = build_magnetometer_packet(mag_temp_buffer, 
                                            mag_packet.x_field,
                                            mag_packet.y_field,
                                            mag_packet.z_field);
    
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    mag_temp_buffer, mag_temp_size);
    
    if (payload_offset == -1) {
        return;    
    }     
    
    GGAPacket gga_packet;
    memset(&gga_packet, 0, sizeof(GGAPacket));
    gga_packet.message_id = MSG_ID_GGA;
    gga_packet.packet_id = 1234;
    gga_packet.utc_time[0] = 12; // HH
    gga_packet.utc_time[1] = 34; // MM
    gga_packet.utc_time[2] = 56; // SS.ss
    gga_packet.latitude = 37.7749; // Example latitude
    gga_packet.latitude_direction = 1; // North
    gga_packet.longitude = -122.4194; // Example longitude
    gga_packet.longitude_direction = 0; // West
    gga_packet.fix_status = 2; // RTK Fix
    gga_packet.hdop = 50; // Example HDOP scaled
    gga_packet.satellites_used = 10; // Example number of satellites
    gga_packet.altitude = 10000; // Example altitude scaled
  

    uint8_t gga_temp_buffer[64];
    size_t gga_temp_size = build_gga_packet(gga_temp_buffer,
                                            gga_packet.utc_time,
                                            gga_packet.latitude,
                                            gga_packet.latitude_direction,
                                            gga_packet.longitude,
                                            gga_packet.longitude_direction,
                                            gga_packet.fix_status,
                                            gga_packet.hdop,
                                            gga_packet.satellites_used,
                                            gga_packet.altitude
                                            );
    
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    gga_temp_buffer, gga_temp_size);
    if (payload_offset == -1) {
        return;    
    }
    satellite_info_t satellite_data;
    // Populate satellite_data with actual satellite information
    satellite_data.prn = 1;
    satellite_data.elevation = 45;
    satellite_data.azimuth = 180;
    satellite_data.snr = 30;
    
    GSVPacket gsv_packet;
    memset(&gsv_packet, 0, sizeof(GSVPacket));
    gsv_packet.message_id = MSG_ID_GSV;
    gsv_packet.packet_id = 1234;
    gsv_packet.satellites_in_view = 12; // Example number of satellites
    gsv_packet.snr = 350; // Example SNR scaled
    gsv_packet.signal_id = 1; // Example signal ID  
    uint8_t gsv_temp_buffer[64];
    size_t gsv_temp_size = build_gsv_packet(gsv_temp_buffer,
                                            gsv_packet.satellites_in_view,
                                            &satellite_data,
                                            gsv_packet.signal_id
                                            );
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    gsv_temp_buffer, gsv_temp_size);
    if (payload_offset == -1) {
        return;    
    }

    BatteryPacket batt_packet;
    memset(&batt_packet, 0, sizeof(BatteryPacket));
    batt_packet.message_id = MSG_ID_BATTERY;
    batt_packet.packet_id = 1234;
    batt_packet.current_voltage = 3700; // Example voltage scaled 
    batt_packet.voltage_draw = 500;    // Example voltage draw scaled
    batt_packet.current_draw = 200;    // Example current draw scaled
    batt_packet.current_drawn = 1500;  // Example current drawn scaled

    uint8_t batt_temp_buffer[32];
    size_t batt_temp_size = build_battery_packet(batt_temp_buffer, 
                                                 batt_packet.current_voltage,
                                                 batt_packet.voltage_draw,
                                                 batt_packet.current_draw,
                                                 batt_packet.current_drawn);   

    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE, 
                                    batt_temp_buffer, batt_temp_size);
    if (payload_offset == -1) {
        return;    
    }

    SystemStatusPacket sys_packet;
    memset(&sys_packet, 0, sizeof(SystemStatusPacket));
    sys_packet.message_id = MSG_ID_SYSTEM_STATUS;
    sys_packet.packet_id = 1234;
    sys_packet.sensor_operational_status = 0x0F; // Example sensor status
    sys_packet.uptime = 3600000; // Example uptime in ms
    sys_packet.flight_software_mission = 2; // Example mission phase
    strncpy(sys_packet.reboot_reason, "Meow", sizeof(sys_packet.reboot_reason));
    sys_packet.system_time[0] = 12; // HH
    sys_packet.system_time[1] = 34; // MM
    sys_packet.system_time[2] = 56; // SS.ss
    uint8_t sys_temp_buffer[128];
    size_t sys_temp_size = build_system_status_packet(sys_temp_buffer,
                                                      sys_packet.sensor_operational_status,
                                                      sys_packet.uptime,
                                                      sys_packet.flight_software_mission,
                                                      sys_packet.reboot_reason,
                                                      sys_packet.system_time);
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    sys_temp_buffer, sys_temp_size);
    if (payload_offset == -1) {
        return;    
    }

    PWMPacket pwm_packet;
    memset(&pwm_packet, 0, sizeof(PWMPacket));
    pwm_packet.message_id = MSG_ID_PWM;
    pwm_packet.packet_id = 1234;
    pwm_packet.servo_0_pwm = 1500; // Example PWM values in µs
    pwm_packet.servo_1_pwm = 1500; // Example PWM values in µs
    pwm_packet.servo_2_pwm = 1500; // Example PWM values in µs
    pwm_packet.motor_rpm_x = 10000; // Example PWM values in µs
    pwm_packet.motor_rpm_y = 10000; // Example PWM values in µs
    pwm_packet.esc_1_pwm = 1000;   // Example PWM values in µs
    pwm_packet.esc_2_pwm = 1000;   // Example PWM values in µs
    uint8_t pwm_temp_buffer[32];
    size_t pwm_temp_size = build_pwm_packet(pwm_temp_buffer,
                                            (uint16_t[]){pwm_packet.servo_0_pwm, pwm_packet.servo_1_pwm, pwm_packet.servo_2_pwm}, pwm_packet.motor_rpm_x, pwm_packet.motor_rpm_y,
                                            (uint16_t[]){pwm_packet.esc_1_pwm, pwm_packet.esc_2_pwm});
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    pwm_temp_buffer, pwm_temp_size);
    if (payload_offset == -1) {
        return;    
    }

    SignalPacket sig_packet;
    memset(&sig_packet, 0, sizeof(SignalPacket));
    sig_packet.message_id = MSG_ID_SIGNAL;
    sig_packet.packet_id = 1234;
    sig_packet.lora_rssi = -70; // Example RSSI in dBm
    sig_packet.lora_snr = 80;   // Example link quality in % 
    sig_packet.xbee_rssi = -75; // Example RSSI in dBm
    sig_packet.xbee_snr = 70;   // Example link quality in %
    sig_packet.link_quality = 90; // Example link quality in %
    uint8_t sig_temp_buffer[32];
    size_t sig_temp_size = build_signal_packet(sig_temp_buffer,
                                               sig_packet.lora_rssi,
                                               sig_packet.lora_snr,
                                               sig_packet.xbee_rssi,
                                               sig_packet.xbee_snr, 
                                               sig_packet.link_quality); 
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    sig_temp_buffer, sig_temp_size);
    if (payload_offset == -1) {
        return;    
    }
    InternalSystemPacket int_sys_packet;
    memset(&int_sys_packet, 0, sizeof(InternalSystemPacket));
    int_sys_packet.message_id = MSG_ID_INTERNAL_SYSTEM;
    int_sys_packet.packet_id = 1234;
    int_sys_packet.cpu_core_0_load = 30; // Example CPU load in %
    int_sys_packet.cpu_core_1_load = 40; // Example CPU load in %
    int_sys_packet.arm_status = 1; // Example armed status

    uint8_t int_sys_temp_buffer[32];
    size_t int_sys_temp_size = build_internal_system_packet(int_sys_temp_buffer,
                                                            int_sys_packet.cpu_core_0_load,
                                                            int_sys_packet.cpu_core_1_load,
                                                            int_sys_packet.arm_status);
    payload_offset = add_sub_packet(payload_buffer, payload_offset, MAX_PAYLOAD_SIZE,
                                    int_sys_temp_buffer, int_sys_temp_size);
    if (payload_offset == -1) {
        return;    
    }
    
    // Create the final packet
    int packet_size = create_packet(PACKET_TYPE_COMBINED, payload_buffer, payload_offset,
                                   packet_buffer, sizeof(packet_buffer), PACKET_FLAG_RELIABLE);

    if (packet_size > 0) {
        send_to_uart(packet_buffer, packet_size);
    }
}
/*
void app_main(void)
{
    // Initialize UART
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_MAIN, UART_BUFFER_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_MAIN, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_MAIN, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Initialize the communication system
    packet_system_init();
    
    // Loop forever, sending packets periodically
    while (1) {
        send_well_packet();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Send every 1 second
    }
} 
*/