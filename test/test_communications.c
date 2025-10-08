#include <unity.h>
#include <string.h>
#include "communications.h"
#include "esp_log.h"

static const char* TAG = "COMMS_TEST";

// Test data
static uint8_t test_buffer[MAX_PACKET_SIZE];
static uint8_t test_payload[MAX_PAYLOAD_SIZE];
static packet_context_t test_context;

void setUp(void) {
    memset(test_buffer, 0, sizeof(test_buffer));
    memset(test_payload, 0, sizeof(test_payload));
    packet_system_init();
    ESP_LOGI(TAG, "Test setup complete");
}

void tearDown(void) {
    // Nothing to do
    ESP_LOGI(TAG, "Test teardown complete");
}

// Test CRC16 implementation matches the ground station
void test_crc16_matches_ground_station(void) {
    ESP_LOGI(TAG, "Testing CRC16 implementation...");
    // Test data from ground station example
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint16_t expected_crc = 0x93E1; // Expected CRC from ground station
    uint16_t calculated_crc = calculate_crc16(data, sizeof(data));
    
    ESP_LOGI(TAG, "Expected CRC16: 0x%04X, Calculated: 0x%04X", expected_crc, calculated_crc);
    TEST_ASSERT_EQUAL_HEX16(expected_crc, calculated_crc);
}

// Test CRC32 implementation matches the ground station
void test_crc32_matches_ground_station(void) {
    ESP_LOGI(TAG, "Testing CRC32 implementation...");
    // Test data from ground station example
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint32_t expected_crc = 0xCDB4AC94; // Expected CRC from ground station
    uint32_t calculated_crc = calculate_crc32(data, sizeof(data));
    
    ESP_LOGI(TAG, "Expected CRC32: 0x%08X, Calculated: 0x%08X", expected_crc, calculated_crc);
    TEST_ASSERT_EQUAL_HEX32(expected_crc, calculated_crc);
}

// Test packet creation without end byte
void test_packet_creation_without_end_byte(void) {
    ESP_LOGI(TAG, "Testing packet creation without end byte...");
    // Create a simple packet
    for (int i = 0; i < 10; i++) {
        test_payload[i] = i;
    }
    
    int size = create_packet(PACKET_TYPE_TELEMETRY, test_payload, 10, 
                           test_buffer, sizeof(test_buffer), PACKET_FLAG_NONE);
    
    // Verify packet size (header(6) + payload(10) + crc32(4)) = 20
    ESP_LOGI(TAG, "Created packet size: %d bytes (expected 20)", size);
    TEST_ASSERT_EQUAL(20, size);
    
    // Check start byte
    ESP_LOGI(TAG, "Start byte: 0x%02X (expected 0x%02X)", test_buffer[0], PACKET_START_BYTE);
    TEST_ASSERT_EQUAL_HEX8(PACKET_START_BYTE, test_buffer[0]);
    
    // Make sure there's no end byte
    // The last 4 bytes should be the CRC32, not an end byte
    ESP_LOGI(TAG, "Last byte: 0x%02X (should not be 0x7F)", test_buffer[size-1]);
    TEST_ASSERT_NOT_EQUAL(0x7F, test_buffer[size-1]);
    
    // Log packet hex dump for debugging
    ESP_LOGI(TAG, "Packet dump:");
    for (int i = 0; i < size; i++) {
        printf("%02X ", test_buffer[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
}

// Test sub-packet CRC16 implementation
void test_sub_packet_crc16(void) {
    ESP_LOGI(TAG, "Testing sub-packet CRC16 implementation...");
    // Create a test sub-packet in our payload buffer
    uint8_t sub_data[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    int offset = add_sub_packet(test_payload, 0, sizeof(test_payload),
                              MSG_ID_ACCEL, sub_data, sizeof(sub_data));
    
    // Verify sub-packet structure
    ESP_LOGI(TAG, "Sub-packet size: %d bytes (expected 9)", offset);
    TEST_ASSERT_EQUAL(9, offset); // ID(1) + Len(1) + Data(5) + CRC(2) = 9
    TEST_ASSERT_EQUAL(MSG_ID_ACCEL, test_payload[0]);
    TEST_ASSERT_EQUAL(5, test_payload[1]); // Length
    
    // Extract and verify CRC16
    uint16_t extracted_crc = (test_payload[7] << 8) | test_payload[8];
    uint16_t calculated_crc = calculate_crc16(test_payload, 7); // CRC over ID + Len + Data
    ESP_LOGI(TAG, "Sub-packet CRC16: Extracted=0x%04X, Calculated=0x%04X", 
             extracted_crc, calculated_crc);
    TEST_ASSERT_EQUAL_HEX16(calculated_crc, extracted_crc);
    
    // Log sub-packet hex dump
    ESP_LOGI(TAG, "Sub-packet dump:");
    for (int i = 0; i < offset; i++) {
        printf("%02X ", test_payload[i]);
    }
    printf("\n");
}

// Test accelerometer packet structure matches specification
void test_accelerometer_packet_structure(void) {
    ESP_LOGI(TAG, "Testing accelerometer packet structure...");
    int16_t x = 100, y = 200, z = 300;
    size_t size = build_accel_packet(test_buffer, x, y, z);
    
    // Verify structure
    ESP_LOGI(TAG, "Accel packet size: %d bytes (expected 11)", size);
    TEST_ASSERT_EQUAL(11, size); // MSG_ID(1) + packet_id(4) + x,y,z(2*3) = 11
    TEST_ASSERT_EQUAL(MSG_ID_ACCEL, test_buffer[0]); // 0xA4
    
    // Check the 32-bit packet ID
    uint32_t packet_id = test_buffer[1] | 
                        (test_buffer[2] << 8) | 
                        (test_buffer[3] << 16) | 
                        (test_buffer[4] << 24);
    ESP_LOGI(TAG, "Packet ID: %u (expected 0)", packet_id);
    TEST_ASSERT_EQUAL(0, packet_id); // First packet should be 0
    
    // Check accelerometer values
    int16_t extracted_x = test_buffer[5] | (test_buffer[6] << 8);
    int16_t extracted_y = test_buffer[7] | (test_buffer[8] << 8);
    int16_t extracted_z = test_buffer[9] | (test_buffer[10] << 8);
    
    ESP_LOGI(TAG, "Accelerometer values - X: %d (expected %d), Y: %d (expected %d), Z: %d (expected %d)",
             extracted_x, x, extracted_y, y, extracted_z, z);
    TEST_ASSERT_EQUAL(x, extracted_x);
    TEST_ASSERT_EQUAL(y, extracted_y);
    TEST_ASSERT_EQUAL(z, extracted_z);
    
    // Log packet hex dump
    ESP_LOGI(TAG, "Accelerometer packet dump:");
    for (int i = 0; i < size; i++) {
        printf("%02X ", test_buffer[i]);
    }
    printf("\n");
}

// Test packet parsing without end byte
void test_packet_parsing_without_end_byte(void) {
    ESP_LOGI(TAG, "Testing packet parsing without end byte...");
    // Create a packet to parse
    for (int i = 0; i < 10; i++) {
        test_payload[i] = i;
    }
    
    int size = create_packet(PACKET_TYPE_TELEMETRY, test_payload, 10,
                           test_buffer, sizeof(test_buffer), PACKET_FLAG_NONE);
    ESP_LOGI(TAG, "Created packet of size %d bytes", size);
    
    // Parse the packet
    packet_type_t packet_type;
    const uint8_t *payload;
    size_t payload_len;
    
    esp_err_t result = parse_packet(test_buffer, size, &packet_type, &payload, &payload_len);
    
    // Verify parsing was successful
    ESP_LOGI(TAG, "Parse result: %d (expected %d/ESP_OK)", result, ESP_OK);
    TEST_ASSERT_EQUAL(ESP_OK, result);
    TEST_ASSERT_EQUAL(PACKET_TYPE_TELEMETRY, packet_type);
    ESP_LOGI(TAG, "Payload length: %d (expected 10)", payload_len);
    TEST_ASSERT_EQUAL(10, payload_len);
    
    // Verify payload contents
    bool payload_ok = true;
    for (int i = 0; i < 10; i++) {
        if (payload[i] != i) {
            payload_ok = false;
            ESP_LOGE(TAG, "Payload mismatch at index %d: got %d, expected %d", i, payload[i], i);
            break;
        }
    }
    TEST_ASSERT_TRUE(payload_ok);
    ESP_LOGI(TAG, "Payload contents verified successfully");
}

// Test complete communication flow with sub-packets
void test_complete_communication_flow(void) {
    ESP_LOGI(TAG, "Testing complete communication flow...");
    // Create a test sensor data structure
    SensorData test_data = {
        .ax = 1.0f, .ay = 0.5f, .az = 9.8f,
        .gx = 0.1f, .gy = 0.2f, .gz = 0.3f,
        .mx = 20.0f, .my = 30.0f, .mz = 40.0f,
        .temperature = 25.5f,
        .pressure = 1013.25f,
        .humidity = 45.0f,
        .altitude = 100.0f,
        .voltage = 3.3f,
        .latitude = 37.7749f,
        .longitude = -122.4194f,
        .gas_level = 150.0f
    };
    
    // Initialize differential packet system
    differential_packet_init();
    
    // Create a differential packet with force full = true to include all data
    int packet_size = create_differential_telemetry_packet(&test_data, test_buffer, 
                                                          sizeof(test_buffer), true);
    
    // Verify packet was created
    ESP_LOGI(TAG, "Created differential telemetry packet: %d bytes", packet_size);
    TEST_ASSERT_GREATER_THAN(0, packet_size);
    
    // Verify packet starts with start byte
    ESP_LOGI(TAG, "Start byte: 0x%02X (expected 0x%02X)", test_buffer[0], PACKET_START_BYTE);
    TEST_ASSERT_EQUAL_HEX8(PACKET_START_BYTE, test_buffer[0]);
    
    // Parse the packet
    packet_type_t packet_type;
    const uint8_t *payload;
    size_t payload_len;
    
    esp_err_t result = parse_packet(test_buffer, packet_size, &packet_type, &payload, &payload_len);
    
    // Verify parsing was successful
    ESP_LOGI(TAG, "Parse result: %d (expected %d/ESP_OK)", result, ESP_OK);
    TEST_ASSERT_EQUAL(ESP_OK, result);
    ESP_LOGI(TAG, "Packet type: %d (expected %d/PACKET_TYPE_COMBINED)", 
             packet_type, PACKET_TYPE_COMBINED);
    TEST_ASSERT_EQUAL(PACKET_TYPE_COMBINED, packet_type);
    
    // Verify payload contains sub-packets (just check for non-zero length)
    ESP_LOGI(TAG, "Payload length: %d (should be > 0)", payload_len);
    TEST_ASSERT_GREATER_THAN(0, payload_len);
    
    // Log first few bytes of payload as hex
    ESP_LOGI(TAG, "Payload preview (first 16 bytes):");
    for (int i = 0; i < 16 && i < payload_len; i++) {
        printf("%02X ", payload[i]);
    }
    printf("\n");
}

// Main test runner
void app_main(void)
{
    // Set log level to info to see all test output
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "Starting communication protocol tests...");
    
    UNITY_BEGIN();
    
    RUN_TEST(test_crc16_matches_ground_station);
    RUN_TEST(test_crc32_matches_ground_station);
    RUN_TEST(test_packet_creation_without_end_byte);
    RUN_TEST(test_sub_packet_crc16);
    RUN_TEST(test_accelerometer_packet_structure);
    RUN_TEST(test_packet_parsing_without_end_byte);
    RUN_TEST(test_complete_communication_flow);
    
    ESP_LOGI(TAG, "All tests completed!");
    UNITY_END();
}