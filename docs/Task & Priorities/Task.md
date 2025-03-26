# Task
There are multiple tasks that will run on the ESP32S3. To ensure a manageable delay, we will divide the tasks and assign them to two cores on the multicore esp. This will allow us to run two tasks in parallel. 

Core 1 in ESP is where our processes run and the Core 0 is where Bluetooth, WiFi and System tasks run. But as we wont be using bluetooth and wifi, Core 0 will most likely be free for us to run non-heavy light tasks on it.

Below are the tasks and on which core they will run

| Task | Core  | Reason    |
| :---:   | :---: | :---: |
| IMU Process | 1   |  Need Continuous Processing |
| Sensor Fusion | 1 | Based on IMU, Processed on the same core|
| All Sensor Processes | 1 | Too heavy for Core 0 |
| LoRa Comms | 1/0 | Not Decided |
| State Management | 0 | Light Weight Process |
| SDCard Logging | 0 | Using in core 1 will cause massive delay |
| Calibration | 1 | Only one time process |



