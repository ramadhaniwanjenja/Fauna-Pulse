# ESP32 Audio + Sensor Real-time Monitoring System

A comprehensive IoT solution that combines audio recording triggered by piezo vibrations with real-time environmental monitoring, all connected to Supabase cloud storage.

## 🎯 Features

- **Audio Recording**: 10-second piezo-triggered audio recording with WAV format output
- **Real-time Sensors**: Temperature, humidity (SHT30), and soil moisture monitoring
- **Cloud Integration**: Automatic upload to Supabase storage and database
- **Memory Optimized**: Efficient memory management for ESP32 constraints
- **Web-compatible**: Generates standard WAV files playable in browsers

## 📦 Hardware Requirements

### Components
- **ESP32 Development Board** (any variant with WiFi)
- **Piezo Sensor/Buzzer** (vibration trigger)
- **SHT30 Temperature/Humidity Sensor** (I2C)
- **Soil Moisture Sensor** (analog output)
- **Breadboard and Jumper Wires**

### Pin Connections
```
ESP32 Pin    |  Component
-------------|------------------
Pin 34       |  Piezo Sensor (analog)
Pin 32       |  Soil Moisture Sensor (analog)
Pin 21 (SDA) |  SHT30 Data Line
Pin 22 (SCL) |  SHT30 Clock Line
GND          |  All sensors ground
3.3V         |  SHT30 VCC
5V           |  Soil moisture sensor VCC
```

## 🛠️ Software Setup

### Required Libraries
Install these libraries through Arduino IDE Library Manager:
```
- WiFi (ESP32 built-in)
- HTTPClient (ESP32 built-in)
- ArduinoJson (by Benoit Blanchon)
- Adafruit SHT31 Library
- Wire (built-in)
```

### Configuration
1. **WiFi Credentials**:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```

2. **Supabase Setup**:
   - Create a Supabase project at [supabase.com](https://supabase.com)
   - Create two tables:
     - `audio_recordings` - for audio metadata
     - `sensor_data` - for sensor readings
   - Create a storage bucket named `recordings`
   - Update configuration:
   ```cpp
   const char* SUPABASE_URL = "https://your-project.supabase.co";
   const char* SUPABASE_ANON_KEY = "your-anon-key";
   ```

## 🗄️ Database Schema

### Table: `audio_recordings`
```sql
CREATE TABLE audio_recordings (
    id INTEGER PRIMARY KEY,
    filename TEXT,
    public_url TEXT,
    sample_rate INTEGER,
    duration INTEGER,
    format TEXT,
    size INTEGER,
    timestamp BIGINT,
    device TEXT,
    quality TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);
```

### Table: `sensor_data`
```sql
CREATE TABLE sensor_data (
    id INTEGER PRIMARY KEY,
    temperature REAL,
    humidity REAL,
    moisture INTEGER,
    timestamp BIGINT,
    device_id TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);
```

## ⚡ System Operation

### Audio Recording System
- **Trigger**: Piezo vibration above threshold
- **Duration**: 10 seconds per recording
- **Sample Rate**: 6kHz (compressed to 2kHz for storage)
- **Format**: 16-bit mono WAV files
- **Cooldown**: 3 seconds between recordings
- **Processing**: Real-time audio synthesis from piezo vibrations

### Sensor Monitoring
- **Reading Interval**: Every 5 seconds
- **Upload Interval**: Every 30 seconds to Supabase
- **Sensors**:
  - Temperature: SHT30 (°C)
  - Humidity: SHT30 (%RH)
  - Soil Moisture: Analog sensor (0-100%)

## 🔧 Technical Specifications

### Memory Usage
- **Audio Buffer**: 60,000 bytes (10s @ 6kHz)
- **Compressed Buffer**: 20,000 bytes (3:1 compression)
- **WAV Buffer**: 40,044 bytes (header + 16-bit data)
- **Total RAM**: ~120KB required

### Audio Processing
- **Compression Ratio**: 3:1 for memory efficiency
- **Bit Depth**: 16-bit signed PCM
- **Channels**: Mono
- **Web Compatibility**: Standard WAV format

## 🚀 Getting Started

1. **Hardware Assembly**:
   - Connect all sensors according to pin diagram
   - Ensure proper power supply (5V for moisture, 3.3V for SHT30)

2. **Software Setup**:
   - Install Arduino IDE with ESP32 board support
   - Install required libraries
   - Configure WiFi and Supabase credentials

3. **Upload Code**:
   - Select your ESP32 board
   - Upload the code
   - Monitor serial output for system status

4. **Testing**:
   - Trigger piezo sensor → should record 10s audio
   - Check Supabase storage for WAV files
   - Monitor sensor data in database

## 📊 Serial Monitor Output

The system provides detailed logging:
```
🎵 Audio System: Trigger piezo for 10-second recording
📊 Sensor System: Real-time monitoring active
📤 Data Upload: Every 30 seconds to Supabase
🌡️ Temp: 25.30 °C | 💧 Humidity: 45.20 %RH | 🌱 Moisture: 65 %
```

## 🔍 Troubleshooting

### Common Issues
- **Memory Allocation Failed**: Reduce `RECORD_TIME` or `SAMPLE_RATE`
- **WiFi Connection Issues**: Check credentials and signal strength
- **SHT30 Not Found**: Verify I2C wiring (SDA=21, SCL=22)
- **Upload Failures**: Check Supabase configuration and network
- **Audio Quality**: Adjust `sensitivity` and `baseline` calibration

### Performance Tips
- Keep WiFi signal strong for reliable uploads
- Use quality power supply for stable sensor readings
- Mount piezo sensor securely for consistent triggering
- Monitor memory usage through serial output

## 📁 File Structure
```
ESP32_Audio_Sensor/
├── ESP32_Audio_Sensor.ino  # Main code file
├── README.md               # This file
└── libraries/              # Required libraries
    ├── ArduinoJson/
    ├── Adafruit_SHT31/
    └── ...
```

## 🌐 Cloud Integration

### Supabase Storage
- **Bucket**: `recordings`
- **File Format**: `piezo_[counter]_10s_[timestamp].wav`
- **Public Access**: URLs generated for web playback
- **Metadata**: Stored in `audio_recordings` table

### Real-time Data
- **Sensor readings**: Uploaded every 30 seconds
- **Audio metadata**: Immediate after recording
- **Device identification**: Unique device_id per ESP32

## 🔧 Customization

### Adjustable Parameters
```cpp
#define SAMPLE_RATE 6000        // Audio sample rate
#define RECORD_TIME 10          // Recording duration (seconds)
#define COMPRESSION_RATIO 3     // Memory compression
#define SENSOR_READ_INTERVAL 5000    // Sensor read frequency
#define SENSOR_UPLOAD_INTERVAL 30000 // Upload frequency
#define TRIGGER_COOLDOWN 3000   // Cooldown between recordings
```

### Sensor Calibration
- **Piezo**: Auto-calibrates on startup
- **Moisture**: Adjust mapping values in `readSensors()`
- **Temperature/Humidity**: SHT30 factory calibrated

## 📝 License

This project is open-source. Feel free to modify and distribute according to your needs.

## 🤝 Contributing

Contributions welcome! Please test thoroughly before submitting pull requests.

---

**System Status**: Ready for deployment
**Last Updated**: 2025
**Tested On**: ESP32 DevKit V1, NodeMCU-32S