#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <base64.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

// WiFi credentials
const char* ssid = "SammyG";
const char* password = "Sammyg123";

// Supabase configuration
const char* SUPABASE_URL = "https://knltowjloqjfyojbnbxc.supabase.co";
const char* SUPABASE_ANON_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImtubHRvd2psb3FqZnlvamJuYnhjIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTEzNTIyODUsImV4cCI6MjA2NjkyODI4NX0.9QUIjZjKZXYcAyqjxXw6cpeAa13GZf01xXr5j29lkYU";
const char* SUPABASE_AUDIO_TABLE = "audio_recordings";
const char* SUPABASE_SENSOR_TABLE = "sensor_data";  // New table for sensor data
const char* SUPABASE_BUCKET = "recordings";

// Pin definitions
#define PIEZO_PIN 34
#define MOISTURE_PIN 32  // ADC-capable pin on ESP32

// Sensor initialization
Adafruit_SHT31 sht30 = Adafruit_SHT31();

// Audio settings - Optimized for 10 seconds
#define SAMPLE_RATE 6000        // Reduced to 6kHz for longer recording
#define RECORD_TIME 10          // 10 seconds recording
#define BUFFER_SIZE (SAMPLE_RATE * RECORD_TIME)  // 60,000 samples
#define COMPRESSION_RATIO 3     // Higher compression to save memory
#define COMPRESSED_SIZE (BUFFER_SIZE / COMPRESSION_RATIO)  // 20,000 samples
#define WAV_HEADER_SIZE 44
#define BYTES_PER_SAMPLE 2      // 16-bit audio (2 bytes per sample)

// Sensor reading intervals
#define SENSOR_READ_INTERVAL 5000  // Read sensors every 5 seconds
#define SENSOR_UPLOAD_INTERVAL 30000  // Upload to Supabase every 30 seconds

// Memory-efficient buffers
uint8_t* audioBuffer = nullptr;
uint8_t* compressedBuffer = nullptr;
uint8_t* wavBuffer = nullptr;

// Audio recording variables
bool isRecording = false;
int recordIndex = 0;
unsigned long recordStartTime = 0;
int audioCounter = 0;

// Sensor variables
unsigned long lastSensorRead = 0;
unsigned long lastSensorUpload = 0;
float currentTemperature = 0.0;
float currentHumidity = 0.0;
int currentMoisture = 0;
int sensorDataCounter = 0;

// Piezo signal processing variables
float baseline = 512.0;
float sensitivity = 2.5;        // Slightly increased sensitivity
uint8_t lastProcessedSample = 128;
unsigned long lastTriggerTime = 0;
const unsigned long TRIGGER_COOLDOWN = 3000;  // 3 second cooldown between recordings

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Starting ESP32 Audio + Sensor Real-time System...");
  printMemoryUsage();
  
  // Initialize I2C for SHT30
  Wire.begin(); // SDA = 21, SCL = 22 (default ESP32 I2C pins)
  
  // Initialize SHT30 sensor
  if (!sht30.begin(0x44)) {
    Serial.println("❌ Couldn't find SHT30 sensor. Check wiring!");
    Serial.println("⚠️ Continuing without SHT30 sensor...");
  } else {
    Serial.println("✅ SHT30 sensor initialized.");
  }
  
  // Calculate total memory needed
  size_t totalMemoryNeeded = BUFFER_SIZE + COMPRESSED_SIZE + (COMPRESSED_SIZE + WAV_HEADER_SIZE);
  Serial.print("Total memory needed: ");
  Serial.print(totalMemoryNeeded);
  Serial.println(" bytes");
  
  // Allocate memory dynamically (16-bit audio needs more space)
  audioBuffer = (uint8_t*)malloc(BUFFER_SIZE);
  compressedBuffer = (uint8_t*)malloc(COMPRESSED_SIZE);
  wavBuffer = (uint8_t*)malloc(COMPRESSED_SIZE * BYTES_PER_SAMPLE + WAV_HEADER_SIZE);
  
  if (!audioBuffer || !compressedBuffer || !wavBuffer) {
    Serial.println("❌ Memory allocation failed!");
    Serial.println("💡 Try reducing RECORD_TIME or SAMPLE_RATE");
    while(1) delay(1000);
  }
  
  Serial.println("✅ Memory allocated successfully!");
  
  // Setup pins
  pinMode(PIEZO_PIN, INPUT);
  pinMode(MOISTURE_PIN, INPUT);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  
  // Calibrate piezo baseline
  calibratePiezo();
  
  // Read initial sensor values
  readSensors();
  
  Serial.println("🎵 Audio System: Trigger piezo for 10-second recording");
  Serial.println("📊 Sensor System: Real-time monitoring active");
  Serial.println("📤 Data Upload: Every 30 seconds to Supabase");
  printMemoryUsage();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle sensor readings
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    readSensors();
    lastSensorRead = currentTime;
  }
  
  // Handle sensor data upload
  if (currentTime - lastSensorUpload >= SENSOR_UPLOAD_INTERVAL) {
    uploadSensorData();
    lastSensorUpload = currentTime;
  }
  
  // Handle audio recording system
  int piezoValue = analogRead(PIEZO_PIN);
  
  // Check for piezo trigger with cooldown
  if (piezoValue > (baseline + 40) && !isRecording && (currentTime - lastTriggerTime) > TRIGGER_COOLDOWN) {
    startRecording();
  }
  
  if (isRecording) {
    recordAudio();
  }
  
  delay(10);  // Small delay to prevent overwhelming the system
}

void readSensors() {
  // Read temperature & humidity from SHT30
  float temperature = sht30.readTemperature();
  float humidity = sht30.readHumidity();
  
  // Read soil moisture
  int rawMoisture = analogRead(MOISTURE_PIN);
  int moisturePercent = map(rawMoisture, 3300, 1500, 0, 100);
  moisturePercent = constrain(moisturePercent, 0, 100);
  
  // Update current values
  if (!isnan(temperature) && !isnan(humidity)) {
    currentTemperature = temperature;
    currentHumidity = humidity;
    Serial.printf("🌡️ Temp: %.2f °C | 💧 Humidity: %.2f %%RH", temperature, humidity);
  } else {
    Serial.print("⚠️ SHT30 sensor read failed");
  }
  
  currentMoisture = moisturePercent;
  Serial.printf(" | 🌱 Moisture: %d %%\n", moisturePercent);
}

void uploadSensorData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected for sensor upload!");
    return;
  }
  
  HTTPClient http;
  
  // Create JSON payload for sensor data
  StaticJsonDocument<512> doc;
  doc["id"] = sensorDataCounter;
  doc["temperature"] = currentTemperature;
  doc["humidity"] = currentHumidity;
  doc["moisture"] = currentMoisture;
  doc["timestamp"] = millis();
  doc["device_id"] = "esp32_audio_sensor";
  doc["created_at"] = "now()";  // Supabase will handle the timestamp
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  String url = String(SUPABASE_URL) + "/rest/v1/" + SUPABASE_SENSOR_TABLE;
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(SUPABASE_ANON_KEY));
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Prefer", "return=representation");
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    Serial.println("📤 Sensor data uploaded successfully!");
    Serial.printf("📊 T:%.1f°C H:%.1f%% M:%d%%\n", currentTemperature, currentHumidity, currentMoisture);
  } else {
    Serial.print("❌ Sensor upload failed: ");
    Serial.println(httpResponseCode);
    String response = http.getString();
    Serial.println("Response: " + response);
  }
  
  http.end();
  sensorDataCounter++;
}

void calibratePiezo() {
  Serial.println("🎯 Calibrating piezo sensor...");
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(PIEZO_PIN);
    delay(10);
  }
  baseline = sum / 100.0;
  Serial.print("Piezo baseline: ");
  Serial.println(baseline);
}

void startRecording() {
  Serial.println("🎵 Recording 10 seconds of piezo vibrations...");
  isRecording = true;
  recordIndex = 0;
  recordStartTime = millis();
  lastTriggerTime = millis();
  
  // Clear buffers
  memset(audioBuffer, 128, BUFFER_SIZE);
  memset(compressedBuffer, 128, COMPRESSED_SIZE);
  
  // Print progress indicator
  Serial.print("Progress: [");
  for (int i = 0; i < 20; i++) Serial.print("-");
  Serial.print("] 0%");
}

void recordAudio() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - recordStartTime;
  
  if (recordIndex < BUFFER_SIZE && elapsedTime < (RECORD_TIME * 1000)) {
    int rawPiezo = analogRead(PIEZO_PIN);
    uint8_t audioSample = convertPiezoToAudio(rawPiezo);
    
    audioBuffer[recordIndex] = audioSample;
    recordIndex++;
    
    // Show progress every 10% (every second)
    if (recordIndex % (BUFFER_SIZE / 10) == 0) {
      int progress = (recordIndex * 100) / BUFFER_SIZE;
      Serial.print("\rProgress: [");
      for (int i = 0; i < 20; i++) {
        if (i < (progress / 5)) Serial.print("=");
        else Serial.print("-");
      }
      Serial.print("] ");
      Serial.print(progress);
      Serial.print("%");
    }
    
    // Sample at 6kHz (167us delay)
    delayMicroseconds(167);
  } else {
    isRecording = false;
    Serial.println("\n🎵 Recording complete! Processing and uploading...");
    processAndUpload();
  }
}

uint8_t convertPiezoToAudio(int piezoValue) {
  // Convert piezo vibration to audio signal
  float deviation = (float)piezoValue - baseline;
  
  // Amplify the signal with sensitivity
  deviation *= sensitivity;
  
  // Create more dynamic audio content
  unsigned long currentTime = millis();
  float timeInRecording = (float)(currentTime - recordStartTime) / 1000.0;  // seconds into recording
  
  // Add multiple frequency components for richer sound
  float freq1 = 440.0 + (abs(deviation) * 2.0);  // Base frequency varies with piezo
  float freq2 = 220.0;  // Sub-harmonic
  
  // Generate complex waveform
  float wave1 = sin(2 * 3.14159 * freq1 * timeInRecording) * 60;
  float wave2 = sin(2 * 3.14159 * freq2 * timeInRecording) * 30;
  
  // Mix the waves with piezo influence
  float mixedWave = wave1 + wave2;
  
  // Apply piezo modulation
  if (abs(deviation) > 20) {
    mixedWave *= (1.0 + abs(deviation) / 50.0);  // Amplify during piezo activity
  } else {
    mixedWave *= 0.5;  // Quieter during no activity
  }
  
  // Add the deviation from piezo
  float finalSignal = mixedWave + deviation;
  
  // Convert to 8-bit audio sample (0-255) - we'll convert to 16-bit later
  int audioSample = 128 + (int)(finalSignal * 0.8);
  
  // Clamp to valid range
  audioSample = constrain(audioSample, 0, 255);
  
  // Enhanced smoothing filter
  uint8_t smoothedSample = (audioSample * 0.7 + lastProcessedSample * 0.3);
  lastProcessedSample = smoothedSample;
  
  return smoothedSample;
}

void processAndUpload() {
  printMemoryUsage();
  
  Serial.println("🔄 Compressing audio data...");
  
  // Compress audio data with better averaging
  for (int i = 0; i < COMPRESSED_SIZE; i++) {
    int sum = 0;
    int count = 0;
    for (int j = 0; j < COMPRESSION_RATIO; j++) {
      int index = i * COMPRESSION_RATIO + j;
      if (index < recordIndex) {
        sum += audioBuffer[index];
        count++;
      }
    }
    compressedBuffer[i] = count > 0 ? sum / count : 128;
    
    // Show compression progress
    if (i % (COMPRESSED_SIZE / 10) == 0) {
      int progress = (i * 100) / COMPRESSED_SIZE;
      Serial.print("Compression: ");
      Serial.print(progress);
      Serial.println("%");
    }
  }
  
  // Create WAV format
  createWAVBuffer();
  
  Serial.print("✅ Final WAV size: ");
  Serial.print(COMPRESSED_SIZE + WAV_HEADER_SIZE);
  Serial.println(" bytes");
  
  // Upload to Supabase Storage and save metadata
  uploadToSupabaseStorage();
  
  Serial.println("🎯 Ready for next recording!");
  Serial.print("⏱️  Cooldown: ");
  Serial.print(TRIGGER_COOLDOWN / 1000);
  Serial.println(" seconds");
  Serial.println("================================");
}

void createWAVBuffer() {
  uint32_t sampleRate = SAMPLE_RATE / COMPRESSION_RATIO;  // 2000 Hz
  uint32_t numChannels = 1;  // Mono
  uint32_t bitsPerSample = 16;
  uint32_t bytesPerSample = bitsPerSample / 8;
  uint32_t blockAlign = numChannels * bytesPerSample;
  uint32_t byteRate = sampleRate * blockAlign;
  uint32_t dataSize = COMPRESSED_SIZE * bytesPerSample;
  uint32_t fileSize = dataSize + WAV_HEADER_SIZE - 8;
  
  // Clear the WAV buffer first
  memset(wavBuffer, 0, COMPRESSED_SIZE * BYTES_PER_SAMPLE + WAV_HEADER_SIZE);
  
  // Create WAV header with proper byte ordering
  uint8_t* header = wavBuffer;
  
  // RIFF header
  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  // File size (little endian)
  header[4] = (fileSize) & 0xFF;
  header[5] = (fileSize >> 8) & 0xFF;
  header[6] = (fileSize >> 16) & 0xFF;
  header[7] = (fileSize >> 24) & 0xFF;
  // WAVE identifier
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  
  // fmt subchunk
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  // Subchunk1Size (16 for PCM)
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
  // AudioFormat (1 for PCM)
  header[20] = 1; header[21] = 0;
  // NumChannels
  header[22] = numChannels & 0xFF; header[23] = (numChannels >> 8) & 0xFF;
  // SampleRate (little endian)
  header[24] = sampleRate & 0xFF;
  header[25] = (sampleRate >> 8) & 0xFF;
  header[26] = (sampleRate >> 16) & 0xFF;
  header[27] = (sampleRate >> 24) & 0xFF;
  // ByteRate (little endian)
  header[28] = byteRate & 0xFF;
  header[29] = (byteRate >> 8) & 0xFF;
  header[30] = (byteRate >> 16) & 0xFF;
  header[31] = (byteRate >> 24) & 0xFF;
  // BlockAlign
  header[32] = blockAlign & 0xFF; header[33] = (blockAlign >> 8) & 0xFF;
  // BitsPerSample
  header[34] = bitsPerSample & 0xFF; header[35] = (bitsPerSample >> 8) & 0xFF;
  
  // data subchunk
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  // Subchunk2Size (little endian)
  header[40] = dataSize & 0xFF;
  header[41] = (dataSize >> 8) & 0xFF;
  header[42] = (dataSize >> 16) & 0xFF;
  header[43] = (dataSize >> 24) & 0xFF;
  
  // Convert and write audio data
  for (int i = 0; i < COMPRESSED_SIZE; i++) {
    // Convert 8-bit unsigned (0-255) to 16-bit signed (-32768 to 32767)
    // Center the 8-bit value around 128, then scale to 16-bit range
    int16_t sample16 = (int16_t)((int)(compressedBuffer[i]) - 128) * 256;
    
    // Apply slight amplification for better web playback
    sample16 = (int16_t)(sample16 * 1.2);
    
    // Clamp to prevent overflow
    if (sample16 > 32767) sample16 = 32767;
    if (sample16 < -32768) sample16 = -32768;
    
    // Write in little endian format
    int bufferIndex = WAV_HEADER_SIZE + (i * bytesPerSample);
    wavBuffer[bufferIndex] = sample16 & 0xFF;           // Low byte
    wavBuffer[bufferIndex + 1] = (sample16 >> 8) & 0xFF; // High byte
  }
  
  Serial.print("📊 Web-compatible WAV created: ");
  Serial.print(sampleRate);
  Serial.print(" Hz, ");
  Serial.print(numChannels);
  Serial.print(" channel(s), ");
  Serial.print(bitsPerSample);
  Serial.print(" bits, ");
  Serial.print(dataSize);
  Serial.print(" bytes data, ");
  Serial.print(fileSize + 8);
  Serial.println(" bytes total");
}

void uploadToSupabaseStorage() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected!");
    return;
  }
  
  HTTPClient http;
  
  // Generate unique filename
  String filename = "piezo_" + String(audioCounter) + "_" + String(RECORD_TIME) + "s_" + String(millis()) + ".wav";
  
  // Upload to Supabase Storage
  String storageUrl = String(SUPABASE_URL) + "/storage/v1/object/" + SUPABASE_BUCKET + "/" + filename;
  
  Serial.println("📤 Uploading: " + filename);
  
  http.begin(storageUrl);
  http.addHeader("Authorization", "Bearer " + String(SUPABASE_ANON_KEY));
  http.addHeader("Content-Type", "audio/wav");  // Ensure proper MIME type
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Cache-Control", "public, max-age=3600");  // Add cache control
  
  // Calculate exact file size
  size_t fileSize = (COMPRESSED_SIZE * BYTES_PER_SAMPLE) + WAV_HEADER_SIZE;
  
  int httpResponseCode = http.POST(wavBuffer, fileSize);
  
  if (httpResponseCode > 0) {
    Serial.println("✅ Upload successful!");
    Serial.print("📝 Response code: ");
    Serial.println(httpResponseCode);
    
    // Get public URL
    String publicUrl = String(SUPABASE_URL) + "/storage/v1/object/public/" + SUPABASE_BUCKET + "/" + filename;
    Serial.println("🎵 Audio URL: " + publicUrl);
    
    // Save metadata to database
    saveAudioMetadataToDatabase(filename, publicUrl);
    
    // Test the URL immediately
    Serial.println("🔍 Testing URL accessibility...");
    testAudioUrl(publicUrl);
    
  } else {
    Serial.print("❌ Upload failed. Error: ");
    Serial.println(httpResponseCode);
    String response = http.getString();
    Serial.println("Response: " + response);
  }
  
  http.end();
  audioCounter++;
  printMemoryUsage();
}

// Add this function to test if the URL is accessible
void testAudioUrl(String url) {
  HTTPClient http;
  http.begin(url);
  http.addHeader("Range", "bytes=0-100");  // Just test first 100 bytes
  
  int httpResponseCode = http.GET();
  
  if (httpResponseCode == 200 || httpResponseCode == 206) {
    Serial.println("✅ URL is accessible!");
    String contentType = http.header("Content-Type");
    Serial.println("📄 Content-Type: " + contentType);
  } else {
    Serial.print("❌ URL test failed: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}

void saveAudioMetadataToDatabase(String filename, String publicUrl) {
  HTTPClient http;
  
  // Create JSON payload
  StaticJsonDocument<512> doc;
  doc["id"] = audioCounter;
  doc["filename"] = filename;
  doc["public_url"] = publicUrl;
  doc["sample_rate"] = SAMPLE_RATE / COMPRESSION_RATIO;
  doc["duration"] = RECORD_TIME;
  doc["format"] = "wav";
  doc["size"] = (COMPRESSED_SIZE * BYTES_PER_SAMPLE) + WAV_HEADER_SIZE;
  doc["timestamp"] = millis();
  doc["device"] = "esp32_piezo_extended";
  doc["quality"] = "compressed";
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  String url = String(SUPABASE_URL) + "/rest/v1/" + SUPABASE_AUDIO_TABLE;
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(SUPABASE_ANON_KEY));
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Prefer", "return=representation");
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    Serial.println("✅ Audio metadata saved!");
  } else {
    Serial.print("❌ Audio metadata save failed: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}

void printMemoryUsage() {
  Serial.print("💾 Free heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.print(" bytes, Min free: ");
  Serial.print(ESP.getMinFreeHeap());
  Serial.println(" bytes");
}