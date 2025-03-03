#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Define motor control pins
#define IN1 26  // GPIO26 for IN1
#define IN2 27  // GPIO27 for IN2
#define IN3 14  // GPIO14 for IN3
#define IN4 12  // GPIO12 for IN4
#define ENA 25  // GPIO25 for ENA (PWM for Motor A)
#define ENB 33  // GPIO33 for ENB (PWM for Motor B)

// Define HC-SR04 pins
#define TRIGGER_PIN 5  // GPIO5 for Trigger
#define ECHO_PIN 18    // GPIO18 for Echo

// Motor speed (0-255)
const int motorSpeed = 90;

// HC-SR04 parameters
const int obstacleDistance = 20;  // Stop if obstacle is within 10 cm

// WiFi credentials
const char* ssid = "NSU-WiFi-6";      // Replace with your WiFi SSID
const char* password = "";  // Replace with your WiFi password

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Function to initialize motor control pins
void setupMotorPins() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Set initial motor state to stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, motorSpeed);
    analogWrite(ENB, motorSpeed);
}

// Function to initialize HC-SR04 pins
void setupHCSR04() {
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

// Function to measure distance using HC-SR04
float measureDistance() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    if (duration == 0) {
        // No echo detected, return an invalid distance
        return -1;
    }
    float distance = duration * 0.034 / 2;  // Convert to cm
    return distance;
}

// Function to move forward
void moveForward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Moving Forward");
}

// Function to stop
void stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped");
}

// Function to handle incoming HTTP POST requests
void handleCommand(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    // Parse the JSON payload
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, (const char*)data);

    if (error) {
        Serial.println("Failed to parse JSON payload.");
        request->send(400, "text/plain", "Invalid JSON payload");
        return;
    }

    // Extract the command
    String command = doc["command"].as<String>();

    if (command == "move_forward") {
        moveForward();
    } else if (command == "stop") {
        stop();
    } else {
        Serial.println("Unknown command");
    }

    request->send(200, "text/plain", "Command received: " + command);
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Initialize motor control pins
    setupMotorPins();

    // Initialize HC-SR04 pins
    setupHCSR04();

    // Connect to Wi-Fi
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    // Wait for connection with timeout
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {  // 10-second timeout
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to WiFi. Check credentials or network.");
        return;
    }

    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());

    // Add a new endpoint to return the distance
    server.on("/distance", HTTP_GET, [](AsyncWebServerRequest *request) {
        float distance = measureDistance();  // Measure the distance
        request->send(200, "text/plain", String(distance));  // Send the distance as a response
    });

    // Start the server
    server.on("/control", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, handleCommand);
    server.begin();
}

void loop() {
    // Measure distance using HC-SR04
    float distance = measureDistance();
    if (distance == -1) {
        Serial.println("Error: No echo detected from HC-SR04.");
    } else {
        Serial.println("Distance: " + String(distance) + " cm");

        // Stop if obstacle is within the specified distance
        if (distance <= obstacleDistance) {
            stop();
        }
    }

    delay(100);  // Small delay for stability
}