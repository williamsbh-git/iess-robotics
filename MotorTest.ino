// Define motor control pins
  #define M1A 7
  #define M1B 6
  #define M2A 10
  #define M2B 9

void setup() {
  // Set motor control pins as outputs
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Setup complete");
}

void loop() {
  // Test different directions for motor 1
  //A high - forwards | B high - backwards
  Serial.println("Testing Motor 1 - Forward");
  analogWrite(M1A, 255);  // PWM signal to control speed
  analogWrite(M1B, 0);   // PWM signal to control speed
  delay(2000);  // Run for 2 seconds

  Serial.println("Testing Motor 1 - Backward");
  analogWrite(M1A, 0);   // PWM signal to control speed
  analogWrite(M1B, 255);  // PWM signal to control speed
  delay(2000);  // Run for 2 seconds

  Serial.println("Testing Motor 1 - Stop");
  analogWrite(M1A, 0);    // PWM signal to stop motor
  analogWrite(M1B, 0);    // PWM signal to stop motor
  delay(2000);  // Run for 2 seconds

  // Test different directions for motor 2
  //A high - forwards | B high - backwards
  Serial.println("Testing Motor 2 - Forward");
  analogWrite(M2A, 255);  // PWM signal to control speed
  analogWrite(M2B, 0);   // PWM signal to control speed
  delay(2000);  // Run for 2 seconds

  Serial.println("Testing Motor 2 - Backward");
  analogWrite(M2A, 0);   // PWM signal to control speed
  analogWrite(M2B, 255);  // PWM signal to control speed
  delay(2000);  // Run for 2 seconds

  Serial.println("Testing Motor 2 - Stop");
  analogWrite(M2A, 0);    // PWM signal to stop motor
  analogWrite(M2B, 0);    // PWM signal to stop motor
  delay(2000);  // Run for 2 seconds

  // Add a short delay before repeating the tests
  delay(1000);
}
