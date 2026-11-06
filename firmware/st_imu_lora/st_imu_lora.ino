// The controller MCU is unspecified; this integration targets Arduino-ESP32.
void adapterSetup();
void adapterLoop();
void setup() { adapterSetup(); }
void loop() { adapterLoop(); }
