// Implementation is ordinary C++ to avoid Arduino auto-prototype rewriting.
void adapterSetup();
void adapterLoop();
void setup() { adapterSetup(); }
void loop() { adapterLoop(); }
