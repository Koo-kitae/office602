// Set Water Level Sensor Pins
#define SEN1 4    // Sensor at Low Level
#define SEN2 2    // Sensor at High Level

// Set RGB LED Pins
#define RED 13
#define GREEN 12
#define BLUE 11

bool full = false;    // Flag that Indicates Tank is full
bool empty = false;   // Flag that Indicates Tank is empty

void setup() { 
  // Set RGB LED Pins to Output    
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  
  // Set Water Level Sensor Pins to Input
  pinMode(SEN1, INPUT);
  pinMode(SEN2, INPUT);
}
 
void loop () {
  empty = digitalRead(SEN1);  // Read Sensor`s Raw Data
  full = digitalRead(SEN2);
  
  if(empty){                  // Tank is empty
    setcolor(0, 1, 0);        // Set LED to Green
    full = false;
  }
  
  else if(full) {             // Tank is full
    setcolor(1, 0, 0);        // Set LED to Red
    empty = false;
  }

  else{                       // There is Water in Tank fully
    setcolor(0, 0, 1);        // Set LED to Blue
    
    } 
}

void setcolor(int red, int green, int blue){
  digitalWrite(RED, red);
  digitalWrite(GREEN, green);
  digitalWrite(BLUE, blue);
}