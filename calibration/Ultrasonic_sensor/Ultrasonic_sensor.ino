// Sensor 1: 28.476
// Sensor 2: 28.249
#define SENSOR_M1 28.476
#define SENSOR_M2 28.249

#define SENSOR_FRONT_TRIG 7
#define SENSOR_FRONT_ECHO 8
#define SENSOR_SIDE_TRIG 12
#define SENSOR_SIDE_ECHO 13


#define TIMEOUT 3000
#define WAITING_TIME 1000


long duration_front;
long duration_side; 

//The following code is run once at the start of execution to setup the different peripherals

void setup() 
{ 
  pinMode(SENSOR_FRONT_TRIG, OUTPUT); 
  digitalWrite(SENSOR_FRONT_TRIG, LOW); 
  pinMode(SENSOR_FRONT_ECHO, INPUT); 
  
  pinMode(SENSOR_SIDE_TRIG, OUTPUT); 
  digitalWrite(SENSOR_SIDE_TRIG, LOW); 
  pinMode(SENSOR_SIDE_ECHO, INPUT); 
  // Set up the serial Communication
  Serial.begin(9600); 
}
void loop() 
{ 
  //The following code is run repeatedly 
  digitalWrite(SENSOR_FRONT_TRIG, HIGH); 
  digitalWrite(SENSOR_SIDE_TRIG, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(SENSOR_FRONT_TRIG, LOW); 
  digitalWrite(SENSOR_SIDE_TRIG, LOW); 
  delayMicroseconds(10); 
  
  duration_front = pulseIn(SENSOR_FRONT_ECHO, HIGH, TIMEOUT); 
  delay(TIMEOUT);
  duration_side = pulseIn(SENSOR_SIDE_ECHO, HIGH, TIMEOUT); 
  //Serial.print("Duration front: ");
  //Serial.print(duration front); 
  //Serial.println(" microseconds"); 
  double distance_front = (duration_front / 2.0) / SENSOR_M2;
  Serial.print("Distance Front: ");
  Serial.print(distance_front);
  Serial.println( "cm");
  
  double distance_side = (duration_side / 2.0) / SENSOR_M1;
  Serial.print("Distance Side: ");
  Serial.print(distance_side);
  Serial.println( "cm");
  delay(WAITING_TIME); 
}
