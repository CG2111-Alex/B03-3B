#define trigFront 7
#define echoFront 8
#define trigSide 12
#define echoSide 13

#define TIMEOUT 30000
#define WAITING_TIME 1000

#define SENSOR_SIDE 28.476
#define SENSOR_FRONT 28.249

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigSide, OUTPUT);
  pinMode(echoSide, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("Front: ");
  Serial.println(distance_front);
  Serial.print("Side: ");
  Serial.println(distance_side);
  

  delay(WAITING_TIME);

}

double US_distance(int trig, int echo, float constant) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, TIMEOUT); 
  double distance = (duration / 2.0) / constant;
  return distance;
}
