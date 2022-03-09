// Pin1 Pin2 pump
// Pin3 4 5 6 right half 3,4 front
// Pin7 8 9 10 left half 7,8 front
// Pin 11 12 middle

int Pin1 = 1;
int Pin2 = 2;
int Pin3 = 3;
int Pin4 = 4;
int Pin5 = 5;
int Pin6 = 6;
int Pin7 = 7;
int Pin8 = 8;
int Pin9 = 9;
int Pin10 = 10;
int Pin11 = 11;
int Pin12 = 12;

void goAhead(int time); //forward
void pump(boolean flag); //pumps controlling
void turnRight(int time1); //turn right
void middle(boolean flag); //bellows controlling
void turnLeft(int time1); //turn left
void reset(); //reset muscle state

void setup()
{
  Serial.begin(9600);
  pinMode(Pin1, OUTPUT);
  pinMode(Pin2, OUTPUT);
  pinMode(Pin3, OUTPUT);
  pinMode(Pin4, OUTPUT);
  pinMode(Pin5, OUTPUT);
  pinMode(Pin6, OUTPUT);
  pinMode(Pin7, OUTPUT);
  pinMode(Pin8, OUTPUT);
  pinMode(Pin9, OUTPUT);
  pinMode(Pin10, OUTPUT);
  pinMode(Pin11, OUTPUT);
  pinMode(Pin12, OUTPUT);
}

void loop()
{
  char val = Serial.read();
  middle(false);
  switch (val) {
    case 'w' :
      goAhead(1000);
      break;
    case 'a':
      turnLeft(1000);
      break;
    case 'd':
      turnRight(1000);
      break;
    case 'q':
      pump(true);
      break;
    case 'e':
      pump(false);
      break;
    case 'g':
      middle(true);
      break;
    case 'h':
      middle(false);
      break;
    default:
      middle(true);
      break;
  }
}
void goAhead(int time1) {

  digitalWrite(Pin3, HIGH); //right front
  digitalWrite(Pin4, LOW); //right front
  digitalWrite(Pin5, LOW); //right back
  digitalWrite(Pin6, HIGH); //right back
  digitalWrite(Pin7, HIGH); //left front
  digitalWrite(Pin8, LOW); //left front
  digitalWrite(Pin9, LOW); //left back
  digitalWrite(Pin10, HIGH); //left back
  delay(time1);

  digitalWrite(Pin3, LOW); //right front
  digitalWrite(Pin4, HIGH); //right front
  digitalWrite(Pin5, HIGH); //right back
  digitalWrite(Pin6, LOW); //right back
  digitalWrite(Pin7, LOW); //left front
  digitalWrite(Pin8, HIGH); //left front
  digitalWrite(Pin9, HIGH); //left back
  digitalWrite(Pin10, LOW); //left back
  delay(time1);
  Serial.print("Command done!");
}

void turnRight(int time1) {
  
  digitalWrite(Pin7, HIGH); //right front
  digitalWrite(Pin8, LOW); //right front
  digitalWrite(Pin9, LOW); //right back
  digitalWrite(Pin10, HIGH); //right back
  
  delay(time1);

  digitalWrite(Pin7, LOW); //right front
  digitalWrite(Pin8, HIGH); //right front
  digitalWrite(Pin9, HIGH); //right back
  digitalWrite(Pin10, LOW); //right back
  
  delay(time1);
  Serial.print("Command done!");
}

void turnLeft(int time1) {
  
  digitalWrite(Pin3, HIGH); //right front
  digitalWrite(Pin4, LOW); //right front
  digitalWrite(Pin5, LOW); //right back
  digitalWrite(Pin6, HIGH); //right back
  
  delay(time1);

  digitalWrite(Pin3, LOW); //right front
  digitalWrite(Pin4, HIGH); //right front
  digitalWrite(Pin5, HIGH); //right back
  digitalWrite(Pin6, LOW); //right back
  
  delay(time1);
  
  Serial.print("Command done!");
}
void pump(boolean flag) {
  if (flag) {
    digitalWrite(Pin1, HIGH);
    digitalWrite(Pin2, HIGH);
  }
  else {
    digitalWrite(Pin1, LOW);
    digitalWrite(Pin2, LOW);
  }
  Serial.print("Command done!");
}

void middle(boolean flag) {
  if (flag) {
    digitalWrite(Pin11, HIGH);
    digitalWrite(Pin12, LOW);
    delay(3500);
    digitalWrite(Pin11, LOW);
    digitalWrite(Pin12, LOW);
  }
  else {
    digitalWrite(Pin11, LOW);
    digitalWrite(Pin12, HIGH);
    delay(4000);
    digitalWrite(Pin11, LOW);
    digitalWrite(Pin12, LOW);
  }
  Serial.print("Command done!");
}
void reset() {
  for (int i = 1;i <= 12; i++) {
    if (digitalRead(i) == HIGH){
      digitalWrite(i,LOW);
    }
  }
  delay(1000);
  Serial.print("Command done!");
}
