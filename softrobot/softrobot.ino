// Pin1 Pin2 pump
// Pin3 4 5 6 right half 3,4 front
// Pin7 8 9 10 left half 7,8 front
// Pin 11 12 middle
// Pin 14,15,16,17 for the brake
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
int Pin13 = 13;
char stringBuffer[128];

int Pin14 = 33;
int Pin15 = 32;
int Pin16 = 30;
int Pin17 = 31;
int Pin18 = 40;

char Lastchar = "";
void goAhead(int time); //forward
void pump(boolean flag); //pumps controlling
void turnRight(int time1); //turn right
void middle(boolean flag); //bellows controlling
void turnLeft(int time1); //turn left
void reset(); //reset muscle state
void keep(int x); //keep current pressure
void resetbrake();

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(0.1);
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
  pinMode(Pin13, OUTPUT);

  pinMode(Pin14, OUTPUT);
  pinMode(Pin15, OUTPUT);
  pinMode(Pin16, OUTPUT);
  pinMode(Pin17, OUTPUT);
  pinMode(Pin18, OUTPUT);
  pump(true);
  resetbrake();
//  Lastchar = "a";
  //  middle(true);
}

void loop()
{
  Serial.readBytes(stringBuffer, Serial.available()-1);
  char Command = Serial.read();
  switch (Command) {
    case 'w' :
      goAhead(1000);
      Lastchar =  'w';
      break;
    case 'a':
      turnLeft(1000);
      Lastchar =  'a';
      break;
    case 'd':
      turnRight(1000);
      Lastchar =  'd';
      break;
    case 'q':
      pump(true);
      Lastchar =  'q';
      break;
    case 'e':
      pump(false);
      Lastchar =  'e';
      break;
    case 'g':
      middle(true);
      Lastchar =  'g';
      break;
    case 'h':
      middle(false);
      Lastchar =  'h';
      break;
    default:
      switch (Lastchar) {
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
        default:
          break;
      }
  }
}
void goAhead(int time1) {
  digitalWrite(Pin14, LOW);
  digitalWrite(Pin15, HIGH);
  digitalWrite(Pin17, HIGH);
  digitalWrite(Pin16, LOW);
  delay(400);
  
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
  keep(1);
  keep(2);

  Serial.print(1);
}

void turnRight(int time1) {
  digitalWrite(Pin16, LOW);
  digitalWrite(Pin17, HIGH);
  digitalWrite(Pin15, LOW);
  digitalWrite(Pin14, HIGH);
  delay(400);
  
  digitalWrite(Pin7, HIGH); //right front
  digitalWrite(Pin8, LOW); //right front
  digitalWrite(Pin9, LOW); //right back
  digitalWrite(Pin10, HIGH); //right back
  digitalWrite(Pin14, HIGH);
  delay(time1);
  digitalWrite(Pin7, LOW); //right front
  digitalWrite(Pin8, HIGH); //right front
  digitalWrite(Pin9, HIGH); //right back
  digitalWrite(Pin10, LOW); //right back
  delay(time1);
//  digitalWrite(Pin14, LOW);
//  digitalWrite(Pin15, HIGH);
  keep(2);
//  delay(400);
//  digitalWrite(Pin15, LOW);
  Serial.print(1);
}

void turnLeft(int time1) {
  digitalWrite(Pin14, LOW);
  digitalWrite(Pin15, HIGH);
  digitalWrite(Pin17, LOW);
  digitalWrite(Pin16, HIGH);
  delay(400);

  digitalWrite(Pin3, HIGH); //right front
  digitalWrite(Pin4, LOW); //right front
  digitalWrite(Pin5, LOW); //right back
  digitalWrite(Pin6, HIGH); //right back
  digitalWrite(Pin16, HIGH);
  delay(time1);
  digitalWrite(Pin3, LOW); //right front
  digitalWrite(Pin4, HIGH); //right front
  digitalWrite(Pin5, HIGH); //right back
  digitalWrite(Pin6, LOW); //right back
  delay(time1);
  keep(1);
  Serial.print(1);
}

void pump(boolean flag) {
  if (flag) {
    //    digitalWrite(Pin1, HIGH);
    digitalWrite(Pin2, HIGH);
    digitalWrite(Pin13, HIGH);
    digitalWrite(Pin18, HIGH);
  }
  else {
    //    digitalWrite(Pin1, LOW);
    digitalWrite(Pin2, LOW);
    digitalWrite(Pin13, LOW);
    digitalWrite(Pin18, LOW);
  }
  Serial.print(1);
}

void middle(boolean flag) {
  digitalWrite(Pin11, HIGH);
  digitalWrite(Pin12, LOW);
  Serial.print(1);
}

void reset() {
  for (int i = 1; i <= 12; i++) {
    if (digitalRead(i) == HIGH) {
      digitalWrite(i, LOW);
    }
  }
  delay(1000);
  Serial.print(1);
}

void keep(int x) { // function to keep the presure
  switch (x) {
    case 1:
      digitalWrite(Pin3, LOW); //right front
      digitalWrite(Pin4, LOW); //right front
      digitalWrite(Pin5, LOW); //right back
      digitalWrite(Pin6, LOW); //right back
      break;
    case 2:
      digitalWrite(Pin7, LOW); //right front
      digitalWrite(Pin8, LOW); //right front
      digitalWrite(Pin9, LOW); //right back
      digitalWrite(Pin10, LOW); //right back
      break;
    case 3:
      digitalWrite(Pin11, LOW); //right front
      digitalWrite(Pin12, LOW); //right front
  }
}

void resetbrake() {
  digitalWrite(Pin14, LOW);
  digitalWrite(Pin15, LOW);
  digitalWrite(Pin16, LOW);
  digitalWrite(Pin17, LOW);
}
