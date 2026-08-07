#include "RTClib.h"
RTC_DS1307 rtc;
byte NUMBER9[] = {1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1,
                 1, 1, 1, 1, 1, 1, 1, 1
                };
byte NUMBER8[] = {0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1,
                 0, 1, 1, 0, 1, 1, 1, 0
                };
byte NUMBER7[] = {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1,
                 1, 1, 1, 0, 1, 0, 0, 0
                };
byte NUMBER6[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1,
                 1, 0, 0, 0, 1, 1, 1, 1
                };
byte NUMBER5[] = {1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1,
                 1, 0, 0, 0, 1, 1, 1, 1
                };
byte NUMBER4[] = {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                 1, 1, 1, 1, 1, 1, 1, 1
                };
byte NUMBER3[] = {1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1,
                 1, 1, 1, 1, 1, 1, 1, 1
                };
byte NUMBER2[] = {1, 0, 0, 0, 0, 0, 1, 1,
                 1, 0, 0, 0, 0, 1, 0, 1,
                 1, 0, 0, 0, 1, 0, 0, 1,
                 1, 0, 0, 1, 0, 0, 0, 1,
                 0, 1, 1, 0, 0, 0, 0, 1
                };
byte NUMBER1[] = {0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0,
                 1, 1, 1, 1, 1, 1, 1, 1,
                 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0
                };
byte NUMBER0[] = { 1, 1, 1, 1, 1, 1, 1, 1,
                  1, 0, 0, 0, 0, 0, 0, 1,
                  1, 0, 0, 0, 0, 0, 0, 1, 
                  1, 0, 0, 0, 0, 0, 0, 1,
                  1, 1, 1, 1, 1, 1, 1, 1
                };
byte _[] = {       0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0
                  };
                  
byte A[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1};
byte B[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0};
byte C[] = {0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1};
byte D[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0};
byte E[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1};
byte F[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0};
byte G[] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0};
byte H[] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1};
byte I[] = {1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1};
byte J[] = {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0};
byte K[] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1};
byte L[] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1};
byte M[] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1};
byte N[] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1};
byte O[] = {0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0};
byte P[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0};
byte Q[] = {0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1};
byte R[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1};
byte S[] = {0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0};
byte T[] = {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
byte U[] = {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0};
byte V[] = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0};
byte W[] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1};
byte X[] = {1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1};
byte Y[] = {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0};
byte Z[] = {1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1};
byte dummy[] = {};
byte* alpha[] = {A, B, C, D, E,F, G, H, NUMBER1, J, K, L, M, N, NUMBER0, P, dummy, R, NUMBER5 ,T, U};//, V, W, X, Y, Z};
byte* num[] = {NUMBER0,NUMBER1,NUMBER2,NUMBER3,NUMBER4,NUMBER5,NUMBER6,NUMBER7,NUMBER8,NUMBER9};
int letterSpace;
int dotTime;
int mode = 0;

int ledPins[] = {A2, 3, 4, 5, 13, 6, 7, 8, 9, 10, 11, 12};
String temp;
String displayStr ;

void setup()
{ Serial.begin(38400);
  
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), intrDisplayText, FALLING);

  // setting the ports of the leds to OUTPUT(SExperiments)
  for ( int i = 3; i <= 13 ; i++ ) {
    pinMode(i, OUTPUT);
  }
  
  pinMode(A2, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(A0, HIGH);//yellow
  digitalWrite(A1, LOW);

  displayStr = "123";
  // defining the space between the letters (ms)
  letterSpace = 800;
  // defining the time dots appear (ms)
  dotTime = 200;


  if (! rtc.begin()) {
   while (1){   
    Serial.println("Couldn't find RTC");
   };
 }
 
  if (! rtc.isrunning()) {
   Serial.println("RTC is NOT running!");
   // following line sets the RTC to the date & time this sketch was compiled
   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   // This line sets the RTC with an explicit date & time, for example to set
   // January 21, 2014 at 3am you would call:
   // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
 }
 
}






void printLetterByte(byte letter[])
{
  int y;
  // printing the first y row of the letter

  int columns = 5;


  for (int i=0;i<columns;i++){
      for (y = 0; y < 8; y++)
        { 
          digitalWrite(ledPins[y], letter[y + 8*i]);
        }
        delayMicroseconds(dotTime);
    };

  
 
  for (y = 0; y < 8; y++)
  {
    digitalWrite(ledPins[y], 0);
  }
  delayMicroseconds(letterSpace);
}


void loop()//write here =)
{ 

  if(Serial.available()){
    detachInterrupt(digitalPinToInterrupt(2));
    
    temp = Serial.readString();
    temp.trim();
    temp.toUpperCase();

    if(temp == "SWITCH 0")
      mode = 0;
    else if(temp == "SWITCH 1")
      mode = 1;
    else if(temp == "SWITCH 2")
      mode = 2; 
    Serial.print("recieved: ");
    Serial.println(temp);
  
 }
attachInterrupt(digitalPinToInterrupt(2), intrDisplayText, FALLING);

  if(mode==0){
    DateTime now = rtc.now();
    char time[6];
    sprintf( time, "%02hhu_%02hhu_%02hhu", now.hour(), now.minute(),now.second() );
    displayStr = time;
//     Serial.print("time: ");
//    Serial.println(displayStr);
  }

  else if(mode==1){
//    Serial.println("mode1:");
    displayStr = temp;
  }
  else if(mode==2){
    //analogueClock;

//        Serial.println("mode2:");
  }
  else{
//        Serial.println("unk mode:");
        }
  

}


void showString(String str){
    char c;
    for (int i=0;i<str.length();i++){
//      if(str[i] >= 'A' && str[i] <='O') 
//        printLetter(alpha[str[i]-'A']);
//      if(str[i] >= '0' && str[i] <='1') 
     
        if(str[i] == '_') {
         printLetterByte(_);
         continue;
        }
        else if (mode==0)
          printLetterByte(num[str[i]-'0']);
//
        else if (mode==1)
          printLetterByte(alpha[str[i]-'A']);
    }
  }

void intrDisplayText() {
  showString(displayStr);
}
