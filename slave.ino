String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int ls,rs;

char inChar;

void setup() 
{
  Serial.begin(9600);
  pinMode(3,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);

  digitalWrite(3,LOW);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  
}

void loop()
 {
  
  if (stringComplete)
   {
      Serial.println(inputString);
      
    if(inputString.equals("green!"))
    {
        analogWrite(3, 0);
        analogWrite(10, 0);
        digitalWrite(13, HIGH);
        delay(5000);
        digitalWrite(13, LOW);
        Serial.print("Done");
    }
    else if(inputString.equals("red!"))
    {
        analogWrite(3, 0);
        analogWrite(10, 0);
        digitalWrite(12, HIGH);
        delay(5000);
        digitalWrite(12, LOW);
        Serial.print("Done");
    }
    else
    {
      ls = inputString.substring(1,4).toInt();
      rs = inputString.substring(5,8).toInt();
      analogWrite(3,ls);
      analogWrite(10,rs);
  }
    inputString = "";
    stringComplete = false;
  }
  
}

void serialEvent()
{
  while (Serial.available())
   {
  		inChar = (char)Serial.read();
  		inputString +=inChar;
    	if (inChar == '!')
     	{
    		stringComplete = true;
    	}

  }
}
