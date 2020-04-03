

String dat="";  

void serialEvent()
{
  if(Serial.available())
 {
  dat=Serial.readString();
  Serial.print(dat);
 }
}

void AttachDcmotor(String Ble_data)
{
  if(Ble_data.indexOf("10") >=0 || Ble_data.indexOf("1110") >=0 || Ble_data.indexOf("2210") >=0)
    {
        digitalWrite(3,HIGH);
        digitalWrite(4,HIGH);
        Ble_data="";
    }
   else if(Ble_data.indexOf("22") >=0)
   {
   
      digitalWrite(3,HIGH);
      digitalWrite(4,LOW);
  
    
   }
   else if(Ble_data.indexOf("11") >=0)
   {
     
       digitalWrite(3,LOW);
       digitalWrite(4,HIGH); 
    }
    
}

void setup() {

Serial.begin(9600);
delay(500);
pinMode(3,OUTPUT);
pinMode(4,OUTPUT);
digitalWrite(3,HIGH);
digitalWrite(4,HIGH);
 Serial.println("Device Ready");
delay(500);

}

void loop() 
{
  // put your main code here, to run repeatedly:
 
 AttachDcmotor(dat);
}
