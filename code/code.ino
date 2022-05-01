#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  

QueueHandle_t queue_1;
QueueHandle_t queue_2;

//motor 1 (quạt)
#define ENA 13
#define IN1 12
#define IN2 11

//motor 2 (máy bơm)
#define ENB 8
#define IN3 9
#define IN4 10

//cảm biến và buzzer
#define buzzer 6
#define thermo A0
#define gas_sen A1
#define flame_sen A2

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  queue_1 = xQueueCreate(5, sizeof(float));
  queue_2 = xQueueCreate(5, sizeof(float));
  
  if (queue_1 == NULL) {
    Serial.println("Queue 1 can not be created");
  }
   if (queue_2 == NULL) {
    Serial.println("Queue 2 can not be created");
  }
  xTaskCreate(TaskDisplay, "Display_task", 128, NULL, 1, NULL);
  xTaskCreate(TaskLDR, "LDR_task", 128, NULL, 1, NULL);
  xTaskCreate(TaskControl, "Task control alarm", 128, NULL, 1, NULL);
  vTaskStartScheduler();
}
void loop() {
}

void both_alarm() {
  tone(buzzer,2500,700); //âm thanh tần số 2500, duration 700 ms
}

void gas_alarm() {
  tone(buzzer,2000,700);
}

void flame_alarm() {
  tone(buzzer,1500,700);
}

void motor_1_Dung_2_Tien() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);
}
 
void motor_1_Tien_2_Dung() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void motor_1_2_Tien() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);
}

void motor_1_Dung_2_Dung() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
}

void TaskDisplay(void * pvParameters) {
 float tempc = 0.0;
 float gas = 0.0;
 int flame;
  while(1) {  
    flame = digitalRead(flame_sen);
    if (xQueueReceive(queue_1, &tempc, portMAX_DELAY) == pdPASS && xQueueReceive(queue_2, &gas, portMAX_DELAY) == pdPASS) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("t: ");
      lcd.setCursor(4, 0);
      lcd.print(tempc);
      lcd.setCursor(9,0);
      lcd.print("gs: ");
      lcd.setCursor(13, 0);
      lcd.print(gas);
       if(gas> 300 && flame == LOW ){
        lcd.setCursor(0,1);
        lcd.print("co gas va co lua");   
       }
        else if(gas > 300 ){
         lcd.setCursor(0,1);
         lcd.print("co gas");
         lcd.setCursor(0,2);
         lcd.print("khong co lua");      
     }
     else if(flame == LOW ) {
         lcd.setCursor(0,1);
         lcd.print("khong co gas");
         lcd.setCursor(0,2);
         lcd.print("co lua"); 
         
    }
    else{
       lcd.setCursor(0,1);
       lcd.print("khong co gas va lua");  
       }     
       }
     }  
    }
    
void TaskControl(void * pvParameters) {
 float tempc = 0.0;
 float gas = 0.0;
 int flame ;
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 while(1) {
//   Serial.println("TaskControl");
   flame = digitalRead(flame_sen);
//   Serial.println("TaskDisplay");
   if (xQueueReceive(queue_1, &tempc, portMAX_DELAY) == pdPASS && xQueueReceive(queue_2, &gas, portMAX_DELAY) == pdPASS)  {
     if(gas> 300 && flame == LOW ){
         lcd.setCursor(0,3);
         lcd.print("Motor1: 1");
         lcd.setCursor(10,3);
         lcd.print("Motor2: 1");
         both_alarm();
       motor_1_2_Tien();
    }
       
       else if(gas > 300 ){
         lcd.setCursor(0,3);
         lcd.print("Motor1: 1");
         lcd.setCursor(10,3);
         lcd.print("Motor2: 0");
         gas_alarm();
         motor_1_Tien_2_Dung();
     }
     else if(flame == LOW ) {
         lcd.setCursor(0,3);
         lcd.print("Motor1: 0");
         lcd.setCursor(10,3);
         lcd.print("Motor2: 1");
         motor_1_Dung_2_Tien();
    }
    else{
         lcd.setCursor(0,3);
         lcd.print("Motor1: 0");
         lcd.setCursor(10,3);
         lcd.print("Motor2: 0");
         motor_1_Dung_2_Dung();
       }     
    }
  }
}

void TaskLDR(void * pvParameters) {
  float vout; 
  float tempc;
  float gas;
  
  while(1) {
//    Serial.println("TaskLDR");
    vout = analogRead(thermo); 
    vout = vout*0.34; // 5*100*vout/1024
    tempc = vout;
    gas = analogRead(gas_sen);
//    Serial.println(tempc);
    xQueueSend(queue_1, &tempc, portMAX_DELAY);

//    Serial.println(gas);
    xQueueSend(queue_2, &gas, portMAX_DELAY);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
  }
}
