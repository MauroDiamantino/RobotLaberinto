#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

///////////Pines/////////////
const byte PULSADOR1 = 4;
const byte PULSADOR2 = 7;
const byte PULSADOR3 = A0;
const byte PULSADOR4 = A1;
const byte LED1 = 9;
const byte LED2 = 10;

//////////Variables globales////////
volatile bool estado_led1 = false;
volatile bool estado_led2 = false;

////////////Funcion de inicializacion///////
void setup(){
	Serial.begin(9600);
	
	pinMode(PULSADOR1, INPUT_PULLUP);
	pinMode(PULSADOR2, INPUT_PULLUP);
	pinMode(PULSADOR3, INPUT_PULLUP);
	pinMode(PULSADOR4, INPUT_PULLUP);
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	
	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);

	attachPinChangeInterrupt(digitalPinToPCINT(PULSADOR1), ISR1, FALLING);
	attachPinChangeInterrupt(digitalPinToPCINT(PULSADOR2), ISR1, FALLING);
	attachPinChangeInterrupt(digitalPinToPCINT(PULSADOR3), ISR2, FALLING);
	attachPinChangeInterrupt(digitalPinToPCINT(PULSADOR4), ISR2, FALLING);
}

/////////Funcion loop//////////////
void loop(){
	//Este lazo lo que hace es mantener ocupado al micro mientras no llegan
	//interrupciones. De este modo, si los pines Pin Change no funcionar�n como lineas
	//de interrupcion externas reales, sino que fueran analizadas por 'polling',
	//entonces el micro tardar�a en atenderlas por lo que est� ocupado. Caso contrario,
	//por mas que el micro este ocupado con las tareas del lazo, deberia atender estas
	//interrupciones inmediatamente. Se eligi� que el indice del 'for' sea de tipo float
	//porque las operaciones de incremento y comparacion son mas complejas a bajo nivel.
  
	for (float i=0; i<65000; i++){
		Serial.println(i);
	}
}

/////////Rutinas de servicio///////////////
void ISR1(){
  sei();
	estado_led1 = !estado_led1;
	digitalWrite(LED1, estado_led1);
  delay(2000);
}

void ISR2(){
  sei();
	estado_led2 = !estado_led2;
	digitalWrite(LED2, estado_led2);
  delay(2000);
}
