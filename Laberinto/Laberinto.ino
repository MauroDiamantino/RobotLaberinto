//////////////////////////////////////////LIBRERIAS//////////////////////////////////////////////////////////////////////////////////////////////////
#include <FlexiTimer2.h>
//#include <TimerOne.h>

//////////////////////////////////////////CONSTANTES//////////////////////////////////////////////////////////////////////////////////////////////////
//Pines
const unsigned int PIN_ENCODER_LEFT = 2;
const unsigned int PIN_ENCODER_RIGHT = 3;

//Constantes del algoritmo
//const float PI = 3.14; //Es innecesario definir esta constante porque ya esta predefinida
const float INT_MUESTREO = 1000.0; //Intervalo de muestro en milisegundos
const unsigned int NUM_FLANCOS_VUELTA = 16; //Cantidad de flancos efectivos por vuelta, es decir flancos que producen interrupciones.
                                            //Esto depende de la configuracion de las interrupciones del micro.

//////////////////////////////////////////VARIABLES GLOBALES//////////////////////////////////////////////////////////////////////////////////////////
//velocidades angulares de las ruedas
float vel_angular_left = 0; //no se obtiene mejora si se usa el tipo de dato 'double' por la placa que se esta usando
float vel_angular_right = 0;

//velocidades lineales de las ruedas 
float vel_lineal_left = 0;
float vel_lineal_right = 0;

//Contadores de los flancos efectivos (aquellos que producen interrupciones) de los encoder.
unsigned int flancos_left = 0;
unsigned int flancos_right = 0;

//Flancos efectivos contados por los encoder en un determinado lapso de tiempo, el intervalo de muestreo.
unsigned int total_flancos_left = 0;  
unsigned int total_flancos_right = 0;

char buffer[100];

///////////////////////////////////////////////FLAGs////////////////////////////////////////////////////////////////////////////////////////////////////
boolean flag_calc_vel = false;

///////////////////////////////////////////////SETUP////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  //Configuración de pines
  pinMode(PIN_ENCODER_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENCODER_RIGHT, INPUT_PULLUP);

  //Configuracion de la comunicación serie
  Serial.begin(9600);
  Serial.println("DATOS DE LA RUEDA IZQUIERDA");
  Serial.println("FLANCOS     VEL_ANG     VEL_LIN");

  //Configuracion de las interrupciones
  //Usar CHANGE si se desea que ambos flancos, negativo y positivo, produzcan interrupciones (sean efectivos), y configurar el numero de flancos efectivos
  //por vuelta (NUM_FLANCOS_VUELTA) en 16. Si se usa solo uno de los flancos, usar FALLING o RISING y 8 flancos efectivos por vuelta.
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT),RS_ENCODER_LEFT, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT),RS_ENCODER_RIGHT, CHANGE);

  //Configuración del Timer 2
  FlexiTimer2::set(INT_MUESTREO, RS_TIMER2); // El tiempo de muestreo se coloca en milisegundos
  FlexiTimer2::start();
}

///////////////////////////////////////////////LOOP////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  
  if (flag_calc_vel == true){

    vel_angular_left = ( (total_flancos_left/NUM_FLANCOS_VUELTA)*2.0*PI / (INT_MUESTREO/1000.0) );// La velocidad angular esta en rad/s. Se divide el intervalo de 
                                                                                                //muestreo en 1000 porque esta en milisegundos
    vel_lineal_left = vel_angular_left*0.03; // La velocidad lineal esta en m/s, se obtiene al multiplicar la velocidad angular por el radio de la rueda (3cm, es decir 0.03m)

    vel_angular_right = ( (total_flancos_right/NUM_FLANCOS_VUELTA)*2.0*PI / (INT_MUESTREO/1000.0) );// La velocidad angular esta en rad/s. Se divide el intervalo de 
                                                                                                //muestreo en 1000 porque esta en milisegundos
    vel_lineal_right = vel_angular_right*0.03; // La velocidad lineal esta en m/s, se obtiene al multiplicar la velocidad angular por el radio de la rueda (3cm)
    
    //sprintf(buffer,"%d            %d           %d",total_flancos_left,vel_angular_left,vel_lineal_left);

    //Exteriorizacion de datos para probar el codigo
    Serial.print(total_flancos_left);
    Serial.print("\t\t");
    Serial.print(vel_angular_left);
    Serial.print("\t\t");
    Serial.println(vel_lineal_left);
     
    flag_calc_vel = false;
  }

}

/////////////////////////////////////////////RUTINAS DE SERVICIO/////////////////////////////////////////////////////////////////////////////////////////////////
void RS_ENCODER_LEFT() {    
  flancos_left++ ;
}

void RS_ENCODER_RIGHT() {    
  flancos_right++;
}

void RS_TIMER2() {
  total_flancos_left = flancos_left;
  total_flancos_right = flancos_right;

  flancos_left = 0;
  flancos_right = 0;

  flag_calc_vel = true;
}


/**
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor){
  
  byte mode;
  
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
    
  } else if(pin == 3 || pin == 11) {
      
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

