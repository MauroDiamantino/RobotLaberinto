#include "Laberinto.h"

//////////////////////////////////////////VARIABLES GLOBALES//////////////////////////////////////////////////////////////////////////////////////////
//Contadores de los flancos efectivos (aquellos que producen interrupciones) de los encoder.
unsigned int flancos_left = 0;
unsigned int flancos_right = 0;

//Flancos efectivos contados por los encoder en un determinado lapso de tiempo, el intervalo de muestreo.
unsigned int total_flancos_left = 0;  
unsigned int total_flancos_right = 0;

//Parametros del PID del lazo de control L y referencia
float kp_left = 0.0, ki_left = 0.0, td_left = 0.0; //Parametros del PID
float r_left = 0.0; //velocidad lineal de referencia (m/s)
                      
//Parametros del PID del lazo de control L y referencia
float kp_right = 0.0, ki_right = 0.0, td_right = 0.0; //Parametros del PID
float r_right = 0.0; //velocidad lineal de referencia (m/s)

//Buffer de la comunicacion serie
char buffer[100];

///////////////////////////////////////////////FLAGs////////////////////////////////////////////////////////////////////////////////////////////////////
boolean flag_control = false;

///////////////////////////////////////////////SETUP////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  
  //Configuración del modo de funcionamiento de los pines
  pinMode(PIN_ENCODER_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENCODER_RIGHT, INPUT_PULLUP);
  pinMode(PIN1_MOTOR_LEFT, OUTPUT);
  pinMode(PIN2_MOTOR_LEFT, OUTPUT);
  pinMode(PIN1_MOTOR_RIGHT, OUTPUT);
  pinMode(PIN2_MOTOR_RIGHT, OUTPUT);

  //Configuracion de la frecuencia de las salidas PWM
  setPwmFrequency(PIN1_MOTOR_LEFT, 8); //se configura la freq PWM en 8KHz aprox
  setPwmFrequency(PIN2_MOTOR_LEFT, 8); //se configura la freq PWM en 8KHz aprox
  setPwmFrequency(PIN1_MOTOR_RIGHT, 8); //se configura la freq PWM en 4KHz aprox
  setPwmFrequency(PIN2_MOTOR_RIGHT, 8); //se configura la freq PWM en 4KHz aprox
  //No es posible configurar las frecuencias PWM de ambos motores en el mismo valor
  //porque las frecuencias base de los pines son diferentes. Sin embargo, lo importante es
  //que las frecuencias sean suficientemente altas, no iguales.
  
  //Configuracion de la comunicación serie
  Serial.begin(VEL_COM_SERIE);
  
  //Configuracion de las interrupciones
  //Usar CHANGE si se desea que ambos flancos, negativo y positivo, produzcan interrupciones (sean efectivos), y configurar el numero de flancos efectivos
  //por vuelta (NUM_FLANCOS_VUELTA) en 16. Si se usa solo uno de los flancos, usar FALLING o RISING y 8 flancos efectivos por vuelta.
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT),RS_ENCODER_LEFT, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT),RS_ENCODER_RIGHT, CHANGE);

  //Configuración del Timer 2, que es utilizado para contar el intervalo de muestreo.
  FlexiTimer2::set(INT_MUESTREO, RS_TIMER2); // El tiempo de muestreo se coloca en milisegundos
  FlexiTimer2::start();
}

///////////////////////////////////////////////LOOP////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  
  if (flag_control == true){

    ControlMotorLeft(r_left);
    ControlMotorRight(r_right);
    
    flag_control = false;
  }
}

/////////////////////////////////////////////////////////RUTINAS DE SERVICIO/////////////////////////////////////////////////////////////////////////////////////////////////
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

  flag_control = true;
}

////////////////////////////////////////////////////////////////////FUNCIONES///////////////////////////////////////////////////////////////////////////////////////////////////
inline float CalcVelLeft(){
  float vel_angular_left = 0.0; //velocidad angular de las rueda izquierda
  float vel_lineal_left = 0.0; //velocidad lineal de la rueda izquierda
  //no se obtiene mejora si se usa el tipo de dato 'double' por la placa que se esta usando
  
  //Calculo de la velocidad lineal del motor izquierdo. Esta es la variable a controlar en el lazo de control L (y_left).
  vel_angular_left = ( (total_flancos_left/NUM_FLANCOS_VUELTA)*2.0*PI / (INT_MUESTREO/1000.0) );// La velocidad angular esta en rad/s. Se divide el intervalo de 
                                                                                              //muestreo en 1000 porque esta en milisegundos
  vel_lineal_left = vel_angular_left*0.03; // La velocidad lineal esta en m/s, se obtiene al multiplicar la velocidad angular por el radio de la rueda (3cm, es decir 0.03m)
  
  return vel_lineal_left;
}


inline float CalcVelRight(){
  float vel_angular_right = 0.0; //velocidad angular de las rueda derecha
  float vel_lineal_right = 0.0; //velocidad lineal de la rueda derecha
  //no se obtiene mejora si se usa el tipo de dato 'double' por la placa que se esta usando

  //Calculo de la velocidad lineal del motor derecho. Esta es la variable a controlar en el lazo de control R (y_right).
  vel_angular_right = ( (total_flancos_right/NUM_FLANCOS_VUELTA)*2.0*PI / (INT_MUESTREO/1000.0) );// La velocidad angular esta en rad/s. Se divide el intervalo de 
                                                                                              //muestreo en 1000 porque esta en milisegundos
  vel_lineal_right = vel_angular_right*0.03; // La velocidad lineal esta en m/s, se obtiene al multiplicar la velocidad angular por el radio de la rueda (3cm)

  return vel_lineal_right;
}


inline void ActuadorLeft(float u){
  byte pwm; //valor de PWM a aplicar en el pin digital correspondiente. Recordar que las salidas
            //PWM son de 8-bit en el Arduino Nano.
  
  if (u >= 0.0){ //voltaje medio de alimentacion del motor positivo (aceleracion)
    if (u!=0.0 && u<ZONA_MUERTA) u = ZONA_MUERTA;
    if (u > TENSION_MAX) u = TENSION_MAX;
    //Mapeo de la accion de control (voltaje) a PWM de 8 bits
    pwm = (byte)( (u/TENSION_MAX)*255.0 ); //representa el duty de la señal PWM
    //Aplicacion de la accion de control (PWM)
    digitalWrite(PIN1_MOTOR_LEFT, LOW);
    analogWrite(PIN2_MOTOR_LEFT,  pwm);
  } else { //voltaje medio de alimentacion del motor negativo (desaceleracion)
    u = abs(u);
    if (u < ZONA_MUERTA) u = ZONA_MUERTA;
    if (u > TENSION_MAX) u = TENSION_MAX;
    //Mapeo de la accion de control (voltaje) a PWM de 8 bits
    pwm = (byte)( (u/TENSION_MAX)*255.0 ); //representa el duty de la señal PWM
    //Aplicacion de la accion de control (PWM)
    digitalWrite(PIN2_MOTOR_LEFT, LOW);
    analogWrite(PIN1_MOTOR_LEFT,  pwm);
  }
}


inline void ActuadorRight(float u){
  byte pwm; //valor de PWM a aplicar en el pin digital correspondiente. Recordar que las
            //salidas PWM son de 8-bit en el Arduino Nano.

  if (u >= 0.0){ //voltaje medio de alimentacion del motor positivo (aceleracion)
    if (u!=0.0 && u<ZONA_MUERTA) u = ZONA_MUERTA;
    if (u > TENSION_MAX) u = TENSION_MAX;
    //Mapeo de la accion de control (voltaje) a PWM de 8 bits
    pwm = (byte)( (u/TENSION_MAX)*255.0 ); //representa el duty de la señal PWM
    //Aplicacion de la accion de control (PWM)
    digitalWrite(PIN1_MOTOR_RIGHT, LOW);
    analogWrite(PIN2_MOTOR_RIGHT,  pwm);
  } else { //voltaje medio de alimentacion del motor negativo (desaceleracion)
    u = abs(u);
    if (u < ZONA_MUERTA) u = ZONA_MUERTA;
    if (u > TENSION_MAX) u = TENSION_MAX;
    //Mapeo de la accion de control (voltaje) a PWM de 8 bits
    pwm = (byte)( (u/TENSION_MAX)*255.0 ); //representa el duty de la señal PWM
    //Aplicacion de la accion de control (PWM)
    digitalWrite(PIN2_MOTOR_RIGHT, LOW);
    analogWrite(PIN1_MOTOR_RIGHT,  pwm);
  }
}


inline void ControlMotorLeft(float r){ //r: velocidad lineal de referencia (m/s)
  //Variables relacionadas con el lazo de control L
  float y; //velocidad lineal sensada (m/s)
  float e; //error de velocidad (m/s)
  float u; //accion de control, voltaje medio a aplicar en el motor, que se materializa como una señal PWM que
                          //se aplica al driver del motor
  float integral; //termino integral de la ecuacion del PID
  float deriv; //termino derivativo de la ecuacion del PID
  static float y1 = 0.0, e1 = 0.0, u1 = 0.0; //y1 = y(k-1); e1=e(k-1); u1 = u(k-1), u1 solo se utiliza para graficar la accion de control
  static float integral1 = 0.0, deriv1 = 0.0; //integral1 = integral(k-1); deriv1 = deriv(k-1)
  
  ////Lazo L////
  //Obtencion del valor medido de la salida
  y = CalcVelLeft();

  //Calculo del error
  e = r - y;

  if (e > TOLERANCIA){
    //Calculo de la accion de control del lazo L
    //Termino integrativo
    integral = integral1 + kp_left*ki_left*INT_MUESTREO*(e + e1)/2;
    //Termino derivativo
    deriv = td_left/(INT_MUESTREO + td_left) * (deriv1 - kp_left*(y - y1));
    u = kp_left*e + integral + deriv;

    ActuadorLeft(u);
    
    //Actualizacion de variables
    y1 = y;
    e1 = e;
    u1 = u;
    integral1 = integral;
    deriv1 = deriv;
  }
}


inline void ControlMotorRight(float r){
  //Variables relacionadas con el lazo de control R
  float y; //velocidad lineal sensada (m/s)
  float e; //error de velocidad (m/s)
  float u; //accion de control, voltaje medio a aplicar en el motor, que se materializa como una señal PWM que
                          //se aplica al driver del motor
  float integral; //termino integral de la ecuacion del PID
  float deriv; //termino derivativo de la ecuacion del PID
  static float y1 = 0.0, e1 = 0.0, u1 = 0.0; //y1 = y(k-1); e1=e(k-1); u1 = u(k-1), u1 solo se utiliza para graficar la accion de control
  static float integral1 = 0.0, deriv1 = 0.0; //integral1 = integral(k-1); deriv1 = deriv(k-1)
  
  ////Lazo R////
  //Obtencion del valor medido de la salida
  y = CalcVelRight();

  //Calculo del error
  e = r - y;

  if (e > TOLERANCIA){
    //Calculo de la accion de control del lazo R
    //Termino integrativo
    integral = integral1 + kp_right*ki_right*INT_MUESTREO*(e + e1)/2;
    //Termino derivativo
    deriv = td_right/(INT_MUESTREO + td_right) * (deriv1 - kp_right*(y - y1));
    u = kp_right*e + integral + deriv;

    ActuadorRight(u);

    //Actualizacion de variables
    y1 = y;
    e1 = e;
    u1 = u;
    integral1 = integral;  
    deriv1 = deriv;
  }
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

