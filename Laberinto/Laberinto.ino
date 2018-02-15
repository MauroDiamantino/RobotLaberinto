//////////////////////////////////////////LIBRERIAS//////////////////////////////////////////////////////////////////////////////////////////////////
#include <FlexiTimer2.h>

//Declaraciones de los prototipos de las funciones con la directiva "inline", a las que se les ha asignado un atributo para forzar al compilador a
//respetar esta directiva
inline void Excitador(float&, float&) __attribute__((always_inline));
//inline void ParserParametroPID(String&, float&, const String) __attribute__((always_inline));
inline float CalcVelLeft() __attribute__((always_inline));
inline float CalcVelRight() __attribute__((always_inline));
inline void ActuadorLeft(float) __attribute__((always_inline));
inline void ActuadorRight(float) __attribute__((always_inline));
inline void ControlMotorLeft(float) __attribute__((always_inline));
inline void ControlMotorRight(float) __attribute__((always_inline));

//////////////////////////////////////////CONSTANTES//////////////////////////////////////////////////////////////////////////////////////////////////
//const float PI = 3.14; //Es innecesario definir esta constante porque ya esta predefinida
const unsigned int INT_MUESTREO = 1; //Intervalo de muestro en milisegundos, 1mS.
const byte SEMI_PER_EXCITACION = 100; //Semiperiodo (en mS) de la señal de excitacion que permite calibrar los PIDs. Se trata de una señal
                                     //cuadrada, generada localmente, que se aplica en las entradas de referencia (r). Tiempo: 50mS.
const unsigned int INT_CAMBIO_PARAM = 2000; //Intervalo de tiempo (en mS) de cambio de los parametreos de los controldadores PID. El micro debe revisar
                                            //el buffer de entrada serie cada vez que se cumple este intervalo de tiempo y actualizar los parametros de los
                                            //PIDs si se han enviado nuevos valores.
                                     
const unsigned long VEL_COM_SERIE = 115200;
const unsigned int NUM_FLANCOS_VUELTA = 16; //Cantidad de flancos efectivos de las señales de los encoders por cada vuelta de una reuda, es decir cantidad de
                                            //flancos por vuelta que producen interrupciones. Esto depende de la configuracion de las interrupciones del micro.
const float ZONA_MUERTA = 4.0; //Zona muerta de los motor, solo considerando tensiones positivas. En realidad la zona muerta es ±4v.
const float TOLERANCIA = 0.1; //Tolerancia para el control de la velocidad de las rueda, 0.1m/s. Este valor debería ser definido
                              //considerando que el hecho de que exista una diferencia en las velocidades de las ruedas producira
                              //una desviacion lateral (giro) del robot mientras este intenta moverse en linea recta, lo cual
                              //podria producir el impacto del mismo contra las paredes del laberinto si el valor no es elegido
                              //correctamente.
const float TENSION_MAX = 11.1; //Los motores tienen las siguientes caracteristicas: 18V, 11 Ohm y 8800RPM.
const float VEL_LIN_MAX = 1.0; //Velocidad lineal maxima (m/s) permitida para ambos motores.
const float RADIO_RUEDA = 0.03; //Radio de la ruedas en metros. Este es de 3cm, es decir 0.03m.

//Pines
const unsigned int PIN_ENCODER_LEFT = 2;
const unsigned int PIN_ENCODER_RIGHT = 3;
const unsigned int PIN1_MOTOR_LEFT = 5;
const unsigned int PIN2_MOTOR_LEFT = 6;
const unsigned int PIN1_MOTOR_RIGHT = 9;
const unsigned int PIN2_MOTOR_RIGHT = 10;

//////////////////////////////////////////VARIABLES GLOBALES//////////////////////////////////////////////////////////////////////////////////////////
//Contadores de los flancos efectivos (aquellos que producen interrupciones) de los encoder.
unsigned int flancos_left = 0;
unsigned int flancos_right = 0;

//Flancos efectivos contados por los encoder en un determinado lapso de tiempo, el intervalo de muestreo.
unsigned int total_flancos_left = 0;  
unsigned int total_flancos_right = 0;

//Contadores de tiempo (la resolucion es el intervalo de muestreo) que permiten ejecutar determinadas funciones 
//cada ciertos intervalos de tiempo mayores al intervalo de muestreo.
byte cont_excitador = SEMI_PER_EXCITACION; //Contador de tiempo del Excitador
unsigned int cont_parser = (unsigned int) (INT_CAMBIO_PARAM + INT_CAMBIO_PARAM/11); //Contador de tiempo del Parser de los parametros de los PIDs. El contador 
                                              //deberia ser inicializado con INT_CAMBIO_PARAM, debido a que esa constante representa el intervalo de tiempo cada 
                                              //cuanto se debe revisar el buffer serie para actualizar los parametros de los PIDs. Sin embargo, al inicializarlo 
                                              //con este valor diferente se busca generar un desfasaje con respecto a la fucion Excitador, para evitar la 
                                              //sobrecarga del micro.

//Parametros del PID del lazo de control L y referencia
float kp_l = 0.0, ki_l = 0.0, td_l = 0.0; //Parametros del PID
float r_left = 0.0; //velocidad lineal de referencia (m/s)

//Parametros del PID del lazo de control L y referencia
float kp_r = 0.0, ki_r = 0.0, td_r = 0.0; //Parametros del PID
float r_right = 0.0; //velocidad lineal de referencia (m/s)

//Buffer de la comunicacion serie
//char buffer_serie[100];
String buffer_serie;

///////////////////////////////////////////////FLAGs////////////////////////////////////////////////////////////////////////////////////////////////////
boolean flag_control = false;
boolean flag_excitador = false;
boolean flag_parser = false;

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

  if (flag_excitador == true){
    Excitador(r_left, r_right);

    flag_excitador = false;
  }

  if (flag_parser == true){
    if (Serial.available()!=0){
      while (Serial.available()!=0){  buffer_serie += Serial.read();  }
      buffer_serie.toLowerCase();
      ParserParametroPID(buffer_serie, kp_l, "kp_l");
      ParserParametroPID(buffer_serie, ki_l, "ki_l");
      ParserParametroPID(buffer_serie, td_l, "td_l");
      ParserParametroPID(buffer_serie, kp_r, "kp_r");
      ParserParametroPID(buffer_serie, ki_r, "ki_r");
      ParserParametroPID(buffer_serie, td_r, "td_r");
    }
    
    flag_parser = false;
  }
}

/////////////////////////////////////////////////////////RUTINAS DE SERVICIO////////////////////////////////////////////////////////////////////////////
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

  cont_excitador--;
  if (cont_excitador==0){
    cont_excitador = SEMI_PER_EXCITACION;
    flag_excitador=true;
  }

  cont_parser--;
  if (cont_parser==0){
    cont_parser = INT_CAMBIO_PARAM;
    flag_parser = true;
  }
}

////////////////////////////////////////////////////////////////////FUNCIONES//////////////////////////////////////////////////////////////////////////////
//Funcion que se encarga de generar una señal cuadrada para excitar ambos lazo de control (excitacion local) y asi poder calibrar los PIDs de los mismos. 
//La señal cuadrada se conecta a las entradas de referencia de ambos lazos (setpoints). Esta señal tiene un semiperiodo mucho mayor que el intervalo de muestreo,
//de modo que luego de que se produzca un flanco (cambio de setpoint) los lazos tengan suficiente tiempo para reducir los errores y estabilizar las salidas 
//de los mismos (velocidades).
inline void Excitador(float& r_left, float& r_right){ //r_left y r_right son referencias (punteros constantes que se de-referencian automáticamente, muy usado en C++).
  if (r_left==0){
    r_left = VEL_LIN_MAX;
  }else{
    r_left = 0;
  }

  if (r_right==0){
    r_right = VEL_LIN_MAX;
  }else{
    r_right = 0;
  }
}


void ParserParametroPID(String& buffer_serie, float& param, const String nombre_param){
  //Constantes
  const unsigned int LONG_SUBSTRING = 5; //Longitud del substring a buscar. Este contiene 4 caracteres correspondientes
                                        //al nombre del parametro en cuestion (todos los parametros tienen nombres con la misma
                                        //longitud) y ademas el caracter de asignacion '='
  const String aux2 = nombre_param + '='; //Se utiliza para formar el string a buscar en el buffer
  
  //Variables
  unsigned int num_chars; //Numero de caracteres en el buffer de entrada serie
  boolean flag_encontrado = false; //Flag que indica si el parametro fue hallado en el buffer o no
  unsigned int i = 0; //Indice de posicion
  String aux1 = ""; //Se utiliza para obtener el substring que representa el valor numerico del parametro
  unsigned int pos_inicial; //Almacena la posicion en el buffer a partir de la cual esta el parametro buscado,
                            //si fue hallado
  unsigned int cont; //Contador de caracteres a borrar en el buffer, una vez que se encuentra un parametro y 
                     //se extrae su valor numerico

  num_chars = buffer_serie.length();

  ////Busqueda del parametro en el string////
  //Lazo de busqueda
  while ( flag_encontrado==false && i<=(num_chars-LONG_SUBSTRING) ){
    if ( buffer_serie.substring(i,i+LONG_SUBSTRING)==aux2 ){  flag_encontrado = true; }
    i++;
  }
  i--;
  
  //Obtencion del valor del parametro, si este fue hallado
  if (flag_encontrado==true){
    pos_inicial= i; //Se almacena la posicion donde comienza el nombre del parametro
    i+=LONG_SUBSTRING; //Se hace avanzar el indice hasta la posicion donde empieza el valor numerico del parametro
    //Lazo de obtencion del valor numerico
    while ( buffer_serie[i]!=',' && buffer_serie[i]!=';' ){ 
      aux1+=buffer_serie[i];
      i++;
    }
    param = aux1.toFloat();

    //Borrado del parametro y su valor del buffer de entrada serie
    cont = i-pos_inicial+1; //Cantidad de caracteres a remover
    buffer_serie.remove(pos_inicial, cont);
  }
}

//Funcion que calcula la velocidad lineal de la rueda izquierda y retorna su valor
inline float CalcVelLeft(){
  float vel_angular_left = 0.0; //velocidad angular de las rueda izquierda
  float vel_lineal_left = 0.0; //velocidad lineal de la rueda izquierda
  //No se obtiene mejora si se usa el tipo de dato 'double' por la placa que se esta usando
  
  //Calculo de la velocidad lineal del motor izquierdo. Esta es la variable a controlar en el lazo de control L (y_left).
  vel_angular_left = ( (total_flancos_left/NUM_FLANCOS_VUELTA)*2.0*PI / (INT_MUESTREO/1000.0) );// La velocidad angular esta en rad/s. Se divide el intervalo de 
                                                                                              //muestreo en 1000 porque esta en milisegundos
  vel_lineal_left = vel_angular_left*RADIO_RUEDA; // La velocidad lineal esta en m/s, se obtiene al multiplicar la velocidad angular por el radio de la rueda
  
  return vel_lineal_left;
}


inline float CalcVelRight(){
  float vel_angular_right = 0.0; //velocidad angular de las rueda derecha
  float vel_lineal_right = 0.0; //velocidad lineal de la rueda derecha
  //no se obtiene mejora si se usa el tipo de dato 'double' por la placa que se esta usando

  //Calculo de la velocidad lineal del motor derecho. Esta es la variable a controlar en el lazo de control R (y_right).
  vel_angular_right = ( (total_flancos_right/NUM_FLANCOS_VUELTA)*2.0*PI / (INT_MUESTREO/1000.0) );// La velocidad angular esta en rad/s. Se divide el intervalo de 
                                                                                              //muestreo en 1000 porque esta en milisegundos
  vel_lineal_right = vel_angular_right*RADIO_RUEDA; // La velocidad lineal esta en m/s, se obtiene al multiplicar la velocidad angular por el radio de la rueda

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
    integral = integral1 + kp_l*ki_l*INT_MUESTREO*(e + e1)/2;
    //Termino derivativo
    deriv = td_l/(INT_MUESTREO + td_l) * (deriv1 - kp_l*(y - y1));
    u = kp_l*e + integral + deriv;

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
    integral = integral1 + kp_r*ki_r*INT_MUESTREO*(e + e1)/2;
    //Termino derivativo
    deriv = td_r/(INT_MUESTREO + td_r) * (deriv1 - kp_r*(y - y1));
    u = kp_r*e + integral + deriv;

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

