//////////////////////////////////////////LIBRERIAS//////////////////////////////////////////////////////////////////////////////////////////////////
#include <FlexiTimer2.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#define DEBUG 0
#define LAZO_A_CALIBRAR 'L' //debe ser 'R' o 'L'

////////////Pines////////////////////
const byte PIN_ENCODER_LEFT1 = A6;//4
const byte PIN_ENCODER_LEFT2 = A7;//7
const byte PIN_ENCODER_RIGHT1 = 4;
const byte PIN_ENCODER_RIGHT2 = 7;
const byte PIN1_MOTOR_LEFT = 9;//5
const byte PIN2_MOTOR_LEFT = 10;//6
const byte PIN1_MOTOR_RIGHT = 5;
const byte PIN2_MOTOR_RIGHT = 6;
const byte PIN_EXCITADOR_ONOFF = 12;

struct RespuestaControlador{
  float y;  //Valor obtenido del sensor de la variable de salida
  float u_p;  //Accion de control generada a partir del error y aplicada a la planta
  float prop; //Parte proporcional de la accion de control
  float integral; //Parte integral de la accion de control
  float deriv; //Parte derivativa de la accion de control
};

//Declaraciones de los prototipos de las funciones con la directiva "inline", a las que se les asigna un atributo para
//forzar al compilador a respetar esta directiva.
inline void Excitador(float&, float&) __attribute__((always_inline));
inline float CalcVelLineal() __attribute__((always_inline));
inline float Actuador(float, const unsigned int, const unsigned int) __attribute__((always_inline));
inline RespuestaControlador ControlMotorLeft(float) __attribute__((always_inline));
inline RespuestaControlador ControlMotorRight(float) __attribute__((always_inline));

//////////////////////////////////////////CONSTANTES//////////////////////////////////////////////////////////////////////////////////////////////////
//const float PI = 3.14; //Es innecesario definir esta constante porque ya esta predefinida
const float INT_MUESTREO = 0.01; //Intervalo de muestro en segundos, 10mS.
const unsigned int SEMI_PER_EXCITACION = (unsigned int) 5000/(1000*INT_MUESTREO); //Semiperiodo (en mS) de la señal de excitacion que permite calibrar los PIDs. 
                                              //Se trata de una señal cuadrada, generada localmente, que se aplica en las entradas de referencia (r). 5mS.
const unsigned int INT_CAMBIO_PARAM = (unsigned int) 5000/(1000*INT_MUESTREO); //Intervalo de tiempo (en mS) para el cambio de los parametreos de los controldadores PID.
                                            //El micro debe revisar el buffer de entrada serie cada vez que se cumple este intervalo de tiempo y actualizar 
                                            //los parametros de los PIDs si se han enviado nuevos valores. 5mS.
                                     
const unsigned long VEL_COM_SERIE = 115200; //Velocidad de la comunicacion serie
const unsigned int NUM_FLANCOS_VUELTA = 288; //Cantidad de flancos efectivos de las señales de los encoders por cada vuelta de una reuda, es decir cantidad de
                                            //flancos por vuelta que producen interrupciones. Esto depende de la configuracion de las interrupciones del micro.
const float ZONA_MUERTA_R = 4.0; //Zona muerta del motor derecho (en volts), solo considerando tensiones positivas. En realidad la zona muerta es ±4v.
const float ZONA_MUERTA_L = 4.0; //Zona muerta del motor derecho (en volts), solo considerando tensiones positivas. En realidad la zona muerta es ±4v.
//const float TOLERANCIA = 0.01; //Tolerancia para el control de la velocidad de las ruedas, 0.01m/s. Este valor debería ser definido considerando que el hecho de
                              //que exista una diferencia en las velocidades de las ruedas producira una desviacion lateral (giro) del robot mientras este
                              //intenta moverse en linea recta, lo cual podria producir el impacto del mismo contra las paredes del laberinto si el
                              //valor no es elegido correctamente.
const float TENSION_MAX = 11.1; //Los motores tienen las siguientes caracteristicas: 18V, 11 Ohm y 8800RPM.
const float VEL_LIN_MAX = 1.0; //Velocidad lineal maxima (m/s) permitida para ambos motores.
const float VEL_LIN_MIN = 0.14; //Velocidad lineal minima (m/s) permitida para ambos motores.
const float RADIO_RUEDA = 0.03; //Radio de la ruedas en metros. Este es de 3cm, es decir 0.03m.
const unsigned int PERIODO_CAPT = (unsigned int) 30/(1000*INT_MUESTREO); //Periodo de captura de muestras de distintas variables a graficar en Simulink. En ms.

//////////////////////////////////////////VARIABLES GLOBALES//////////////////////////////////////////////////////////////////////////////////////////
//Contadores de los flancos efectivos (aquellos que producen interrupciones) de los encoder.
volatile unsigned int flancos_left = 0; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]
volatile unsigned int flancos_right = 0; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]

//Flancos efectivos contados por los encoder en un determinado lapso de tiempo, el intervalo de muestreo.
volatile unsigned int total_flancos_left = 0; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]
volatile unsigned int total_flancos_right = 0; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]

//Parametros del PID del lazo de control L y referencia
float kp_l = 3.0, ki_l = 0.006, td_l = 0.005; //Parametros del PID
float r_left = 0.0; //Velocidad lineal de referencia (m/s)

//Parametros del PID del lazo de control L y referencia
float kp_r = 0.0, ki_r = 0.0, td_r = 0.0; //Parametros del PID
float r_right = 0.0; //Velocidad lineal de referencia (m/s)

//Buffer de la comunicacion serie
String buffer_serie;

///////////////////////RX Y TX////////////////////////////////////////
byte cantidad_sensores = 6;
byte cantidad_actuadores = 1;
//BUFFERS
unsigned int envio[20];
byte inData[20]; //buffer temporal de lectura. No usar
byte inData1[20]; //buffer final de lectura.Datos crudos.
float actuador[20]; //buffer final de datos de actuadores. Datos en formato 15 bit. 
float sensor[20]; //buffer final de escritura
float Ksensor = 1.0; //mapeo
float Kactuador = 1.0; //mapeo
int contador = 0;
int encabezado = 0, i = 0, j = 0, k = 0;
bool evento_tx = 0, evento_rx = 0, evento_captura=0; //banderas de eventos-tareas
byte modo = 0;
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////FLAGs////////////////////////////////////////////////////////////////////////////////////////////////////
volatile bool flag_control = false; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]
volatile bool flag_excitar = false; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]
volatile bool flag_excitador_on = false; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]
volatile bool flag_parser = false; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]

///////////////////////////////////////////////SETUP////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  //Configuración del modo de funcionamiento de los pines
  pinMode(PIN_ENCODER_LEFT1, INPUT);
  pinMode(PIN_ENCODER_LEFT2, INPUT);
  pinMode(PIN_ENCODER_RIGHT1, INPUT);
  pinMode(PIN_ENCODER_RIGHT2, INPUT);
  pinMode(PIN1_MOTOR_LEFT, OUTPUT);
  pinMode(PIN2_MOTOR_LEFT, OUTPUT);
  pinMode(PIN1_MOTOR_RIGHT, OUTPUT);
  pinMode(PIN2_MOTOR_RIGHT, OUTPUT);
  pinMode(PIN_EXCITADOR_ONOFF, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

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
  //Usar CHANGE si se desea que ambos flancos, negativo y positivo, produzcan interrupciones (es decir sean efectivos), y configurar el numero de flancos 
  //efectivos por vuelta (NUM_FLANCOS_VUELTA) en 288. Si se usa solo uno de los flancos, usar FALLING o RISING y 144 flancos efectivos por vuelta.
  attachPinChangeInterrupt(digitalPinToPCINT(PIN_ENCODER_LEFT1), RS_ENCODER_LEFT, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(PIN_ENCODER_LEFT2), RS_ENCODER_LEFT, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(PIN_ENCODER_RIGHT1), RS_ENCODER_RIGHT, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(PIN_ENCODER_RIGHT2), RS_ENCODER_RIGHT, CHANGE);

  //Configuración del Timer 2, que es utilizado para contar el intervalo de muestreo.
  FlexiTimer2::set(1000*INT_MUESTREO, RS_TIMER2); // El tiempo de muestreo se coloca en milisegundos
  FlexiTimer2::start();
}

///////////////////////////////////////////////LOOP////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  RespuestaControlador respControladorL = {0.0,0.0,0.0,0.0,0.0};
  RespuestaControlador respControladorR = {0.0,0.0,0.0,0.0,0.0};
  
  if (flag_control == true){
    respControladorL = ControlMotorLeft(r_left);
    respControladorR = ControlMotorRight(r_right);
#if DEBUG==1
    //Se envian por el puerto serie diferentes variables de interes para evualuar en forma visual el desempeño de los controladores.
    Serial.print(" r_l=");
    Serial.print(r_left);
    Serial.print(" tot_flan_l=");
    Serial.print(total_flancos_left);
    Serial.print(" y_l=");
    Serial.print(respControladorL.y);
    Serial.print(" u_pl=");
    Serial.println(respControladorL.u_p);
#endif
   
    flag_control = false;
  }

#if DEBUG==0
  if (evento_captura==true){

    //Se envian por el puerto serie las muestras de las velocidades reales para evaluar la evolucion de las mismas en MatLab y así poder calibrar los PIDs.
    //Ademas se envian las muestras de las referencias, la accion de control total y las distancias componentes de la misma.
//    char bufferOut[100];
//  #if LAZO_A_CALIBRAR=='L'
//    //sprintf(bufferOut,"%d,%d,%d,%d,%d,%d",int(100*r_left),int(100*respControladorL.y),int(100*respControladorL.u_p), int(100*respControladorL.prop), int(100*respControladorL.integral),int(100*respControladorL.deriv));
//    sprintf(bufferOut,"%d%d%d%d%d",int(100*r_left),int(100*respControladorL.y), int(100*respControladorL.prop), int(100*respControladorL.integral),int(100*respControladorL.deriv));
//    Serial.print(bufferOut);
//  #else
//    //sprintf(bufferOut,"%d,%d,%d,%d,%d,%d",int(100*r_right),int(100*respControladorR.y),int(100*respControladorR.u_p), int(100*respControladorR.prop), int(100*respControladorR.integral),int(100*respControladorR.deriv));
//    Serial.print(bufferOut);
//  #endif
    #if LAZO_A_CALIBRAR=='L'
      sensor[0]=r_left;
      sensor[1]=respControladorL.y;
      sensor[2]=respControladorL.u_p;
      sensor[3]=respControladorL.prop;
      sensor[4]=respControladorL.integral;
      sensor[5]=respControladorL.deriv;
    #else
      sensor[0]=r_right;
      sensor[1]=respControladorR.y;
      sensor[2]=respControladorR.u_p;
      sensor[3]=respControladorR.prop;
      sensor[4]=respControladorR.integral;
      sensor[5]=respControladorR.deriv;
    #endif
  }
#endif


  if ( digitalRead(PIN_EXCITADOR_ONOFF)==LOW ){
    flag_excitador_on = true;
  }else{
    digitalWrite(LED_BUILTIN, LOW);
    flag_excitador_on = false;
  }

  if (flag_excitar == true){
    Excitador(r_left, r_right);
    
    flag_excitar = false;
  }

  if (flag_parser == true){
    unsigned int aux = Serial.available();
    
    if (aux != 0){
      buffer_serie.reserve(aux);
      buffer_serie = Serial.readStringUntil('\n');
      buffer_serie.toLowerCase();
   
#if DEBUG
      if ( !ParserParametrosPID(buffer_serie, kp_l, ki_l, td_l, kp_r, ki_r, td_r); ){
        Serial.println("El formato del string recibido no cumple con el formato esperado:");
        Serial.println("\t\"kp_x=x.xx; ki_x=x.xx; td_x=x.xx;\\n\"");
      }
#else
      ParserParametrosPID(buffer_serie, kp_l, ki_l, td_l, kp_r, ki_r, td_r);
#endif    
      buffer_serie = ""; //Se borra el contenido del buffer
    }
    flag_parser = false;
  }

  //*************************************************************************************
  //*************** EVENTO DETECCION DE TRAMA   *************************************
  //*************************************************************************************
  if (evento_rx == 1) {
    modo = inData1[0];                // RECIBE MODO
    cantidad_actuadores = inData1[1]; // RECIBE CANTIDAD DE AcTUADORES
    cantidad_sensores = inData1[2];   // RECIBE CANTIDAD DE SENSORES
    for (k = 0; k < cantidad_actuadores; k++) {
      actuador[k] = ((float)256.00 * (0x7F & inData1[2 * k + 4]) + inData1[2 * k + 3]); //arma dato de 15 bits de magnitud
      if ((inData1[2 * k + 4] & 0x80) > 0) { // signo en bit 16
        actuador[k] = Kactuador * actuador[k] * (-1.0);
      }
    }
    evento_rx = 0; // RESET DE BANDERA RX TRAMA
    evento_tx = 1; // ACTIVA BANDERA DE TX
  }

  //*************************************************************************************
  //*************** XXX  TRANSMIION DE DATOS  XXX  *************************************
  //*************************************************************************************
  if (evento_tx == 1)
  {
    Serial.write(255);    // envia cabecera FF FF
    Serial.write(255);    // envia cabecera FF FF
    Serial.write(modo);   //envia modo
  
    for (k = 0; k < cantidad_sensores; k++) {
      envio[k] =  (unsigned int)(fabs(sensor[k]) * Ksensor) & 0x7FFF;
      if (sensor[k] < 0) {
        envio[0] = envio[0] | 0x8000;
      }
      Serial.write(highByte(envio[k]));
      Serial.write(lowByte(envio[k]));
    }
    Serial.write(0);       //fin trama
    evento_tx = 0;
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
  //Contadores de tiempo (la resolucion es el intervalo de muestreo) que permiten ejecutar determinadas funciones 
  //cada ciertos intervalos de tiempo mayores al intervalo de muestreo.
  static unsigned int cont_excitador = SEMI_PER_EXCITACION; //Contador de tiempo del Excitador.
  static unsigned int cont_parser = (unsigned int) (INT_CAMBIO_PARAM + INT_CAMBIO_PARAM/11); //Contador de tiempo del Parser de los parametros de los PIDs. El contador 
                                              //deberia ser inicializado con INT_CAMBIO_PARAM, debido a que esa constante representa el intervalo de tiempo cada 
                                              //cuanto se debe revisar el buffer serie para actualizar los parametros de los PIDs. Sin embargo, al inicializarlo 
                                              //con este valor diferente se busca generar un desfasaje con respecto a la funcion Excitador, para evitar la 
                                              //sobrecarga del micro.
  static unsigned int cont_capt = PERIODO_CAPT;
  
  total_flancos_left = flancos_left;
  total_flancos_right = flancos_right;

  flancos_left = 0;
  flancos_right = 0;

  flag_control = true;

  cont_capt--;
  if (cont_capt<=0){
    cont_capt=PERIODO_CAPT;
    evento_captura=true;
  }
  
  if ( flag_excitador_on ){
    cont_excitador--;
    if (cont_excitador<=0){
      cont_excitador = SEMI_PER_EXCITACION;
      flag_excitar = true;
    }
  }

  cont_parser--;
  if (cont_parser<=0){
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
  static bool estado = false;

  if (estado==false){
    r_left = VEL_LIN_MAX/3.0;
    r_right = VEL_LIN_MAX/3.0;
  }else{
    r_left = 2.0*VEL_LIN_MAX/3.0;
    r_right = 2.0*VEL_LIN_MAX/3.0;
  }

  estado = !estado;
  digitalWrite(LED_BUILTIN, estado);
}


//Funcion que se encarga de parsear (analizar un string y obtener datos del mismo) un string recibido por el puerto serie, para obtener los valores de
//los parametros de los controladores PID. La estructura (o formato) que debe tener el string es la siguiente:
//    "kp_x=x.xx; ki_x=x.xx; td_x=x.xx;\n"
//Donde la 'x' representa caracteres variables. En el caso de las x's que siguen a los guiones bajos, solo pueden ser 'l'(ele) o 'r'. El resto de las x's solo pueden ser
//numeros. La cantidad de digitos fraccionarios puede variar. La funcion primero controla el formato del string y si es correcto extrae los valores
//y se los asigna a los parametros correspondientes. En este caso, responde mostrando los valores asignados y retorna con una valor booleano 'ALTO'. En caso de que 
//el formato sea incorrecto retorna con una valor booleano 'BAJO'.
boolean ParserParametrosPID(String& buffer_serie, float& kp_l, float& ki_l, float& td_l, float& kp_r, float& ki_r, float& td_r){
  char aux_lazo;
  unsigned int num_chars; //Numero de caracteres en el buffer de entrada serie
  unsigned int i = 0; //Indice de posicion en el string
  boolean flag_correcto = false; //Flag que indica si el string recibido tiene una esctructura correcta
  unsigned int pos_inic[3];
  unsigned int pos_final[3];

  //Analisis del formato: Se analiza si el formato es correcto. Solo se extraen valores si el formato es correcto.
  num_chars = buffer_serie.length();

  if (num_chars >= 29){
    
    //Kp
    if ( buffer_serie.substring(0,3)=="kp_" ){
      aux_lazo = buffer_serie[3];
      
      if( aux_lazo=='l' || aux_lazo=='r' ){
        
        if ( buffer_serie[4]=='=' ){
          pos_inic[0] = 5; //posicion inicial del valor del primer parametro
          i = 5;
          
          if ( ControlParteEntera(i, buffer_serie) ){
            i++; //i estaba apuntando al '.', se incremente para que señale al primer digito fraccionario
            
            if( ControlParteFrac(i, buffer_serie) ){
              pos_final[0] = i; //posicion final del valor del primer parametro +1
              i++; //i estaba apuntando al ';', se incremente para que señale al caracter espacio

                //Ki
                if ( buffer_serie[i]==' ' && buffer_serie.substring(i+1,i+4)=="ki_" && buffer_serie[i+4]==aux_lazo && buffer_serie[i+5]=='=' ){
                  i = i+6;
                  pos_inic[1] = i; //posicion inicial del valor del segundo parametro
                  
                  if ( ControlParteEntera(i, buffer_serie) ) {
                    i++; //i estaba apuntando al '.', se incremente para que señale al primer digito fraccionario
                    
                    if( ControlParteFrac(i, buffer_serie) ){
                      pos_final[1] = i; //posicion final del valor del segundo parametro +1
                      i++; //i estaba apuntando al ';', se incremente para que señale al caracter espacio

                      //Td
                      if ( buffer_serie[i]==' ' && buffer_serie.substring(i+1,i+4)=="td_" && buffer_serie[i+4]==aux_lazo && buffer_serie[i+5]=='=' ){
                        i = i+6;
                        pos_inic[2] = i; //posicion inicial del valor del tercer parametro
                        
                        if ( ControlParteEntera(i, buffer_serie) ) {
                          i++; //i estaba apuntando al '.', se incremente para que señale al primer digito fraccionario
                          
                          if( ControlParteFrac(i, buffer_serie) ){
                            pos_final[2] = i; //posicion final del valor del tercer parametro +1
                            
                            flag_correcto = true; //Si se llega a este punto es que el formato del string es correcto!
                          }
                        }
                      }
                     }
                  }
                }   
            }
          }
        }
      }
    }
  }
  //Obtencion y asignacion: se obtiene los valores numericos de las posiciones correspondientes
  //y asignacion a los parametros del lazo L o del lazo R
  
  if ( flag_correcto==true ){
    String aux;
    if ( aux_lazo=='l' ){
      aux = buffer_serie.substring(pos_inic[0], pos_final[0]);
      kp_l = aux.toFloat();
      aux = buffer_serie.substring(pos_inic[1], pos_final[1]);
      ki_l = aux.toFloat();
      aux = buffer_serie.substring(pos_inic[2], pos_final[2]);
      td_l = aux.toFloat();
#if DEBUG==1
      Serial.print("Asignado: kp_l=");
      Serial.print(kp_l);
      Serial.print("; ki_l=");
      Serial.print(ki_l);
      Serial.print("; td_l=");
      Serial.print(td_l);
      Serial.println(';');
//#else
//      char bufferOut[100];
//      sprintf(bufferOut,"%d,%d,%d",int(100*kp_l),int(100*ki_l),int(100*td_l));
//      Serial.print(bufferOut);
#endif
    } else {
      aux = buffer_serie.substring(pos_inic[0], pos_final[0]);
      kp_r = aux.toFloat();
      aux = buffer_serie.substring(pos_inic[1], pos_final[1]);
      ki_r = aux.toFloat();
      aux = buffer_serie.substring(pos_inic[2], pos_final[2]);
      td_r = aux.toFloat();
#if DEBUG==1
      Serial.print("Asignado: kp_r=");
      Serial.print(kp_r);
      Serial.print("; ki_r=");
      Serial.print(ki_r);
      Serial.print("; td_r=");
      Serial.print(td_r);
      Serial.println(';');
//#else
//      char bufferOut[100];
//      sprintf(bufferOut,"%d,%d,%d",int(100*kp_r),int(100*ki_r),int(100*td_r));
//      Serial.print(bufferOut);
#endif
    }
  }
  
  return (flag_correcto);
}


//Funcion que controla el formato de la parte fraccionaria de los numeros del string recibido por el puerto serie.
//Es utilizada en la funcion ParserParametrosPID().
boolean ControlParteEntera(unsigned int& i, String& buffer_serie){
  boolean flag_error = false;
  
  while ( flag_error==false && buffer_serie[i]!='.' ){
    if ( buffer_serie[i]<'0' && buffer_serie[i]>'9' )   flag_error = true;
    i++;
  }

  return !flag_error;
}


//Funcion que controla el formato de la parte fraccionaria de los numeros del string recibido por el puerto serie.
//Es utilizada en la funcion ParserParametrosPID().
boolean ControlParteFrac(unsigned int& i, String& buffer_serie){
  boolean flag_error = false;
  
  while ( flag_error==false && buffer_serie[i]!=';' ){
    if ( buffer_serie[i]<'0' && buffer_serie[i]>'9' )   flag_error = true;
    i++;
  }

  return !flag_error;
}


//Funcion que calcula la velocidad lineal de una rueda (cualquiera de las dos) y retorna su valor. El calculo se basa en la cantidad total de flancos 
//(parametro de la funcion) medidos por el encoder durante un intervalo de muestreo. El algoritmo requiere del conocimiento del intervalo de muestreo
//(INT_MUESTREO), el numero de flancos efectivos por vuelta (NUM_FLANCOS_VUELTA) y el radio de las ruedas (RADIO_RUEDA).
inline float CalcVelLineal(unsigned int total_flancos){
  float vel_angular = 0.0; //velocidad angular
  float vel_lineal = 0.0; //velocidad lineal
  //No se obtiene mejora si se usa el tipo de dato 'double' por la placa que se esta usando
  
  //Calculo de la velocidad lineal del motor en cuestion
  vel_angular = ( float(total_flancos) / NUM_FLANCOS_VUELTA )*2.0*PI / ( float(INT_MUESTREO) );// La velocidad angular esta en rad/s. Se divide el intervalo de 
                                                                                      //muestreo en 1000 porque esta en milisegundos
  vel_lineal = vel_angular*RADIO_RUEDA; // La velocidad lineal esta en m/s, se obtiene al multiplicar la velocidad angular por el radio de la rueda
  
  return vel_lineal;
}


inline float Actuador(float u, const unsigned int PIN1_MOTOR, const unsigned int PIN2_MOTOR, const float ZONA_MUERTA){
  byte salidaPWM; //valor de PWM a aplicar en el pin digital correspondiente. Recordar que las salidas
            //PWM son de 8-bit en el Arduino Nano.
  
  if (u >= 0.0){ //voltaje medio de alimentacion del motor positivo
    if (u !=0.0 ){  u = abs(u) + ZONA_MUERTA; }
    if (u > TENSION_MAX){ u = TENSION_MAX;  }
    //Mapeo de la accion de control (voltaje) a PWM de 8 bits
    salidaPWM = (byte)( (u/TENSION_MAX)*255.0 ); //representa el duty de la señal PWM
    //Aplicacion de la accion de control (PWM)
    digitalWrite(PIN1_MOTOR, LOW);
    analogWrite(PIN2_MOTOR,  salidaPWM);
    return u;
  } else { //voltaje medio de alimentacion del motor negativo
    u = abs(u) + ZONA_MUERTA;
    if (u > TENSION_MAX){ u = TENSION_MAX;  }
    //Mapeo de la accion de control (voltaje) a PWM de 8 bits
    salidaPWM = (byte)( (u/TENSION_MAX)*255.0 ); //representa el duty de la señal PWM
    //Aplicacion de la accion de control (PWM)
    digitalWrite(PIN2_MOTOR, LOW);
    analogWrite(PIN1_MOTOR,  salidaPWM);
    return (-u);
  }
}


inline RespuestaControlador ControlMotorLeft(float r){ //r: velocidad lineal de referencia (m/s)
  //Constantes
  const float KB = 1.0;
  const byte N = 2; //Razon entre la constante de tiempo de la accion derivativa y la constante de tiempo del filtro en cascada.
  const byte N_filt = 10;
  //Variables del lazo de control L
  float y; //velocidad lineal sensada (m/s)
  static float reg_filt[N_filt];
  float sum=0.0;
  float y_filt;
  float e; //error de velocidad (m/s)
  float u, u_p; //acciones de control, antes y despues del actuador
  float integral; //termino integral de la ecuacion del PID
  float deriv; //termino derivativo de la ecuacion del PID
  float prop; ////termino derivativo de la ecuacion del PID
  static float y_filt1 = 0.0, e1 = 0.0; //y1 = y(k-1); e1=e(k-1);
  static float integral1 = 0.0, deriv1 = 0.0; //integral1 = integral(k-1); deriv1 = deriv(k-1)
  static float antiwindup = 0.0;
  RespuestaControlador respControlador;
  
  ////Lazo L////
  //Obtencion del valor real de la salida
  y = CalcVelLineal(total_flancos_left);

  //Filtrado de la velocidad sensada con un promediador
  for (unsigned int i=N_filt-1;i>0;i--){
    reg_filt[i]=reg_filt[i-1];
    sum+=reg_filt[i];
  }
  reg_filt[0]=y;
  sum+=y;
  y_filt = sum/N_filt;
   
  //Calculo del error
  e = r - y_filt;
#if DEBUG
  Serial.print(" e_l=");
  Serial.print(e);
#endif

  //Calculo de la accion de control del lazo L, siguiendo la estructura de un PID modificado de modo que la accion derivativa se
  //genera a partir de la salida de lazo (y). Ademas se implementa un filtro pasa-bajo en cascada con la accion derivativa.
  //Termino integrativo, con aproximacion rectangular y con la tecnica antiwindup "back-calculation"
  //integral = integral1 + kp_l*ki_l*INT_MUESTREO*(e + e1)/2.0;
  integral = integral1 + INT_MUESTREO*(kp_l*ki_l*e1 + antiwindup); //El valor de la variable "antiwindup" corresponde al intervalo de muestro anterior
  //Termino derivativo (se implementa con un filtro pasa-bajo en cascada)
  deriv = td_l/(N*INT_MUESTREO + td_l) * (deriv1 - kp_l*N*(y_filt - y_filt1));
  //Termino proporcional
  prop = kp_l*e;
  //Accion de control total
  u = prop + integral + deriv;

  //Aplicacion de la señal de control generada por el PID
  u_p = Actuador(u, PIN1_MOTOR_LEFT, PIN2_MOTOR_LEFT, ZONA_MUERTA_L);

  //Calculo de la variable antiwindup, que se usara en el calculo de la accion de control en el prox int de muestreo
  antiwindup = KB*(u_p - u);

  //Actualizacion de variables
  y_filt1 = y_filt;
  e1 = e;
  integral1 = integral;
  deriv1 = deriv;

  //Se retorna el valor medido de la salida del lazo y la accion de control, con sus terminos separados, para que sean graficados
  respControlador.y = y_filt;
  respControlador.u_p = u_p;
  respControlador.prop = prop;
  respControlador.integral = integral;
  respControlador.deriv = deriv;
  return respControlador;
}


inline RespuestaControlador ControlMotorRight(float r){
  //Variables relacionadas con el lazo de control R
  float y; //velocidad lineal sensada (m/s)
  float e; //error de velocidad (m/s)
  float u; //accion de control, voltaje medio a aplicar en el motor, que se materializa
          //como una señal PWM que se aplica al driver del motor
  float integral; //termino integral de la ecuacion del PID
  float deriv; //termino derivativo de la ecuacion del PID
  float prop; ////termino derivativo de la ecuacion del PID
  static float y1 = 0.0, e1 = 0.0; //y1 = y(k-1); e1=e(k-1);
  static float integral1 = 0.0, deriv1 = 0.0; //integral1 = integral(k-1); deriv1 = deriv(k-1)
  const byte N = 2; //Razon entre la constante de tiempo de la accion derivativa y la constante de tiempo del filtro en cascada.
  RespuestaControlador respControlador;
  
  ////Lazo R////
  //Obtencion del valor real de la salida
  y = CalcVelLineal(total_flancos_right);

  //Calculo del error
  e = r - y;

  //Calculo de la accion de control del lazo R, siguiendo la estructura de un PID modificado de modo que la accion derivativa se
  //genera a partir de la salida de lazo (y). Ademas se implementa un filtro pasa-bajo en cascada con la accion derivativa.
  //Termino integrativo
  integral = integral1 + kp_r*ki_r*INT_MUESTREO*(e + e1)/2.0;
  //Termino derivativo (se implementa con un filtro pasa-bajo en cascada)
  deriv = td_r/(N*INT_MUESTREO + td_r) * (deriv1 - kp_r*N*(y - y1));
  //Termino proporcional
  prop = kp_l*e;
  //Accion de control total
  u = prop + integral + deriv;

  //Aplicacion de la señal de control generada por el PID
  Actuador(u, PIN1_MOTOR_RIGHT, PIN2_MOTOR_RIGHT, ZONA_MUERTA_R);
  
  //Actualizacion de variables
  y1 = y;
  e1 = e;
  integral1 = integral;  
  deriv1 = deriv;

  //Se retorna el valor medido de la salida del lazo y la accion de control para que sean graficados
  respControlador.y = y;
  respControlador.y = u;
  respControlador.prop = prop;
  respControlador.integral = integral;
  respControlador.deriv = deriv;
  return respControlador;
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
      TCCR0B = TCCR0B & (0b11111000 | mode);
    } else {
      TCCR1B = TCCR1B & (0b11111000 | mode);
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
    
    TCCR2B = TCCR2B & (0b11111000 | mode);
  }
}

//*************************************************************************************
//********** SERVICIO DE INTERRUPCION  ISR(SERIAL_RX)   *********************
//*************************************************************************************
void serialEvent() {
  inData[i] = Serial.read();  //LEE UN BYTE
  //******************** || SEPARANDO LOS DADOS RECIBIDOS || *************************
  if (encabezado < 2)
  {
    if (inData[i] == 255)//enganche de cabecera
    {
      encabezado = encabezado + 1;
    } else {//si no engancha cabecera sigue intentando 
      i = 0;
      encabezado = 0;
    }
  } else {//si recien engancho cabecera encabezado=2,i=1 
    i = i + 1; 
    if (i <= 3) {
      if (i == 1) {
        modo = inData[0];
      }
      if (i == 2) {
        cantidad_actuadores = inData[1];
      }
      if (i == 3) {
        cantidad_sensores = inData[2];
      }
    } else {
      if (i >= (4 + 2 * cantidad_actuadores))
      {
        if (inData[3 + 2 * cantidad_actuadores] == 0)
        {
          for (k = 0; k < (2 * cantidad_actuadores + 3); k++)
          {
            inData1[k] = inData[k];
          }
          evento_rx = 1; //indico nuevo dato  
     
        }
        i = 0; //reinicion enganche de trama
        encabezado = 0;
      }
    }
  }
}//--END serialEvent()-------------- -------------------------

//Referencias
//[1] https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

