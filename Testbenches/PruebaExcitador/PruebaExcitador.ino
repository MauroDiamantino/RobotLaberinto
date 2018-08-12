#include <FlexiTimer2.h>

//Constantes
const byte INT_MUESTREO = 1; //Intervalo de muestro en milisegundos, 1mS.
const unsigned int SEMI_PER_EXCITACION = 1000; //Semiperiodo (en mS) de la señal de excitacion que permite calibrar los PIDs. Se trata de una señal
                                     //cuadrada, generada localmente, que se aplica en las entradas de referencia (r).
const unsigned long VEL_COM_SERIE = 115200; //Velocidad de la comunicacion serie
const float VEL_LIN_MAX = 1.0; //Velocidad lineal maxima (m/s) permitida para ambos motores.

//Pines
const byte PIN_EXCITADOR_ONOFF = 13;

//VARIABLES GLOBALES
//Contadores de los flancos efectivos (aquellos que producen interrupciones) de los encoder.
volatile unsigned int flancos_left = 0; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]
volatile unsigned int flancos_right = 0; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]

//Flancos efectivos contados por los encoder en un determinado lapso de tiempo, el intervalo de muestreo.
volatile unsigned int total_flancos_left = 0; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]
volatile unsigned int total_flancos_right = 0; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]

//Referencias
float r_left = 0.0; //velocidad lineal de referencia (m/s)
float r_right = 0.0; //velocidad lineal de referencia (m/s)

//Flags
volatile boolean flag_control = false; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]
volatile boolean flag_excitador = false; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]
volatile boolean flag_parser = false; //Se declara "volatile" siguiendo las recomendaciones que aparecen en la pagina de la funcion attachInterrupt. [1]

void setup() {
  pinMode(PIN_EXCITADOR_ONOFF, INPUT_PULLUP);

  //Configuracion de la comunicación serie
  Serial.begin(VEL_COM_SERIE);

  //Configuración del Timer 2, que es utilizado para contar el intervalo de muestreo.
  FlexiTimer2::set(INT_MUESTREO, RS_TIMER2); // El tiempo de muestreo se coloca en milisegundos
  FlexiTimer2::start();
}

void loop() {
  if (flag_excitador == true){
    Excitador();
    
    flag_excitador = false;
  }
  Serial.print("r_l: ");
  Serial.println(r_left);
  Serial.print("r_r: ");
  Serial.println(r_right);
  Serial.print("\n");
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

  total_flancos_left = flancos_left;
  total_flancos_right = flancos_right;

  flancos_left = 0;
  flancos_right = 0;

  flag_control = true;

  if ( digitalRead(PIN_EXCITADOR_ONOFF)==LOW ){
    cont_excitador--;
    if (cont_excitador==0){
      cont_excitador = SEMI_PER_EXCITACION;
      flag_excitador=true;
    }
  } else if (r_left==0 || r_right==0){
    r_left = VEL_LIN_MAX;
    r_right = VEL_LIN_MAX;
    flag_excitador = false;
  }
}

////////////////////////////////////////////////////////////////////FUNCIONES//////////////////////////////////////////////////////////////////////////////
//Funcion que se encarga de generar una señal cuadrada para excitar ambos lazo de control (excitacion local) y asi poder calibrar los PIDs de los mismos. 
//La señal cuadrada se conecta a las entradas de referencia de ambos lazos (setpoints). Esta señal tiene un semiperiodo mucho mayor que el intervalo de muestreo,
//de modo que luego de que se produzca un flanco (cambio de setpoint) los lazos tengan suficiente tiempo para reducir los errores y estabilizar las salidas 
//de los mismos (velocidades).
//inline void Excitador(float& r_left, float& r_right){ //r_left y r_right son referencias (punteros constantes que se de-referencian automáticamente, muy usado en C++).
inline void Excitador(){
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
