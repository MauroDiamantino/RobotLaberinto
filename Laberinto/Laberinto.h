//////////////////////////////////////////LIBRERIAS//////////////////////////////////////////////////////////////////////////////////////////////////
#include <FlexiTimer2.h>

//////////////////////////////////////////CONSTANTES//////////////////////////////////////////////////////////////////////////////////////////////////

//Constantes del algoritmo
//const float PI = 3.14; //Es innecesario definir esta constante porque ya esta predefinida
const float INT_MUESTREO = 1000.0; //Intervalo de muestro en milisegundos
const unsigned long VEL_COM_SERIE = 115200;
const unsigned int NUM_FLANCOS_VUELTA = 16; //Cantidad de flancos efectivos por vuelta, es decir flancos que producen interrupciones.
                                            //Esto depende de la configuracion de las interrupciones del micro.
const float ZONA_MUERTA = 4.0; //Zona muerta del motor tomando el valor absoluto de la tension de alimentacion.
                                //En realidad la zona muerta es ±4v.
const float TOLERANCIA = 0.1; //Tolerancia para el control de la velocidad de las rueda, 0.1m/s. Este valor debería ser definido
                              //considerando que el hecho de que exista una diferencia en las velocidades de las ruedas producira
                              //una desviacion lateral (giro) del robot mientras este intenta moverse en linea recta, lo cual
                              //podria producir el impacto del mismo contra las paredes del laberinto si el valor no es elegido
                              //correctamente.
const float TENSION_MAX = 11.1; //Nuestro motor es de 18V, 11 Ohm y 8800RPM
const float VEL_LIN_MAX_LEFT = 1.0; //velocidad lineal maxima (m/s) permitida para el motor izquierdo
const float VEL_LIN_MAX_RIGHT = 1.0; //velocidad lineal maxima (m/s) permitida para el motor derecho

//Pines
const unsigned int PIN_ENCODER_LEFT = 2;
const unsigned int PIN_ENCODER_RIGHT = 3;
const unsigned int PIN1_MOTOR_LEFT = 5;
const unsigned int PIN2_MOTOR_LEFT = 6;
const unsigned int PIN1_MOTOR_RIGHT = 9;
const unsigned int PIN2_MOTOR_RIGHT = 10;
