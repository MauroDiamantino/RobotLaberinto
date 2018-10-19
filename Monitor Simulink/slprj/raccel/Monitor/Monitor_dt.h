#include "__cf_Monitor.h"
#include "ext_types.h"
static uint_T rtDataTypeSizes [ ] = { sizeof ( real_T ) , sizeof ( real32_T )
, sizeof ( int8_T ) , sizeof ( uint8_T ) , sizeof ( int16_T ) , sizeof (
uint16_T ) , sizeof ( int32_T ) , sizeof ( uint32_T ) , sizeof ( boolean_T )
, sizeof ( fcn_call_T ) , sizeof ( int_T ) , sizeof ( pointer_T ) , sizeof (
action_T ) , 2 * sizeof ( uint32_T ) } ; static const char_T *
rtDataTypeNames [ ] = { "real_T" , "real32_T" , "int8_T" , "uint8_T" ,
"int16_T" , "uint16_T" , "int32_T" , "uint32_T" , "boolean_T" , "fcn_call_T"
, "int_T" , "pointer_T" , "action_T" , "timer_uint32_pair_T" } ; static
DataTypeTransition rtBTransitions [ ] = { { ( char_T * ) ( & rtB . e51hxnpgyt
[ 0 ] ) , 0 , 0 , 5 } , { ( char_T * ) ( & rtB . e4tzxunoxe [ 0 ] ) , 1 , 0 ,
5 } , { ( char_T * ) ( & rtDW . pth3iimf1a ) , 11 , 0 , 7 } } ; static
DataTypeTransitionTable rtBTransTable = { 3U , rtBTransitions } ; static
DataTypeTransition rtPTransitions [ ] = { { ( char_T * ) ( & rtP .
QueryInstrument_P1_Size [ 0 ] ) , 0 , 0 , 178 } , { ( char_T * ) ( & rtP .
Gain_Gain ) , 1 , 0 , 1 } } ; static DataTypeTransitionTable rtPTransTable =
{ 2U , rtPTransitions } ;
