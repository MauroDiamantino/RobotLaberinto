#include "__cf_Monitor.h"
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "Monitor_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "Monitor.h"
#include "Monitor_capi.h"
#include "Monitor_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING (
"Monitor/Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 , 0 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "" ) , 0 , 1 ,
0 , 0 , 0 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static
const rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 2 , TARGET_STRING (
"Monitor/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 1 , 0 } , { 3 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P1" ) , 1 , 1
, 0 } , { 4 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING (
"P2" ) , 1 , 1 , 0 } , { 5 , TARGET_STRING ( "Monitor/Query Instrument" ) ,
TARGET_STRING ( "P3" ) , 1 , 1 , 0 } , { 6 , TARGET_STRING (
"Monitor/Query Instrument" ) , TARGET_STRING ( "P4" ) , 1 , 1 , 0 } , { 7 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P5" ) , 1 , 1
, 0 } , { 8 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING (
"P6" ) , 1 , 1 , 0 } , { 9 , TARGET_STRING ( "Monitor/Query Instrument" ) ,
TARGET_STRING ( "P7" ) , 1 , 1 , 0 } , { 10 , TARGET_STRING (
"Monitor/Query Instrument" ) , TARGET_STRING ( "P8" ) , 1 , 1 , 0 } , { 11 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P9" ) , 1 , 1
, 0 } , { 12 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING (
"P10" ) , 1 , 1 , 0 } , { 13 , TARGET_STRING ( "Monitor/Query Instrument" ) ,
TARGET_STRING ( "P11" ) , 1 , 1 , 0 } , { 14 , TARGET_STRING (
"Monitor/Query Instrument" ) , TARGET_STRING ( "P12" ) , 1 , 1 , 0 } , { 15 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P13" ) , 1 ,
1 , 0 } , { 16 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING
( "P14" ) , 1 , 1 , 0 } , { 17 , TARGET_STRING ( "Monitor/Query Instrument" )
, TARGET_STRING ( "P15" ) , 1 , 1 , 0 } , { 18 , TARGET_STRING (
"Monitor/Query Instrument" ) , TARGET_STRING ( "P16" ) , 1 , 2 , 0 } , { 19 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P17" ) , 1 ,
1 , 0 } , { 20 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING
( "P18" ) , 1 , 2 , 0 } , { 21 , TARGET_STRING ( "Monitor/Query Instrument" )
, TARGET_STRING ( "P19" ) , 1 , 1 , 0 } , { 22 , TARGET_STRING (
"Monitor/Query Instrument" ) , TARGET_STRING ( "P20" ) , 1 , 3 , 0 } , { 23 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P21" ) , 1 ,
1 , 0 } , { 24 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING
( "P22" ) , 1 , 4 , 0 } , { 25 , TARGET_STRING ( "Monitor/Query Instrument" )
, TARGET_STRING ( "P23" ) , 1 , 5 , 0 } , { 26 , TARGET_STRING (
"Monitor/Query Instrument" ) , TARGET_STRING ( "P25" ) , 1 , 1 , 0 } , { 27 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P26" ) , 1 ,
3 , 0 } , { 28 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING
( "P27" ) , 1 , 1 , 0 } , { 29 , TARGET_STRING ( "Monitor/Query Instrument" )
, TARGET_STRING ( "P28" ) , 1 , 1 , 0 } , { 30 , TARGET_STRING (
"Monitor/Query Instrument" ) , TARGET_STRING ( "P29" ) , 1 , 1 , 0 } , { 31 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P30" ) , 1 ,
1 , 0 } , { 32 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING
( "P31" ) , 1 , 1 , 0 } , { 33 , TARGET_STRING ( "Monitor/Query Instrument" )
, TARGET_STRING ( "P32" ) , 1 , 1 , 0 } , { 34 , TARGET_STRING (
"Monitor/Query Instrument" ) , TARGET_STRING ( "P33" ) , 1 , 1 , 0 } , { 35 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P34" ) , 1 ,
1 , 0 } , { 36 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING
( "P35" ) , 1 , 2 , 0 } , { 37 , TARGET_STRING ( "Monitor/Query Instrument" )
, TARGET_STRING ( "P36" ) , 1 , 1 , 0 } , { 38 , TARGET_STRING (
"Monitor/Query Instrument" ) , TARGET_STRING ( "P37" ) , 1 , 1 , 0 } , { 39 ,
TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING ( "P38" ) , 1 ,
1 , 0 } , { 40 , TARGET_STRING ( "Monitor/Query Instrument" ) , TARGET_STRING
( "P39" ) , 1 , 1 , 0 } , { 41 , TARGET_STRING ( "Monitor/Simulation Pace" )
, TARGET_STRING ( "P1" ) , 1 , 1 , 0 } , { 42 , TARGET_STRING (
"Monitor/Simulation Pace" ) , TARGET_STRING ( "P2" ) , 1 , 1 , 0 } , { 43 ,
TARGET_STRING ( "Monitor/Simulation Pace" ) , TARGET_STRING ( "P3" ) , 1 , 1
, 0 } , { 44 , TARGET_STRING ( "Monitor/Simulation Pace" ) , TARGET_STRING (
"P4" ) , 1 , 1 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 } } ; static
const rtwCAPI_ModelParameters rtModelParameters [ ] = { { 0 , ( NULL ) , 0 ,
0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . e4tzxunoxe [ 0 ] , & rtB .
e51hxnpgyt [ 0 ] , & rtP . Gain_Gain , & rtP . QueryInstrument_P1 , & rtP .
QueryInstrument_P2 , & rtP . QueryInstrument_P3 , & rtP . QueryInstrument_P4
, & rtP . QueryInstrument_P5 , & rtP . QueryInstrument_P6 , & rtP .
QueryInstrument_P7 , & rtP . QueryInstrument_P8 , & rtP . QueryInstrument_P9
, & rtP . QueryInstrument_P10 , & rtP . QueryInstrument_P11 , & rtP .
QueryInstrument_P12 , & rtP . QueryInstrument_P13 , & rtP .
QueryInstrument_P14 , & rtP . QueryInstrument_P15 , & rtP .
QueryInstrument_P16 [ 0 ] , & rtP . QueryInstrument_P17 , & rtP .
QueryInstrument_P18 [ 0 ] , & rtP . QueryInstrument_P19 , & rtP .
QueryInstrument_P20 [ 0 ] , & rtP . QueryInstrument_P21 , & rtP .
QueryInstrument_P22 [ 0 ] , & rtP . QueryInstrument_P23 [ 0 ] , & rtP .
QueryInstrument_P25 , & rtP . QueryInstrument_P26 [ 0 ] , & rtP .
QueryInstrument_P27 , & rtP . QueryInstrument_P28 , & rtP .
QueryInstrument_P29 , & rtP . QueryInstrument_P30 , & rtP .
QueryInstrument_P31 , & rtP . QueryInstrument_P32 , & rtP .
QueryInstrument_P33 , & rtP . QueryInstrument_P34 , & rtP .
QueryInstrument_P35 [ 0 ] , & rtP . QueryInstrument_P36 , & rtP .
QueryInstrument_P37 , & rtP . QueryInstrument_P38 , & rtP .
QueryInstrument_P39 , & rtP . SimulationPace_P1 , & rtP . SimulationPace_P2 ,
& rtP . SimulationPace_P3 , & rtP . SimulationPace_P4 , } ; static int32_T *
rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "float" ,
"real32_T" , 0 , 0 , sizeof ( real32_T ) , SS_SINGLE , 0 , 0 } , { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , SS_DOUBLE , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_MATRIX_COL_MAJOR , 0 , 2 , 0 } , { rtwCAPI_SCALAR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 6 , 2 , 0 } , {
rtwCAPI_VECTOR , 8 , 2 , 0 } , { rtwCAPI_VECTOR , 10 , 2 , 0 } } ; static
const uint_T rtDimensionArray [ ] = { 5 , 1 , 1 , 1 , 1 , 9 , 1 , 10 , 1 , 15
, 1 , 3 } ; static const real_T rtcapiStoredFloats [ ] = { 0.01 , 0.0 } ;
static const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , 0 } , } ; static const rtwCAPI_SampleTimeMap
rtSampleTimeMap [ ] = { { ( const void * ) & rtcapiStoredFloats [ 0 ] , (
const void * ) & rtcapiStoredFloats [ 1 ] , 0 , 1 } } ; static
rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 2 , ( NULL )
, 0 , ( NULL ) , 0 } , { rtBlockParameters , 43 , rtModelParameters , 0 } , {
( NULL ) , 0 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap
, rtSampleTimeMap , rtDimensionArray } , "float" , { 1117595932U ,
2452019217U , 2211008495U , 3397671295U } , ( NULL ) , 0 , 0 } ; const
rtwCAPI_ModelMappingStaticInfo * Monitor_GetCAPIStaticMap ( void ) { return &
mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void Monitor_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( ( *
rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void Monitor_host_InitializeDataMapInfo ( Monitor_host_DataMapInfo_T *
dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , NULL ) ; rtwCAPI_SetPath (
dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
