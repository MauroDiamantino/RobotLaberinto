#include "__cf_Monitor.h"
#include "rt_logging_mmi.h"
#include "Monitor_capi.h"
#include <math.h>
#include "Monitor.h"
#include "Monitor_private.h"
#include "Monitor_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; RTWExtModeInfo * gblRTWExtModeInfo = NULL ; extern boolean_T
gblExtModeStartPktReceived ; void raccelForceExtModeShutdown ( ) { if ( !
gblExtModeStartPktReceived ) { boolean_T stopRequested = false ;
rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 2 , & stopRequested ) ; }
rtExtModeShutdown ( 2 ) ; }
#include "slsv_diagnostic_codegen_c_api.h"
const int_T gblNumToFiles = 0 ; const int_T gblNumFrFiles = 0 ; const int_T
gblNumFrWksBlocks = 0 ;
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 0 ; int_T gbl_raccel_NumST = 2 ; const char_T
* gbl_raccel_Version = "8.13 (R2017b) 24-Jul-2017" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#else
UNUSED_PARAMETER ( S ) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; const char * gblSlvrJacPatternFileName =
"slprj\\raccel\\Monitor\\Monitor_Jpattern.mat" ; const int_T
gblNumRootInportBlks = 0 ; const int_T gblNumModelInputs = 0 ; extern
rtInportTUtable * gblInportTUtables ; extern const char * gblInportFileName ;
const int_T gblInportDataTypeIdx [ ] = { - 1 } ; const int_T gblInportDims [
] = { - 1 } ; const int_T gblInportComplex [ ] = { - 1 } ; const int_T
gblInportInterpoFlag [ ] = { - 1 } ; const int_T gblInportContinuous [ ] = {
- 1 } ;
#include "simstruc.h"
#include "fixedpoint.h"
B rtB ; DW rtDW ; static SimStruct model_S ; SimStruct * const rtS = &
model_S ; void MdlStart ( void ) { { void * * slioCatalogueAddr =
rt_slioCatalogueAddr ( ) ; void * r2 = ( NULL ) ; void * *
pOSigstreamManagerAddr = ( NULL ) ; const char *
errorCreatingOSigstreamManager = ( NULL ) ; const char *
errorAddingR2SharedResource = ( NULL ) ; * slioCatalogueAddr =
rtwGetNewSlioCatalogue ( rt_GetMatSigLogSelectorFileName ( ) ) ;
errorAddingR2SharedResource = rtwAddR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) , 1 ) ; if (
errorAddingR2SharedResource != ( NULL ) ) { rtwTerminateSlioCatalogue (
slioCatalogueAddr ) ; * slioCatalogueAddr = ( NULL ) ; ssSetErrorStatus ( rtS
, errorAddingR2SharedResource ) ; return ; } r2 = rtwGetR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ) ;
pOSigstreamManagerAddr = rt_GetOSigstreamManagerAddr ( ) ;
errorCreatingOSigstreamManager = rtwOSigstreamManagerCreateInstance (
rt_GetMatSigLogSelectorFileName ( ) , r2 , pOSigstreamManagerAddr ) ; if (
errorCreatingOSigstreamManager != ( NULL ) ) { * pOSigstreamManagerAddr = (
NULL ) ; ssSetErrorStatus ( rtS , errorCreatingOSigstreamManager ) ; return ;
} } { SimStruct * rts = ssGetSFunction ( rtS , 0 ) ; { gblCurrentSFcnIdx = 0
; sfcnStart ( rts ) ; } if ( ssGetErrorStatus ( rts ) != ( NULL ) ) return ;
} { bool externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( ) ; rtwISigstreamManagerGetInputIsInDatasetFormat (
pISigstreamManager , & externalInputIsInDatasetFormat ) ; if (
externalInputIsInDatasetFormat ) { } } } void MdlOutputs ( int_T tid ) {
int32_T i ; if ( ssIsSampleHit ( rtS , 0 , tid ) ) { { SimStruct * rts =
ssGetSFunction ( rtS , 0 ) ; { gblCurrentSFcnIdx = 0 ; { int_T origSampleHit
= ssGetSampleHitPtr ( rts ) [ 0 ] ; _ssSetSampleHit ( rts , 0 , 1 ) ;
sfcnOutputs ( rts , ( tid <= 1 ) && gbl_raccel_tid01eq ? 0 : tid ) ;
_ssSetSampleHit ( rts , 0 , origSampleHit ) ; } } } for ( i = 0 ; i < 5 ; i
++ ) { rtB . e4tzxunoxe [ i ] = rtP . Gain_Gain * ( real32_T ) rtB .
e51hxnpgyt [ i ] ; } } if ( ssIsSampleHit ( rtS , 1 , tid ) ) { }
UNUSED_PARAMETER ( tid ) ; } void MdlUpdate ( int_T tid ) { UNUSED_PARAMETER
( tid ) ; } void MdlTerminate ( void ) { { SimStruct * rts = ssGetSFunction (
rtS , 0 ) ; { gblCurrentSFcnIdx = 0 ; sfcnTerminate ( rts ) ; } } if (
rt_slioCatalogue ( ) != ( NULL ) ) { void * * slioCatalogueAddr =
rt_slioCatalogueAddr ( ) ; rtwSaveDatasetsToMatFile (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ,
rt_GetMatSigstreamLoggingFileName ( ) ) ; rtwTerminateSlioCatalogue (
slioCatalogueAddr ) ; * slioCatalogueAddr = NULL ; } } void
MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS , 0 ) ; ssSetNumY (
rtS , 0 ) ; ssSetNumU ( rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ;
ssSetNumSampleTimes ( rtS , 2 ) ; ssSetNumBlocks ( rtS , 6 ) ;
ssSetNumBlockIO ( rtS , 2 ) ; ssSetNumBlockParams ( rtS , 179 ) ; } void
MdlInitializeSampleTimes ( void ) { ssSetSampleTime ( rtS , 0 , 0.01 ) ;
ssSetSampleTime ( rtS , 1 , 0.1 ) ; ssSetOffsetTime ( rtS , 0 , 0.0 ) ;
ssSetOffsetTime ( rtS , 1 , 0.0 ) ; } void raccel_set_checksum ( ) {
ssSetChecksumVal ( rtS , 0 , 1117595932U ) ; ssSetChecksumVal ( rtS , 1 ,
2452019217U ) ; ssSetChecksumVal ( rtS , 2 , 2211008495U ) ; ssSetChecksumVal
( rtS , 3 , 3397671295U ) ; }
#if defined(_MSC_VER)
#pragma optimize( "", off )
#endif
SimStruct * raccel_register_model ( void ) { static struct _ssMdlInfo mdlInfo
; ( void ) memset ( ( char * ) rtS , 0 , sizeof ( SimStruct ) ) ; ( void )
memset ( ( char * ) & mdlInfo , 0 , sizeof ( struct _ssMdlInfo ) ) ;
ssSetMdlInfoPtr ( rtS , & mdlInfo ) ; { static time_T mdlPeriod [
NSAMPLE_TIMES ] ; static time_T mdlOffset [ NSAMPLE_TIMES ] ; static time_T
mdlTaskTimes [ NSAMPLE_TIMES ] ; static int_T mdlTsMap [ NSAMPLE_TIMES ] ;
static int_T mdlSampleHits [ NSAMPLE_TIMES ] ; static boolean_T
mdlTNextWasAdjustedPtr [ NSAMPLE_TIMES ] ; static int_T mdlPerTaskSampleHits
[ NSAMPLE_TIMES * NSAMPLE_TIMES ] ; static time_T mdlTimeOfNextSampleHit [
NSAMPLE_TIMES ] ; { int_T i ; for ( i = 0 ; i < NSAMPLE_TIMES ; i ++ ) {
mdlPeriod [ i ] = 0.0 ; mdlOffset [ i ] = 0.0 ; mdlTaskTimes [ i ] = 0.0 ;
mdlTsMap [ i ] = i ; } } mdlSampleHits [ 0 ] = 1 ; ssSetSampleTimePtr ( rtS ,
& mdlPeriod [ 0 ] ) ; ssSetOffsetTimePtr ( rtS , & mdlOffset [ 0 ] ) ;
ssSetSampleTimeTaskIDPtr ( rtS , & mdlTsMap [ 0 ] ) ; ssSetTPtr ( rtS , &
mdlTaskTimes [ 0 ] ) ; ssSetSampleHitPtr ( rtS , & mdlSampleHits [ 0 ] ) ;
ssSetTNextWasAdjustedPtr ( rtS , & mdlTNextWasAdjustedPtr [ 0 ] ) ;
ssSetPerTaskSampleHitsPtr ( rtS , & mdlPerTaskSampleHits [ 0 ] ) ;
ssSetTimeOfNextSampleHitPtr ( rtS , & mdlTimeOfNextSampleHit [ 0 ] ) ; } {
static int_T mdlPerTaskSampleHits [ NSAMPLE_TIMES * NSAMPLE_TIMES ] ; ( void
) memset ( ( void * ) & mdlPerTaskSampleHits [ 0 ] , 0 , 2 * 2 * sizeof (
int_T ) ) ; ssSetPerTaskSampleHitsPtr ( rtS , & mdlPerTaskSampleHits [ 0 ] )
; } ssSetSolverMode ( rtS , SOLVER_MODE_MULTITASKING ) ; { ssSetBlockIO ( rtS
, ( ( void * ) & rtB ) ) ; ( void ) memset ( ( ( void * ) & rtB ) , 0 ,
sizeof ( B ) ) ; } ssSetDefaultParam ( rtS , ( real_T * ) & rtP ) ; { void *
dwork = ( void * ) & rtDW ; ssSetRootDWork ( rtS , dwork ) ; ( void ) memset
( dwork , 0 , sizeof ( DW ) ) ; } { static DataTypeTransInfo dtInfo ; ( void
) memset ( ( char_T * ) & dtInfo , 0 , sizeof ( dtInfo ) ) ;
ssSetModelMappingInfo ( rtS , & dtInfo ) ; dtInfo . numDataTypes = 14 ;
dtInfo . dataTypeSizes = & rtDataTypeSizes [ 0 ] ; dtInfo . dataTypeNames = &
rtDataTypeNames [ 0 ] ; dtInfo . BTransTable = & rtBTransTable ; dtInfo .
PTransTable = & rtPTransTable ; } Monitor_InitializeDataMapInfo ( ) ;
ssSetIsRapidAcceleratorActive ( rtS , true ) ; ssSetRootSS ( rtS , rtS ) ;
ssSetVersion ( rtS , SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS ,
"Monitor" ) ; ssSetPath ( rtS , "Monitor" ) ; ssSetTStart ( rtS , 0.0 ) ;
ssSetTFinal ( rtS , rtInf ) ; ssSetStepSize ( rtS , 0.01 ) ;
ssSetFixedStepSize ( rtS , 0.01 ) ; { static RTWLogInfo rt_DataLoggingInfo ;
rt_DataLoggingInfo . loggingInterval = NULL ; ssSetRTWLogInfo ( rtS , &
rt_DataLoggingInfo ) ; } { rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) ,
( NULL ) ) ; rtliSetLogXSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ;
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "" ) ; rtliSetLogXFinal ( ssGetRTWLogInfo ( rtS ) ,
"" ) ; rtliSetLogVarNameModifier ( ssGetRTWLogInfo ( rtS ) , "none" ) ;
rtliSetLogFormat ( ssGetRTWLogInfo ( rtS ) , 2 ) ; rtliSetLogMaxRows (
ssGetRTWLogInfo ( rtS ) , 1000 ) ; rtliSetLogDecimation ( ssGetRTWLogInfo (
rtS ) , 1 ) ; rtliSetLogY ( ssGetRTWLogInfo ( rtS ) , "" ) ;
rtliSetLogYSignalInfo ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ;
rtliSetLogYSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ; } { static
ssSolverInfo slvrInfo ; ssSetSolverInfo ( rtS , & slvrInfo ) ;
ssSetSolverName ( rtS , "FixedStepDiscrete" ) ; ssSetVariableStepSolver ( rtS
, 0 ) ; ssSetSolverConsistencyChecking ( rtS , 0 ) ;
ssSetSolverAdaptiveZcDetection ( rtS , 0 ) ; ssSetSolverRobustResetMethod (
rtS , 0 ) ; ssSetSolverStateProjection ( rtS , 0 ) ;
ssSetSolverMassMatrixType ( rtS , ( ssMatrixType ) 0 ) ;
ssSetSolverMassMatrixNzMax ( rtS , 0 ) ; ssSetModelOutputs ( rtS , MdlOutputs
) ; ssSetModelLogData ( rtS , rt_UpdateTXYLogVars ) ;
ssSetModelLogDataIfInInterval ( rtS , rt_UpdateTXXFYLogVars ) ;
ssSetModelUpdate ( rtS , MdlUpdate ) ; ssSetTNextTid ( rtS , INT_MIN ) ;
ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ;
ssSetNumNonsampledZCs ( rtS , 0 ) ; } ssSetChecksumVal ( rtS , 0 ,
1117595932U ) ; ssSetChecksumVal ( rtS , 1 , 2452019217U ) ; ssSetChecksumVal
( rtS , 2 , 2211008495U ) ; ssSetChecksumVal ( rtS , 3 , 3397671295U ) ; {
static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE ; static
RTWExtModeInfo rt_ExtModeInfo ; static const sysRanDType * systemRan [ 1 ] ;
gblRTWExtModeInfo = & rt_ExtModeInfo ; ssSetRTWExtModeInfo ( rtS , &
rt_ExtModeInfo ) ; rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo ,
systemRan ) ; systemRan [ 0 ] = & rtAlwaysEnabled ;
rteiSetModelMappingInfoPtr ( ssGetRTWExtModeInfo ( rtS ) , &
ssGetModelMappingInfo ( rtS ) ) ; rteiSetChecksumsPtr ( ssGetRTWExtModeInfo (
rtS ) , ssGetChecksums ( rtS ) ) ; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS )
, ssGetTPtr ( rtS ) ) ; } ssSetNumSFunctions ( rtS , 1 ) ; { static SimStruct
childSFunctions [ 1 ] ; static SimStruct * childSFunctionPtrs [ 1 ] ; ( void
) memset ( ( void * ) & childSFunctions [ 0 ] , 0 , sizeof ( childSFunctions
) ) ; ssSetSFunctions ( rtS , & childSFunctionPtrs [ 0 ] ) ; ssSetSFunction (
rtS , 0 , & childSFunctions [ 0 ] ) ; { SimStruct * rts = ssGetSFunction (
rtS , 0 ) ; static time_T sfcnPeriod [ 1 ] ; static time_T sfcnOffset [ 1 ] ;
static int_T sfcnTsMap [ 1 ] ; ( void ) memset ( ( void * ) sfcnPeriod , 0 ,
sizeof ( time_T ) * 1 ) ; ( void ) memset ( ( void * ) sfcnOffset , 0 ,
sizeof ( time_T ) * 1 ) ; ssSetSampleTimePtr ( rts , & sfcnPeriod [ 0 ] ) ;
ssSetOffsetTimePtr ( rts , & sfcnOffset [ 0 ] ) ; ssSetSampleTimeTaskIDPtr (
rts , sfcnTsMap ) ; { static struct _ssBlkInfo2 _blkInfo2 ; struct
_ssBlkInfo2 * blkInfo2 = & _blkInfo2 ; ssSetBlkInfo2Ptr ( rts , blkInfo2 ) ;
} { static struct _ssPortInfo2 _portInfo2 ; struct _ssPortInfo2 * portInfo2 =
& _portInfo2 ; _ssSetBlkInfo2PortInfo2Ptr ( rts , portInfo2 ) ; }
ssSetMdlInfoPtr ( rts , ssGetMdlInfoPtr ( rtS ) ) ; { static struct
_ssSFcnModelMethods2 methods2 ; ssSetModelMethods2 ( rts , & methods2 ) ; } {
static struct _ssSFcnModelMethods3 methods3 ; ssSetModelMethods3 ( rts , &
methods3 ) ; } { static struct _ssSFcnModelMethods4 methods4 ;
ssSetModelMethods4 ( rts , & methods4 ) ; } { static struct _ssStatesInfo2
statesInfo2 ; static ssPeriodicStatesInfo periodicStatesInfo ;
ssSetStatesInfo2 ( rts , & statesInfo2 ) ; ssSetPeriodicStatesInfo ( rts , &
periodicStatesInfo ) ; } { static struct _ssPortOutputs outputPortInfo [ 1 ]
; ssSetPortInfoForOutputs ( rts , & outputPortInfo [ 0 ] ) ;
_ssSetNumOutputPorts ( rts , 1 ) ; { static struct _ssOutPortUnit
outputPortUnits [ 1 ] ; _ssSetPortInfo2ForOutputUnits ( rts , &
outputPortUnits [ 0 ] ) ; } ssSetOutputPortUnit ( rts , 0 , 0 ) ; { static
int_T dimensions [ 2 ] ; dimensions [ 0 ] = 5 ; dimensions [ 1 ] = 1 ;
_ssSetOutputPortDimensionsPtr ( rts , 0 , dimensions ) ;
_ssSetOutputPortNumDimensions ( rts , 0 , 2 ) ; ssSetOutputPortWidth ( rts ,
0 , 5 ) ; ssSetOutputPortSignal ( rts , 0 , ( ( real_T * ) rtB . e51hxnpgyt )
) ; } } ssSetModelName ( rts , "QueryInstrument" ) ; ssSetPath ( rts ,
"Monitor/Query Instrument" ) ; if ( ssGetRTModel ( rtS ) == ( NULL ) ) {
ssSetParentSS ( rts , rtS ) ; ssSetRootSS ( rts , ssGetRootSS ( rtS ) ) ; }
else { ssSetRTModel ( rts , ssGetRTModel ( rtS ) ) ; ssSetParentSS ( rts , (
NULL ) ) ; ssSetRootSS ( rts , rts ) ; } ssSetVersion ( rts ,
SIMSTRUCT_VERSION_LEVEL2 ) ; { static mxArray * sfcnParams [ 39 ] ;
ssSetSFcnParamsCount ( rts , 39 ) ; ssSetSFcnParamsPtr ( rts , & sfcnParams [
0 ] ) ; ssSetSFcnParam ( rts , 0 , ( mxArray * ) rtP .
QueryInstrument_P1_Size ) ; ssSetSFcnParam ( rts , 1 , ( mxArray * ) rtP .
QueryInstrument_P2_Size ) ; ssSetSFcnParam ( rts , 2 , ( mxArray * ) rtP .
QueryInstrument_P3_Size ) ; ssSetSFcnParam ( rts , 3 , ( mxArray * ) rtP .
QueryInstrument_P4_Size ) ; ssSetSFcnParam ( rts , 4 , ( mxArray * ) rtP .
QueryInstrument_P5_Size ) ; ssSetSFcnParam ( rts , 5 , ( mxArray * ) rtP .
QueryInstrument_P6_Size ) ; ssSetSFcnParam ( rts , 6 , ( mxArray * ) rtP .
QueryInstrument_P7_Size ) ; ssSetSFcnParam ( rts , 7 , ( mxArray * ) rtP .
QueryInstrument_P8_Size ) ; ssSetSFcnParam ( rts , 8 , ( mxArray * ) rtP .
QueryInstrument_P9_Size ) ; ssSetSFcnParam ( rts , 9 , ( mxArray * ) rtP .
QueryInstrument_P10_Size ) ; ssSetSFcnParam ( rts , 10 , ( mxArray * ) rtP .
QueryInstrument_P11_Size ) ; ssSetSFcnParam ( rts , 11 , ( mxArray * ) rtP .
QueryInstrument_P12_Size ) ; ssSetSFcnParam ( rts , 12 , ( mxArray * ) rtP .
QueryInstrument_P13_Size ) ; ssSetSFcnParam ( rts , 13 , ( mxArray * ) rtP .
QueryInstrument_P14_Size ) ; ssSetSFcnParam ( rts , 14 , ( mxArray * ) rtP .
QueryInstrument_P15_Size ) ; ssSetSFcnParam ( rts , 15 , ( mxArray * ) rtP .
QueryInstrument_P16_Size ) ; ssSetSFcnParam ( rts , 16 , ( mxArray * ) rtP .
QueryInstrument_P17_Size ) ; ssSetSFcnParam ( rts , 17 , ( mxArray * ) rtP .
QueryInstrument_P18_Size ) ; ssSetSFcnParam ( rts , 18 , ( mxArray * ) rtP .
QueryInstrument_P19_Size ) ; ssSetSFcnParam ( rts , 19 , ( mxArray * ) rtP .
QueryInstrument_P20_Size ) ; ssSetSFcnParam ( rts , 20 , ( mxArray * ) rtP .
QueryInstrument_P21_Size ) ; ssSetSFcnParam ( rts , 21 , ( mxArray * ) rtP .
QueryInstrument_P22_Size ) ; ssSetSFcnParam ( rts , 22 , ( mxArray * ) rtP .
QueryInstrument_P23_Size ) ; ssSetSFcnParam ( rts , 23 , ( mxArray * ) rtP .
QueryInstrument_P24_Size ) ; ssSetSFcnParam ( rts , 24 , ( mxArray * ) rtP .
QueryInstrument_P25_Size ) ; ssSetSFcnParam ( rts , 25 , ( mxArray * ) rtP .
QueryInstrument_P26_Size ) ; ssSetSFcnParam ( rts , 26 , ( mxArray * ) rtP .
QueryInstrument_P27_Size ) ; ssSetSFcnParam ( rts , 27 , ( mxArray * ) rtP .
QueryInstrument_P28_Size ) ; ssSetSFcnParam ( rts , 28 , ( mxArray * ) rtP .
QueryInstrument_P29_Size ) ; ssSetSFcnParam ( rts , 29 , ( mxArray * ) rtP .
QueryInstrument_P30_Size ) ; ssSetSFcnParam ( rts , 30 , ( mxArray * ) rtP .
QueryInstrument_P31_Size ) ; ssSetSFcnParam ( rts , 31 , ( mxArray * ) rtP .
QueryInstrument_P32_Size ) ; ssSetSFcnParam ( rts , 32 , ( mxArray * ) rtP .
QueryInstrument_P33_Size ) ; ssSetSFcnParam ( rts , 33 , ( mxArray * ) rtP .
QueryInstrument_P34_Size ) ; ssSetSFcnParam ( rts , 34 , ( mxArray * ) rtP .
QueryInstrument_P35_Size ) ; ssSetSFcnParam ( rts , 35 , ( mxArray * ) rtP .
QueryInstrument_P36_Size ) ; ssSetSFcnParam ( rts , 36 , ( mxArray * ) rtP .
QueryInstrument_P37_Size ) ; ssSetSFcnParam ( rts , 37 , ( mxArray * ) rtP .
QueryInstrument_P38_Size ) ; ssSetSFcnParam ( rts , 38 , ( mxArray * ) rtP .
QueryInstrument_P39_Size ) ; } ssSetPWork ( rts , ( void * * ) & rtDW .
pth3iimf1a ) ; { static struct _ssDWorkRecord dWorkRecord [ 1 ] ; static
struct _ssDWorkAuxRecord dWorkAuxRecord [ 1 ] ; ssSetSFcnDWork ( rts ,
dWorkRecord ) ; ssSetSFcnDWorkAux ( rts , dWorkAuxRecord ) ; _ssSetNumDWork (
rts , 1 ) ; ssSetDWorkWidth ( rts , 0 , 1 ) ; ssSetDWorkDataType ( rts , 0 ,
SS_POINTER ) ; ssSetDWorkComplexSignal ( rts , 0 , 0 ) ; ssSetDWork ( rts , 0
, & rtDW . pth3iimf1a ) ; } { raccelLoadSFcnMexFile ( "QueryInstrument" ,
"Monitor:8" , rts , 0 ) ; if ( ssGetErrorStatus ( rtS ) ) { return rtS ; } }
sfcnInitializeSampleTimes ( rts ) ; ssSetSampleTime ( rts , 0 , 0.01 ) ;
ssSetOffsetTime ( rts , 0 , 0.0 ) ; sfcnTsMap [ 0 ] = 0 ;
ssSetNumNonsampledZCs ( rts , 0 ) ; _ssSetOutputPortConnected ( rts , 0 , 1 )
; _ssSetOutputPortBeingMerged ( rts , 0 , 0 ) ; } } return rtS ; }
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif
const int_T gblParameterTuningTid = - 1 ; void MdlOutputsParameterSampleTime
( int_T tid ) { UNUSED_PARAMETER ( tid ) ; }
