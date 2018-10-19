#include "__cf_Monitor.h"
#ifndef RTW_HEADER_Monitor_h_
#define RTW_HEADER_Monitor_h_
#include <stddef.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef Monitor_COMMON_INCLUDES_
#define Monitor_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "sigstream_rtw.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging.h"
#include "dt_info.h"
#include "ext_work.h"
#endif
#include "Monitor_types.h"
#include "multiword_types.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME Monitor
#define NSAMPLE_TIMES (2) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (2) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (0)   
#elif NCSTATES != 0
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T e51hxnpgyt [ 5 ] ; real32_T e4tzxunoxe [ 5 ] ; } B ;
typedef struct { void * pth3iimf1a ; struct { void * LoggedData [ 2 ] ; }
pb5lgtcc5y ; struct { void * LoggedData [ 4 ] ; } cmfu1hogpu ; } DW ; typedef
struct { rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T
QueryInstrument_P1_Size [ 2 ] ; real_T QueryInstrument_P1 ; real_T
QueryInstrument_P2_Size [ 2 ] ; real_T QueryInstrument_P2 ; real_T
QueryInstrument_P3_Size [ 2 ] ; real_T QueryInstrument_P3 ; real_T
QueryInstrument_P4_Size [ 2 ] ; real_T QueryInstrument_P4 ; real_T
QueryInstrument_P5_Size [ 2 ] ; real_T QueryInstrument_P5 ; real_T
QueryInstrument_P6_Size [ 2 ] ; real_T QueryInstrument_P6 ; real_T
QueryInstrument_P7_Size [ 2 ] ; real_T QueryInstrument_P7 ; real_T
QueryInstrument_P8_Size [ 2 ] ; real_T QueryInstrument_P8 ; real_T
QueryInstrument_P9_Size [ 2 ] ; real_T QueryInstrument_P9 ; real_T
QueryInstrument_P10_Size [ 2 ] ; real_T QueryInstrument_P10 ; real_T
QueryInstrument_P11_Size [ 2 ] ; real_T QueryInstrument_P11 ; real_T
QueryInstrument_P12_Size [ 2 ] ; real_T QueryInstrument_P12 ; real_T
QueryInstrument_P13_Size [ 2 ] ; real_T QueryInstrument_P13 ; real_T
QueryInstrument_P14_Size [ 2 ] ; real_T QueryInstrument_P14 ; real_T
QueryInstrument_P15_Size [ 2 ] ; real_T QueryInstrument_P15 ; real_T
QueryInstrument_P16_Size [ 2 ] ; real_T QueryInstrument_P16 [ 9 ] ; real_T
QueryInstrument_P17_Size [ 2 ] ; real_T QueryInstrument_P17 ; real_T
QueryInstrument_P18_Size [ 2 ] ; real_T QueryInstrument_P18 [ 9 ] ; real_T
QueryInstrument_P19_Size [ 2 ] ; real_T QueryInstrument_P19 ; real_T
QueryInstrument_P20_Size [ 2 ] ; real_T QueryInstrument_P20 [ 10 ] ; real_T
QueryInstrument_P21_Size [ 2 ] ; real_T QueryInstrument_P21 ; real_T
QueryInstrument_P22_Size [ 2 ] ; real_T QueryInstrument_P22 [ 15 ] ; real_T
QueryInstrument_P23_Size [ 2 ] ; real_T QueryInstrument_P23 [ 3 ] ; real_T
QueryInstrument_P24_Size [ 2 ] ; real_T QueryInstrument_P25_Size [ 2 ] ;
real_T QueryInstrument_P25 ; real_T QueryInstrument_P26_Size [ 2 ] ; real_T
QueryInstrument_P26 [ 10 ] ; real_T QueryInstrument_P27_Size [ 2 ] ; real_T
QueryInstrument_P27 ; real_T QueryInstrument_P28_Size [ 2 ] ; real_T
QueryInstrument_P28 ; real_T QueryInstrument_P29_Size [ 2 ] ; real_T
QueryInstrument_P29 ; real_T QueryInstrument_P30_Size [ 2 ] ; real_T
QueryInstrument_P30 ; real_T QueryInstrument_P31_Size [ 2 ] ; real_T
QueryInstrument_P31 ; real_T QueryInstrument_P32_Size [ 2 ] ; real_T
QueryInstrument_P32 ; real_T QueryInstrument_P33_Size [ 2 ] ; real_T
QueryInstrument_P33 ; real_T QueryInstrument_P34_Size [ 2 ] ; real_T
QueryInstrument_P34 ; real_T QueryInstrument_P35_Size [ 2 ] ; real_T
QueryInstrument_P35 [ 9 ] ; real_T QueryInstrument_P36_Size [ 2 ] ; real_T
QueryInstrument_P36 ; real_T QueryInstrument_P37_Size [ 2 ] ; real_T
QueryInstrument_P37 ; real_T QueryInstrument_P38_Size [ 2 ] ; real_T
QueryInstrument_P38 ; real_T QueryInstrument_P39_Size [ 2 ] ; real_T
QueryInstrument_P39 ; real_T SimulationPace_P1 ; real_T SimulationPace_P2 ;
real_T SimulationPace_P3 ; real_T SimulationPace_P4 ; real32_T Gain_Gain ; }
; extern const char * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern DW
rtDW ; extern P rtP ; extern const rtwCAPI_ModelMappingStaticInfo *
Monitor_GetCAPIStaticMap ( void ) ; extern SimStruct * const rtS ; extern
const int_T gblNumToFiles ; extern const int_T gblNumFrFiles ; extern const
int_T gblNumFrWksBlocks ; extern rtInportTUtable * gblInportTUtables ; extern
const char * gblInportFileName ; extern const int_T gblNumRootInportBlks ;
extern const int_T gblNumModelInputs ; extern const int_T
gblInportDataTypeIdx [ ] ; extern const int_T gblInportDims [ ] ; extern
const int_T gblInportComplex [ ] ; extern const int_T gblInportInterpoFlag [
] ; extern const int_T gblInportContinuous [ ] ; extern const int_T
gblParameterTuningTid ; extern size_t gblCurrentSFcnIdx ; extern DataMapInfo
* rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ;
void MdlOutputs ( int_T tid ) ; void MdlOutputsParameterSampleTime ( int_T
tid ) ; void MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void
MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ;
SimStruct * raccel_register_model ( void ) ;
#endif
