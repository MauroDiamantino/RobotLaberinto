#include "__cf_Monitor.h"
#ifndef RTW_HEADER_Monitor_private_h_
#define RTW_HEADER_Monitor_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)   if(!(ptr)) {\
  ssSetErrorStatus(rtS, RT_MEMORY_ALLOCATION_ERROR);\
  }
#endif
#if !defined(rt_FREE)
#if !defined(_WIN32)
#define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((ptr));\
  (ptr) = (NULL);\
  }
#else
#define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((void *)(ptr));\
  (ptr) = (NULL);\
  }
#endif
#endif
extern void QueryInstrument ( SimStruct * rts ) ;
#if !defined(MULTITASKING) && !defined(NRT)
#error Model (Monitor) was built \in MultiTasking solver mode, however the MULTITASKING define \is not present. Please verify that your template makefile is \configured correctly.
#endif
#endif
