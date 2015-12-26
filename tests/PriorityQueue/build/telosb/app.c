#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 150 "/usr/bin/../lib/gcc/msp430/4.6.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 212
typedef unsigned int size_t;
#line 324
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdint.h" 3
typedef signed char int8_t;
typedef int int16_t;
typedef long int int32_t;
__extension__ 
#line 41
typedef long long int int64_t;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
__extension__ 
#line 46
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 55
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 61
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 70
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 76
typedef unsigned long long int uint_fast64_t;









typedef int16_t intptr_t;
typedef uint16_t uintptr_t;





__extension__ 
#line 93
typedef long long int intmax_t;
__extension__ 
#line 94
typedef unsigned long long int uintmax_t;
# 431 "/usr/lib/ncc/nesc_nx.h"
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 48 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 44 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/string.h" 3
extern void *memset(void *arg_0x2b32ae225020, int arg_0x2b32ae2252a0, size_t arg_0x2b32ae225560);
#line 65
extern void *memset(void *arg_0x2b32ae23d060, int arg_0x2b32ae23d2e0, size_t arg_0x2b32ae23d5a0);
# 62 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdlib.h" 3
#line 59
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;






#line 66
typedef struct __nesc_unnamed4243 {
  long int quot;
  long int rem;
} ldiv_t;
#line 97
void *malloc(size_t size);
# 122 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 19 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 31
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x2b32ae27d170);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x2b32ae282480);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 212
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 265
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/home/loki/tinyos-release-tinyos-2_1_2/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 47 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/intrinsics.h" 3
void __nop(void );



void __dint(void );



void __eint(void );


unsigned int __read_status_register(void );


typedef unsigned int __istate_t;
# 164 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/msp430f1611.h" 3
extern volatile unsigned char ME1 __asm ("__""ME1");
#line 183
extern volatile unsigned char ME2 __asm ("__""ME2");
#line 195
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
#line 267
extern volatile unsigned char P1OUT __asm ("__""P1OUT");

extern volatile unsigned char P1DIR __asm ("__""P1DIR");





extern volatile unsigned char P1IE __asm ("__""P1IE");

extern volatile unsigned char P1SEL __asm ("__""P1SEL");




extern volatile unsigned char P2OUT __asm ("__""P2OUT");

extern volatile unsigned char P2DIR __asm ("__""P2DIR");





extern volatile unsigned char P2IE __asm ("__""P2IE");

extern volatile unsigned char P2SEL __asm ("__""P2SEL");










extern volatile unsigned char P3OUT __asm ("__""P3OUT");

extern volatile unsigned char P3DIR __asm ("__""P3DIR");

extern volatile unsigned char P3SEL __asm ("__""P3SEL");




extern volatile unsigned char P4OUT __asm ("__""P4OUT");

extern volatile unsigned char P4DIR __asm ("__""P4DIR");

extern volatile unsigned char P4SEL __asm ("__""P4SEL");










extern volatile unsigned char P5OUT __asm ("__""P5OUT");

extern volatile unsigned char P5DIR __asm ("__""P5DIR");

extern volatile unsigned char P5SEL __asm ("__""P5SEL");




extern volatile unsigned char P6OUT __asm ("__""P6OUT");

extern volatile unsigned char P6DIR __asm ("__""P6DIR");

extern volatile unsigned char P6SEL __asm ("__""P6SEL");
#line 384
extern volatile unsigned char U0TCTL __asm ("__""U0TCTL");
#line 439
extern volatile unsigned char U1CTL __asm ("__""U1CTL");

extern volatile unsigned char U1TCTL __asm ("__""U1TCTL");



extern volatile unsigned char U1MCTL __asm ("__""U1MCTL");

extern volatile unsigned char U1BR0 __asm ("__""U1BR0");

extern volatile unsigned char U1BR1 __asm ("__""U1BR1");

extern const volatile unsigned char U1RXBUF __asm ("__""U1RXBUF");
#line 595
extern volatile unsigned int TACTL __asm ("__""TACTL");

extern volatile unsigned int TACCTL0 __asm ("__""TACCTL0");

extern volatile unsigned int TACCTL1 __asm ("__""TACCTL1");

extern volatile unsigned int TACCTL2 __asm ("__""TACCTL2");

extern volatile unsigned int TAR __asm ("__""TAR");
#line 720
extern volatile unsigned int TBCCTL0 __asm ("__""TBCCTL0");
#line 734
extern volatile unsigned int TBR __asm ("__""TBR");

extern volatile unsigned int TBCCR0 __asm ("__""TBCCR0");
#line 849
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");

extern volatile unsigned char BCSCTL2 __asm ("__""BCSCTL2");
#line 1021
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
# 343 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 378
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4249 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 433
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 448
enum __nesc_unnamed4250 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 8 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4251 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4252 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 82 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/mac/tkn154/TKN154.h"
#line 43
typedef enum ieee154_status {

  IEEE154_SUCCESS = 0x00, 
  IEEE154_BEACON_LOSS = 0xE0, 
  IEEE154_CHANNEL_ACCESS_FAILURE = 0xE1, 
  IEEE154_COUNTER_ERROR = 0xDB, 
  IEEE154_DENIED = 0xE2, 
  IEEE154_DISABLE_TRX_FAILURE = 0xE3, 
  IEEE154_FRAME_TOO_LONG = 0xE5, 
  IEEE154_IMPROPER_KEY_TYPE = 0xDC, 
  IEEE154_IMPROPER_SECURITY_LEVEL = 0xDD, 
  IEEE154_INVALID_ADDRESS = 0xF5, 
  IEEE154_INVALID_GTS = 0xE6, 
  IEEE154_INVALID_HANDLE = 0xE7, 
  IEEE154_INVALID_INDEX = 0xF9, 
  IEEE154_INVALID_PARAMETER = 0xE8, 
  IEEE154_LIMIT_REACHED = 0xFA, 
  IEEE154_NO_ACK = 0xE9, 
  IEEE154_NO_BEACON = 0xEA, 
  IEEE154_NO_DATA = 0xEB, 
  IEEE154_NO_SHORT_ADDRESS = 0xEC, 
  IEEE154_ON_TIME_TOO_LONG = 0xF6, 
  IEEE154_OUT_OF_CAP = 0xED, 
  IEEE154_PAN_ID_CONFLICT = 0xEE, 
  IEEE154_PAST_TIME = 0xF7, 
  IEEE154_READ_ONLY = 0xFB, 
  IEEE154_REALIGNMENT = 0xEF, 
  IEEE154_SCAN_IN_PROGRESS = 0xFC, 
  IEEE154_SECURITY_ERROR = 0xE4, 
  IEEE154_SUPERFRAME_OVERLAP = 0xFD, 
  IEEE154_TRACKING_OFF = 0xF8, 
  IEEE154_TRANSACTION_EXPIRED = 0xF0, 
  IEEE154_TRANSACTION_OVERFLOW = 0xF1, 
  IEEE154_TX_ACTIVE = 0xF2, 
  IEEE154_UNAVAILABLE_KEY = 0xF3, 
  IEEE154_UNSUPPORTED_ATTRIBUTE = 0xF4, 
  IEEE154_UNSUPPORTED_LEGACY = 0xDE, 
  IEEE154_UNSUPPORTED_SECURITY = 0xDF, 
  IEEE154_PURGED = 0xDA
} ieee154_status_t;






#line 84
typedef enum ieee154_association_status {

  IEEE154_ASSOCIATION_SUCCESSFUL = 0x00, 
  IEEE154_PAN_AT_CAPACITY = 0x01, 
  IEEE154_ACCESS_DENIED = 0x02
} ieee154_association_status_t;





#line 91
typedef enum ieee154_disassociation_reason {

  IEEE154_COORDINATOR_WISHES_DEVICE_TO_LEAVE = 0x01, 
  IEEE154_DEVICE_WISHES_TO_LEAVE = 0x02
} ieee154_disassociation_reason_t;






#line 97
typedef union ieee154_address {


  uint16_t shortAddress;
  uint64_t extendedAddress;
} ieee154_address_t;








#line 104
typedef struct ieee154_security {


  uint8_t SecurityLevel;
  uint8_t KeyIdMode;
  uint8_t KeySource[8];
  uint8_t KeyIndex;
} ieee154_security_t;










#line 113
typedef nx_struct __nesc_unnamed4253 {
  unsigned char __nesc_filler0[1];
} __attribute__((packed)) 






ieee154_CapabilityInformation_t;










#line 124
typedef nx_struct __nesc_unnamed4254 {
  unsigned char __nesc_filler1[2];
} __attribute__((packed)) 






ieee154_SuperframeSpec_t;
#line 150
#line 135
typedef struct ieee154_PANDescriptor {
  uint8_t CoordAddrMode;
  uint16_t CoordPANId;
  ieee154_address_t CoordAddress;
  uint8_t LogicalChannel;
  uint8_t ChannelPage;
  ieee154_SuperframeSpec_t SuperframeSpec;
  bool GTSPermit;
  uint8_t LinkQuality;
  uint32_t TimeStamp;
  ieee154_status_t SecurityFailure;
  uint8_t SecurityLevel;
  uint8_t KeyIdMode;
  uint64_t KeySource;
  uint8_t KeyIndex;
} ieee154_PANDescriptor_t;

enum __nesc_unnamed4255 {

  BEACON_ENABLED_PAN, 
  NONBEACON_ENABLED_PAN, 


  TX_OPTIONS_ACK = 0x01, 
  TX_OPTIONS_GTS = 0x02, 
  TX_OPTIONS_INDIRECT = 0x04, 


  ADDR_MODE_NOT_PRESENT = 0x00, 
  ADDR_MODE_RESERVED = 0x01, 
  ADDR_MODE_SHORT_ADDRESS = 0x02, 
  ADDR_MODE_EXTENDED_ADDRESS = 0x03, 


  ENERGY_DETECTION_SCAN = 0x00, 
  ACTIVE_SCAN = 0x01, 
  PASSIVE_SCAN = 0x02, 
  ORPHAN_SCAN = 0x03, 


  FRAMETYPE_BEACON = 0x00, 
  FRAMETYPE_DATA = 0x01, 
  FRAMETYPE_ACK = 0x02, 
  FRAMETYPE_CMD = 0x03
};





typedef uint8_t ieee154_phyCurrentChannel_t;
typedef uint32_t ieee154_phyChannelsSupported_t;
typedef uint8_t ieee154_phyTransmitPower_t;
typedef uint8_t ieee154_phyCCAMode_t;
typedef uint8_t ieee154_phyCurrentPage_t;
typedef uint16_t ieee154_phyMaxFrameDuration_t;
typedef uint8_t ieee154_phySHRDuration_t;
typedef uint8_t ieee154_phySymbolsPerOctet_t;

typedef uint8_t ieee154_macAckWaitDuration_t;
typedef bool ieee154_macAssociatedPANCoord_t;
typedef bool ieee154_macAssociationPermit_t;
typedef bool ieee154_macAutoRequest_t;
typedef bool ieee154_macBattLifeExt_t;
typedef uint8_t ieee154_macBattLifeExtPeriods_t;
typedef uint8_t *ieee154_macBeaconPayload_t;
typedef uint8_t ieee154_macBeaconPayloadLength_t;
typedef uint8_t ieee154_macBeaconOrder_t;
typedef uint32_t ieee154_macBeaconTxTime_t;
typedef uint8_t ieee154_macBSN_t;
typedef uint64_t ieee154_macCoordExtendedAddress_t;
typedef uint16_t ieee154_macCoordShortAddress_t;
typedef uint8_t ieee154_macDSN_t;
typedef bool ieee154_macGTSPermit_t;
typedef uint8_t ieee154_macMaxBE_t;
typedef uint8_t ieee154_macMaxCSMABackoffs_t;
typedef uint32_t ieee154_macMaxFrameTotalWaitTime_t;
typedef uint8_t ieee154_macMaxFrameRetries_t;
typedef uint8_t ieee154_macMinBE_t;
typedef uint8_t ieee154_macMinLIFSPeriod_t;
typedef uint8_t ieee154_macMinSIFSPeriod_t;
typedef uint16_t ieee154_macPANId_t;
typedef bool ieee154_macPromiscuousMode_t;
typedef uint8_t ieee154_macResponseWaitTime_t;
typedef bool ieee154_macRxOnWhenIdle_t;
typedef bool ieee154_macSecurityEnabled_t;
typedef uint16_t ieee154_macShortAddress_t;
typedef uint8_t ieee154_macSuperframeOrder_t;
typedef uint16_t ieee154_macSyncSymbolOffset_t;
typedef bool ieee154_macTimestampSupported_t;
typedef uint16_t ieee154_macTransactionPersistenceTime_t;


typedef bool ieee154_macPanCoordinator_t;
#line 315
enum __nesc_unnamed4256 {

  IEEE154_aMaxPHYPacketSize = 127
};
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/WIDS.h"
#line 35
typedef enum wids_attack {

  NO_ATTACK = 0X00, 

  CONSTANT_JAMMING = 0X01, 
  DECEPTIVE_JAMMING = 0X02, 
  REACTIVE_JAMMING = 0X03, 
  RANDOM_JAMMING = 0X04, 

  LINKLAYER_JAMMING = 0x10, 
  BACKOFF_MANIPULATION = 0x11, 
  REPLAYPROTECTION_ATTACK = 0x12, 
  GTS_ATTACK = 0x13, 
  ACK_ATTACK = 0x14, 

  SELECTIVE_FORWARDING = 0xA0, 
  SINKHOLE = 0xA1, 
  SYBIL = 0xA2, 
  WORMHOLE = 0xA3, 
  HELLO_FLOODING = 0xA4
} 
wids_attack_t;
#line 94
#line 58
typedef enum wids_observable {

  OBS_1 = 1, 
  OBS_2 = 2, 
  OBS_3 = 3, 
  OBS_4 = 4, 
  OBS_5 = 5, 
  OBS_6 = 6, 
  OBS_7 = 7, 
  OBS_8 = 8, 
  OBS_9 = 9, 
  OBS_10 = 10, 
  OBS_11 = 11, 
  OBS_12 = 12, 
  OBS_13 = 13, 
  OBS_14 = 14, 
  OBS_15 = 15, 



  OBS_16 = 16, 
  OBS_17 = 17, 
  OBS_18 = 18, 
  OBS_19 = 19, 
  OBS_20 = 20, 
  OBS_21 = 21, 
  OBS_22 = 22, 
  OBS_23 = 23, 
  OBS_24 = 24, 
  OBS_25 = 25, 
  OBS_26 = 26, 
  OBS_27 = 27, 
  OBS_28 = 28, 
  OBS_29 = 29, 
  OBS_30 = 30
} 
wids_observable_t;







#line 96
typedef struct wids_rxFrame_detail {

  ieee154_address_t srcAddr;

  uint8_t seqNo;
} 
wids_rxFrame_detail_t;
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/THREATMODEL.h"
enum model_evaluation_mode {

  TIME_SCHEDULED, 
  EVENT_BASED, 
  ON_DEMAND
};


enum wids_alarm_level {

  HIGH = 200, 
  LOW = 50
};







#line 54
typedef struct wids_obs_list {

  wids_observable_t *obs;
  struct wids_obs_list *next;
} 
wids_obs_list_t;









#line 61
typedef struct wids_state {

  wids_attack_t attack;
  uint8_t alarm_level;
  wids_obs_list_t *observables;
  struct wids_state_transition *transitions;
  struct wids_state *next;
} 
wids_state_t;






#line 71
typedef struct wids_state_transition {

  wids_state_t *state;
  struct wids_state_transition *next;
} 
wids_state_transition_t;





#line 78
typedef struct wids_threat_model {

  wids_state_t *states;
} 
wids_threat_model_t;







#line 84
typedef struct wids_state_trace {

  wids_state_t *state;
  uint8_t observation_count;
  uint8_t alarm_value;
} 
wids_state_trace_t;
# 40 "/usr/bin/../lib/gcc/msp430/4.6.3/include/stdarg.h" 3
typedef __builtin_va_list __gnuc_va_list;
#line 102
typedef __gnuc_va_list va_list;
# 52 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdio.h" 3
int __attribute((format(printf, 1, 2))) printf(const char *string, ...);






int putchar(int c);
# 6 "/home/loki/tinyos-release-tinyos-2_1_2/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4257 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4258 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 83 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4259 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4260 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4261 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 121
#line 109
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 123
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 131
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 136
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/cc2420_tkn154/TKN154_PHY.h"
enum __nesc_unnamed4262 {

  IEEE154_SUPPORTED_CHANNELS = 0x07FFF800, 
  IEEE154_SYMBOLS_PER_OCTET = 2, 
  IEEE154_TXPOWER_TOLERANCE = 0x80, 
  IEEE154_SHR_DURATION = 5 * IEEE154_SYMBOLS_PER_OCTET, 
  IEEE154_MAX_FRAME_DURATION = IEEE154_SHR_DURATION + (IEEE154_aMaxPHYPacketSize + 1) * IEEE154_SYMBOLS_PER_OCTET, 
  IEEE154_PREAMBLE_LENGTH = 4 * IEEE154_SYMBOLS_PER_OCTET, 
  IEEE154_SYNC_SYMBOL_OFFSET = 1 * IEEE154_SYMBOLS_PER_OCTET, 
  IEEE154_MIN_LIFS_PERIOD = 40, 
  IEEE154_MIN_SIFS_PERIOD = 12, 
  IEEE154_ACK_WAIT_DURATION = 20 + 12 + IEEE154_SHR_DURATION + 6 * IEEE154_SYMBOLS_PER_OCTET, 
  IEEE154_TIMESTAMP_SUPPORTED = TRUE
};
# 4 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/mac/tkn154/timer/Timer62500hz.h"
typedef struct __nesc_unnamed4263 {
} 
#line 4
T62500hz;
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/mac/tkn154/TKN154_platform.h"
enum __nesc_unnamed4264 {


  IEEE154_RADIO_TX_DELAY = 400, 



  IEEE154_RADIO_RX_DELAY = 400, 




  BEACON_PAYLOAD_UPDATE_INTERVAL = 2500
};
# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/mac/tkn154/TKN154_MAC.h"
enum __nesc_unnamed4265 {

  IEEE154_phyCurrentChannel = 0x00, 
  IEEE154_phyChannelsSupported = 0x01, 
  IEEE154_phyTransmitPower = 0x02, 
  IEEE154_phyCCAMode = 0x03, 
  IEEE154_phyCurrentPage = 0x04, 
  IEEE154_phyMaxFrameDuration = 0x05, 
  IEEE154_phySHRDuration = 0x06, 
  IEEE154_phySymbolsPerOctet = 0x07, 


  IEEE154_macAckWaitDuration = 0x40, 
  IEEE154_macAssociatedPANCoord = 0x56, 
  IEEE154_macAssociationPermit = 0x41, 
  IEEE154_macAutoRequest = 0x42, 
  IEEE154_macBattLifeExt = 0x43, 
  IEEE154_macBattLifeExtPeriods = 0x44, 
  IEEE154_macBeaconPayload = 0x45, 
  IEEE154_macBeaconPayloadLength = 0x46, 
  IEEE154_macBeaconOrder = 0x47, 
  IEEE154_macBeaconTxTime = 0x48, 
  IEEE154_macBSN = 0x49, 
  IEEE154_macCoordExtendedAddress = 0x4A, 
  IEEE154_macCoordShortAddress = 0x4B, 
  IEEE154_macDSN = 0x4C, 
  IEEE154_macGTSPermit = 0x4D, 
  IEEE154_macMaxBE = 0x57, 
  IEEE154_macMaxCSMABackoffs = 0x4E, 
  IEEE154_macMaxFrameTotalWaitTime = 0x58, 
  IEEE154_macMaxFrameRetries = 0x59, 
  IEEE154_macMinBE = 0x4F, 
  IEEE154_macMinLIFSPeriod = 0xA0, 
  IEEE154_macMinSIFSPeriod = 0xA1, 
  IEEE154_macPANId = 0x50, 
  IEEE154_macPromiscuousMode = 0x51, 
  IEEE154_macResponseWaitTime = 0x5A, 
  IEEE154_macRxOnWhenIdle = 0x52, 
  IEEE154_macSecurityEnabled = 0x5D, 
  IEEE154_macShortAddress = 0x53, 
  IEEE154_macSuperframeOrder = 0x54, 
  IEEE154_macSyncSymbolOffset = 0x5B, 
  IEEE154_macTimestampSupported = 0x5C, 
  IEEE154_macTransactionPersistenceTime = 0x55, 


  IEEE154_macPanCoordinator = 0xF0
};

enum __nesc_unnamed4266 {

  MHR_INDEX_FC1 = 0, 
  MHR_INDEX_FC2 = 1, 
  MHR_INDEX_SEQNO = 2, 
  MHR_INDEX_ADDRESS = 3, 
  MHR_MAX_LEN = 23, 


  FC1_FRAMETYPE_BEACON = 0x00, 
  FC1_FRAMETYPE_DATA = 0x01, 
  FC1_FRAMETYPE_ACK = 0x02, 
  FC1_FRAMETYPE_CMD = 0x03, 
  FC1_FRAMETYPE_MASK = 0x07, 

  FC1_SECURITY_ENABLED = 0x08, 
  FC1_FRAME_PENDING = 0x10, 
  FC1_ACK_REQUEST = 0x20, 
  FC1_PAN_ID_COMPRESSION = 0x40, 

  FC2_DEST_MODE_SHORT = 0x08, 
  FC2_DEST_MODE_EXTENDED = 0x0c, 
  FC2_DEST_MODE_MASK = 0x0c, 
  FC2_DEST_MODE_OFFSET = 2, 

  FC2_SRC_MODE_SHORT = 0x80, 
  FC2_SRC_MODE_EXTENDED = 0xc0, 
  FC2_SRC_MODE_MASK = 0xc0, 
  FC2_SRC_MODE_OFFSET = 6, 

  FC2_FRAME_VERSION_1 = 0x10, 
  FC2_FRAME_VERSION_2 = 0x20, 
  FC2_FRAME_VERSION_MASK = 0x30
};








enum __nesc_unnamed4267 {





  RADIO_CLIENT_SCAN = 0U, 
  RADIO_CLIENT_PIB = 1U, 
  RADIO_CLIENT_PROMISCUOUSMODE = 2U, 

  RADIO_CLIENT_BEACONTRANSMIT = 3U, 
  RADIO_CLIENT_COORDBROADCAST = 4U, 
  RADIO_CLIENT_COORDCAP = 5U, 
  RADIO_CLIENT_COORDCFP = 6U, 
  RADIO_CLIENT_COORD_INACTIVE_PERIOD = 7U, 

  RADIO_CLIENT_BEACONSYNCHRONIZE = 8U, 
  RADIO_CLIENT_DEVICECAP = 9U, 
  RADIO_CLIENT_DEVICECFP = 10U, 
  RADIO_CLIENT_DEVICE_INACTIVE_PERIOD = 11U
};

enum __nesc_unnamed4268 {

  OUTGOING_SUPERFRAME, 
  INCOMING_SUPERFRAME
};
#line 205
#line 183
typedef struct __nesc_unnamed4269 {

  uint8_t length;
  uint8_t mhr[MHR_MAX_LEN];
#line 198
  uint8_t network;



  uint8_t type;
} 

ieee154_header_t;





#line 207
typedef nx_struct __nesc_unnamed4270 {
  nx_uint32_t timestamp;
  nx_int8_t rssi;
  nx_uint8_t linkQuality;
} __attribute__((packed)) ieee154_metadata_t;










#line 213
typedef struct __nesc_unnamed4271 {

  ieee154_header_t *header;
  uint8_t *payload;
  ieee154_metadata_t *metadata;
  uint8_t headerLen;
  uint8_t payloadLen;
  uint8_t client;
  uint8_t handle;
} ieee154_txframe_t;





#line 224
typedef struct __nesc_unnamed4272 {

  ieee154_header_t header;
  ieee154_metadata_t metadata;
} ieee154_txcontrol_t;






#line 230
typedef struct ieee154_csma {
  uint8_t BE;
  uint8_t macMaxBE;
  uint8_t macMaxCsmaBackoffs;
  uint8_t NB;
} ieee154_csma_t;





#line 237
typedef struct __nesc_unnamed4273 {
  uint32_t transactionTime;
  ieee154_txframe_t *frame;
  ieee154_csma_t csma;
} ieee154_cap_frame_backup_t;




enum __nesc_unnamed4274 {
  CMD_FRAME_ASSOCIATION_REQUEST = 1, 
  CMD_FRAME_ASSOCIATION_RESPONSE = 2, 
  CMD_FRAME_DISASSOCIATION_NOTIFICATION = 3, 
  CMD_FRAME_DATA_REQUEST = 4, 
  CMD_FRAME_PAN_ID_CONFLICT_NOTIFICATION = 5, 
  CMD_FRAME_ORPHAN_NOTIFICATION = 6, 
  CMD_FRAME_BEACON_REQUEST = 7, 
  CMD_FRAME_COORDINATOR_REALIGNMENT = 8, 
  CMD_FRAME_GTS_REQUEST = 9
};

enum __nesc_unnamed4275 {

  BEACON_INDEX_SF_SPEC1 = 0, 
  BEACON_INDEX_SF_SPEC2 = 1, 
  BEACON_INDEX_GTS_SPEC = 2, 

  SF_SPEC1_BO_MASK = 0x0F, 
  SF_SPEC1_BO_OFFSET = 0, 
  SF_SPEC1_SO_MASK = 0xF0, 
  SF_SPEC1_SO_OFFSET = 4, 

  SF_SPEC2_FINAL_CAPSLOT_MASK = 0x0F, 
  SF_SPEC2_FINAL_CAPSLOT_OFFSET = 0, 
  SF_SPEC2_BATT_LIFE_EXT = 0x10, 
  SF_SPEC2_PAN_COORD = 0x40, 
  SF_SPEC2_ASSOCIATION_PERMIT = 0x80, 

  GTS_DESCRIPTOR_COUNT_MASK = 0x07, 
  GTS_DESCRIPTOR_COUNT_OFFSET = 0, 
  GTS_LENGTH_MASK = 0xF0, 
  GTS_LENGTH_OFFSET = 4, 
  GTS_SPEC_PERMIT = 0x80, 

  PENDING_ADDRESS_SHORT_MASK = 0x07, 
  PENDING_ADDRESS_EXT_MASK = 0x70
};

enum __nesc_unnamed4276 {

  IEEE154_aTurnaroundTime = 12, 

  FRAMECTL_LENGTH_MASK = 0x7F, 
  FRAMECTL_PROMISCUOUS = 0x80
};


enum __nesc_unnamed4277 {

  IEEE154_aNumSuperframeSlots = 16, 
  IEEE154_aMaxMPDUUnsecuredOverhead = 25, 
  IEEE154_aMinMPDUOverhead = 9, 
  IEEE154_aBaseSlotDuration = 60, 
  IEEE154_aBaseSuperframeDuration = IEEE154_aBaseSlotDuration * IEEE154_aNumSuperframeSlots, 
  IEEE154_aGTSDescPersistenceTime = 4, 
  IEEE154_aMaxBeaconOverhead = 75, 
  IEEE154_aMaxBeaconPayloadLength = IEEE154_aMaxPHYPacketSize - IEEE154_aMaxBeaconOverhead, 
  IEEE154_aMaxLostBeacons = 4, 
  IEEE154_aMaxMACSafePayloadSize = IEEE154_aMaxPHYPacketSize - IEEE154_aMaxMPDUUnsecuredOverhead, 
  IEEE154_aMaxMACPayloadSize = IEEE154_aMaxPHYPacketSize - IEEE154_aMinMPDUOverhead, 
  IEEE154_aMaxSIFSFrameSize = 18, 
  IEEE154_aMinCAPLength = 440, 
  IEEE154_aUnitBackoffPeriod = 20
};


typedef bool token_requested_t  ;
# 20 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/mac/tkn154/platform_message.h"
#line 17
typedef union message_header {
  ieee154_header_t ieee154;
  serial_header_t serial;
} message_header_t;


#line 22
typedef union TOSRadioFooter {
} message_footer_t;



#line 25
typedef union TOSRadioMetadata {
  ieee154_metadata_t ieee154;
} message_metadata_t;
# 19 "/home/loki/tinyos-release-tinyos-2_1_2/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[118];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 72 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/printf.h"
int printfflush();






#line 77
typedef nx_struct printf_msg {
  nx_uint8_t buffer[28];
} __attribute__((packed)) printf_msg_t;

enum __nesc_unnamed4278 {
  AM_PRINTF_MSG = 100
};
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4279 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 75
#line 62
typedef struct __nesc_unnamed4280 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 87
#line 77
typedef struct __nesc_unnamed4281 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 102
#line 89
typedef struct __nesc_unnamed4282 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 41 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4283 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4284 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4285 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4286 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 43 "/home/loki/tinyos-release-tinyos-2_1_2/tos/types/Leds.h"
enum __nesc_unnamed4287 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 11 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/UTIL.h"
#line 5
typedef struct list {

  void *element;
  void *key;
  struct list *next;
} 
list_t;







#line 14
typedef struct linked_list {

  void *element;
  struct linked_list *next;
} 
linked_list_t;
# 15 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HEAP.h"
#line 5
typedef struct heap_node {

  void *element;
  uint8_t key;
  uint8_t size;
  struct heap_node *parent;
  struct heap_node *lsibling;
  struct heap_node *rsibling;
  struct heap_node *child;
} 
heap_node_t;







#line 17
typedef struct binomial_heap {

  heap_node_t *tree;
  struct binomial_heap *prev;
  struct binomial_heap *next;
} 
binomial_heap_t;
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4288 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4289 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4290 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4291 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4292 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4293 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4294 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;
#line 169
#line 150
typedef enum __nesc_unnamed4295 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4296 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4297 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4298 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;

msp430_uart_union_config_t msp430_uart_default_config = { 
{ 
.utxe = 1, 
.urxe = 1, 
.ubr = UBR_1MHZ_57600, 
.umctl = UMCTL_1MHZ_57600, 
.ssel = 0x02, 
.pena = 0, 
.pev = 0, 
.spb = 0, 
.clen = 1, 
.listen = 0, 
.mm = 0, 
.ckpl = 0, 
.urxse = 0, 
.urxeie = 1, 
.urxwie = 0, 
.utxe = 1, 
.urxe = 1 } };
#line 248
#line 240
typedef struct __nesc_unnamed4299 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4300 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4301 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4302 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4303 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4304 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4305 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 33 "/home/loki/tinyos-release-tinyos-2_1_2/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
typedef wids_state_trace_t PQC__WPriorityQueue__t;
typedef uint8_t PQC__WPriorityQueue__p;
typedef TMilli PQC__Timer__precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4306 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
typedef wids_state_trace_t /*PQAppC.FH*/FibonacciHeapC__0__elementType;
typedef /*PQAppC.FH*/FibonacciHeapC__0__elementType /*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__t;
typedef uint8_t /*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__p;
typedef uint8_t /*PQAppC.FH*/FibonacciHeapC__0__HashMap__k;
typedef heap_node_t /*PQAppC.FH*/FibonacciHeapC__0__HashMap__e;
typedef uint8_t /*PQAppC.HashMapC*/HashMapC__0__key_type;
typedef heap_node_t /*PQAppC.HashMapC*/HashMapC__0__el_type;
typedef /*PQAppC.HashMapC*/HashMapC__0__key_type /*PQAppC.HashMapC*/HashMapC__0__Hash__key_type;
typedef /*PQAppC.HashMapC*/HashMapC__0__key_type /*PQAppC.HashMapC*/HashMapC__0__HashMap__k;
typedef /*PQAppC.HashMapC*/HashMapC__0__el_type /*PQAppC.HashMapC*/HashMapC__0__HashMap__e;
typedef uint8_t /*PQAppC.SimpleHashC*/SimpleHashC__0__Hash__key_type;
enum /*PlatformSerialC.UartC*/Msp430Uart1C__0____nesc_unnamed4307 {
  Msp430Uart1C__0__CLIENT_ID = 0U
};
typedef T32khz /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__precision_tag;
typedef uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__size_type;
enum /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0____nesc_unnamed4308 {
  Msp430Usart1C__0__CLIENT_ID = 0U
};
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 62
static error_t MotePlatformC__Init__init(void );
# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 43
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 45
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 51 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2b32ae8ec8b0);
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2b32ae8ec8b0);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 41 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 76 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b32ae774e60);
# 75 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b32ae774e60);
# 57 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 60 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Boot.nc"
static void PQC__Boot__booted(void );
# 75 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static void PQC__insert__runTask(void );
# 83 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
static void PQC__Timer__fired(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Toggle(void );




static void LedsP__Leds__led1On(void );




static void LedsP__Leds__led1Off(void );
#line 94
static void LedsP__Leds__led2Off(void );
#line 89
static void LedsP__Leds__led2On(void );
# 99 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 58
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );




static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 75 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 78 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 136 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 75 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 83 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 83
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b32aedfc3f0);
# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b32aedfc3f0, 
# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
uint32_t dt);
# 82 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 10 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/WPriorityQueue.nc"
static void /*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__insert(/*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__t *element, /*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__p priority);
#line 6
static /*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__t */*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__findMax(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t /*PQAppC.FH*/FibonacciHeapC__0__Init__init(void );
#line 62
static error_t /*PQAppC.HashMapC*/HashMapC__0__Init__init(void );
# 4 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HashMap.nc"
static error_t /*PQAppC.HashMapC*/HashMapC__0__HashMap__insert(/*PQAppC.HashMapC*/HashMapC__0__HashMap__e *element, /*PQAppC.HashMapC*/HashMapC__0__HashMap__k key);
# 5 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HashFunction.nc"
static uint8_t /*PQAppC.SimpleHashC*/SimpleHashC__0__Hash__getHash(/*PQAppC.SimpleHashC*/SimpleHashC__0__Hash__key_type key);
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/Putchar.nc"
static int SerialPrintfP__Putchar__putchar(int c);
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t SerialPrintfP__Init__init(void );
# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/StdControl.nc"
static error_t SerialPrintfP__StdControl__start(void );
# 59 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(
# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcd280);
# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartByte.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartByte__send(
# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcb6a0, 
# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartByte.nc"
uint8_t byte);
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefc9890);
# 79 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcc020, 
# 79 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcc020, 
# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcc020, 
# 53 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 82 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );
# 97 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefca4e0);
# 102 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefca4e0);
# 128 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefca4e0);
# 97 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(
# 43 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcf020);
# 102 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(
# 43 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcf020);
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(
# 51 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefe69b0, 
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(
# 51 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefe69b0);
# 143 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart1P__Usart__enableUartRx(void );
#line 123
static void HplMsp430Usart1P__Usart__enableUart(void );
#line 97
static void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart1P__Usart__disableIntr(void );

static void HplMsp430Usart1P__Usart__enableTxIntr(void );
#line 90
static void HplMsp430Usart1P__Usart__setUmctl(uint8_t umctl);
#line 133
static void HplMsp430Usart1P__Usart__enableUartTx(void );
#line 178
static void HplMsp430Usart1P__Usart__disableTxIntr(void );
#line 148
static void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 182
static void HplMsp430Usart1P__Usart__enableIntr(void );




static bool HplMsp430Usart1P__Usart__isTxIntrPending(void );
#line 207
static void HplMsp430Usart1P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart1P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart1P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart1P__Usart__disableUart(void );
#line 174
static void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 202
static void HplMsp430Usart1P__Usart__clrTxIntr(void );
#line 158
static void HplMsp430Usart1P__Usart__disableSpi(void );
#line 138
static void HplMsp430Usart1P__Usart__disableUartTx(void );
# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/AsyncStdControl.nc"
static error_t HplMsp430Usart1P__AsyncStdControl__start(void );
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b32af10a020, 
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b32af10a020);
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 61 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b32af151840);
# 59 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b32af14fc40);
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
# 97 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b32af1524b0);
# 102 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b32af1524b0);
# 128 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b32af1524b0);
# 90 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 75 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 81 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 102 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static void TelosSerialP__Resource__granted(void );
# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/StdControl.nc"
static error_t TelosSerialP__StdControl__start(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t PutcharP__Init__init(void );
#line 62
static error_t PlatformP__MoteInit__init(void );
#line 62
static error_t PlatformP__MoteClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 10 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 43 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 51 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4309 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 79
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 100
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 163
static inline void Msp430ClockP__startTimerB(void );
#line 175
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 204
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 229
static inline error_t Msp430ClockP__Init__init(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 51 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2b32ae8ec8b0);
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 126 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2b32ae8ec8b0);
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 81
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000C)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000A)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x001A)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0018)))  ;
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 59 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 112
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b32ae774e60);
# 76 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4310 {

  SchedulerBasicP__NUM_TASKS = 4U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 97
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 124
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 170
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 10 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/WPriorityQueue.nc"
static void PQC__WPriorityQueue__insert(PQC__WPriorityQueue__t *element, PQC__WPriorityQueue__p priority);
#line 6
static PQC__WPriorityQueue__t *PQC__WPriorityQueue__findMax(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t PQC__Init__init(void );
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static error_t PQC__insert__postTask(void );
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Leds.nc"
static void PQC__Leds__led0Toggle(void );




static void PQC__Leds__led1On(void );




static void PQC__Leds__led1Off(void );
#line 94
static void PQC__Leds__led2Off(void );
#line 89
static void PQC__Leds__led2On(void );
# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
static void PQC__Timer__startPeriodic(uint32_t dt);
# 25 "PQC.nc"
enum PQC____nesc_unnamed4311 {
#line 25
  PQC__insert = 0U
};
#line 25
typedef int PQC____nesc_sillytask_insert[PQC__insert];
#line 16
bool PQC__s = FALSE;
uint8_t PQC__score = 1;

static inline void PQC__Boot__booted(void );





static void PQC__insert__runTask(void );
#line 51
static inline void PQC__Timer__fired(void );
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );



static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );





static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );
static void LedsP__Led1__clr(void );




static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
static void LedsP__Led2__clr(void );
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 84
static inline void LedsP__Leds__led0Toggle(void );




static inline void LedsP__Leds__led1On(void );




static inline void LedsP__Leds__led1Off(void );









static inline void LedsP__Leds__led2On(void );




static inline void LedsP__Leds__led2Off(void );
# 65 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );

static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 85 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 85 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void );
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 41 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 57 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 65
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 82 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4312 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 133
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 77 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4313 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 147
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 162
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 177
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 109 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 83 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 74 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4314 {
#line 74
  AlarmToTimerC__0__fired = 1U
};
#line 74
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 55
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 71
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 136 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 129
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b32aedfc3f0);
#line 71
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4315 {
#line 71
  VirtualizeTimerC__0__updateFromTimer = 2U
};
#line 71
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4316 {

  VirtualizeTimerC__0__NUM_TIMERS = 1U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4317 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 100
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 139
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);
#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
static error_t /*PQAppC.FH*/FibonacciHeapC__0__HashMapInit__init(void );
# 4 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HashMap.nc"
static error_t /*PQAppC.FH*/FibonacciHeapC__0__HashMap__insert(/*PQAppC.FH*/FibonacciHeapC__0__HashMap__e *element, /*PQAppC.FH*/FibonacciHeapC__0__HashMap__k key);
# 15 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/FibonacciHeapC.nc"
static __inline void /*PQAppC.FH*/FibonacciHeapC__0__heapNodeInit(heap_node_t *node);









static __inline void /*PQAppC.FH*/FibonacciHeapC__0__binHeapInit(binomial_heap_t *heap);





binomial_heap_t */*PQAppC.FH*/FibonacciHeapC__0__heap;
binomial_heap_t */*PQAppC.FH*/FibonacciHeapC__0__heapPtr;

heap_node_t */*PQAppC.FH*/FibonacciHeapC__0__max = (void *)0;

static inline void /*PQAppC.FH*/FibonacciHeapC__0__ristruttura(void );

static inline error_t /*PQAppC.FH*/FibonacciHeapC__0__Init__init(void );



static inline void /*PQAppC.FH*/FibonacciHeapC__0__setMax(void );
#line 64
static inline /*PQAppC.FH*/FibonacciHeapC__0__elementType */*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__findMax(void );
#line 106
static inline void /*PQAppC.FH*/FibonacciHeapC__0__ristruttura(void );
#line 163
static inline void /*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__insert(/*PQAppC.FH*/FibonacciHeapC__0__elementType *element, uint8_t priority);
# 5 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HashFunction.nc"
static uint8_t /*PQAppC.HashMapC*/HashMapC__0__Hash__getHash(/*PQAppC.HashMapC*/HashMapC__0__Hash__key_type key);
# 43 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HashMapC.nc"
list_t */*PQAppC.HashMapC*/HashMapC__0__hashmap[10];

static inline error_t /*PQAppC.HashMapC*/HashMapC__0__Init__init(void );







static inline error_t /*PQAppC.HashMapC*/HashMapC__0__HashMap__insert(/*PQAppC.HashMapC*/HashMapC__0__el_type *element, /*PQAppC.HashMapC*/HashMapC__0__key_type key);
# 9 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/SimpleHashC.nc"
static inline uint8_t /*PQAppC.SimpleHashC*/SimpleHashC__0__Hash__getHash(uint8_t key);
# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartByte.nc"
static error_t SerialPrintfP__UartByte__send(uint8_t byte);
# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/StdControl.nc"
static error_t SerialPrintfP__UartControl__start(void );
# 50 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/SerialPrintfP.nc"
static inline error_t SerialPrintfP__Init__init(void );



static inline error_t SerialPrintfP__StdControl__start(void );









int printfflush(void )   ;




static inline int SerialPrintfP__Putchar__putchar(int c);
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefc9890);
# 181 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableTxIntr(void );
#line 178
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableTxIntr(void );



static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void );




static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__isTxIntrPending(void );
#line 224
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data);
#line 174
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 202
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__clrTxIntr(void );
# 79 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcc020, 
# 79 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcc020, 
# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcc020, 
# 53 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 97 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefca4e0);
# 128 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefca4e0);
# 102 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(
# 43 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x2b32aefcf020);
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len;
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len;
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;
#line 60
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos;
#line 61
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner;

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id);
#line 85
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id);
#line 101
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id);
#line 134
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data);
#line 162
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id);
#line 178
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartByte__send(uint8_t id, uint8_t data);
#line 208
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );

static inline bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id);

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id);

static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
# 99 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__UCLK__selectIOFunc(void );
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart1P__Interrupts__txDone(void );
# 99 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__URXD__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__URXD__selectModuleFunc(void );






static void HplMsp430Usart1P__UTXD__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__UTXD__selectModuleFunc(void );






static void HplMsp430Usart1P__SOMI__selectIOFunc(void );
#line 99
static void HplMsp430Usart1P__SIMO__selectIOFunc(void );
# 87 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static volatile uint8_t HplMsp430Usart1P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430Usart1P__ME2 __asm ("0x0005");
static volatile uint8_t HplMsp430Usart1P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430Usart1P__U1TCTL __asm ("0x0079");
static volatile uint8_t HplMsp430Usart1P__U1RCTL __asm ("0x007A");
static volatile uint8_t HplMsp430Usart1P__U1TXBUF __asm ("0x007F");



void sig_UART1RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0006)))  ;




void sig_UART1TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0004)))  ;



static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void );
#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 203
static inline void HplMsp430Usart1P__Usart__enableUart(void );







static inline void HplMsp430Usart1P__Usart__disableUart(void );








static inline void HplMsp430Usart1P__Usart__enableUartTx(void );




static inline void HplMsp430Usart1P__Usart__disableUartTx(void );





static inline void HplMsp430Usart1P__Usart__enableUartRx(void );




static inline void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 251
static inline void HplMsp430Usart1P__Usart__disableSpi(void );
#line 283
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config);









static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 318
static inline bool HplMsp430Usart1P__Usart__isTxIntrPending(void );
#line 339
static inline void HplMsp430Usart1P__Usart__clrTxIntr(void );







static inline void HplMsp430Usart1P__Usart__clrIntr(void );







static inline void HplMsp430Usart1P__Usart__disableTxIntr(void );



static inline void HplMsp430Usart1P__Usart__disableIntr(void );










static inline void HplMsp430Usart1P__Usart__enableTxIntr(void );






static inline void HplMsp430Usart1P__Usart__enableIntr(void );






static inline void HplMsp430Usart1P__Usart__tx(uint8_t data);
# 90 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b32af10a020, 
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b32af10a020);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4318 {
#line 49
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[1U];



static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 61 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b32af151840);
# 59 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b32af14fc40);
# 81 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 102 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b32af1524b0);
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4319 {
#line 75
  ArbiterP__0__grantedTask = 3U
};
#line 75
typedef int /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4320 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4321 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4322 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
#line 93
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 133
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static inline bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 190
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);



static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);









static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void );
# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/AsyncStdControl.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void );
# 74 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );
# 97 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
static error_t TelosSerialP__Resource__immediateRequest(void );
# 8 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosa/TelosSerialP.nc"
msp430_uart_union_config_t TelosSerialP__msp430_uart_telos_config = { { .ubr = UBR_1MHZ_115200, .umctl = UMCTL_1MHZ_115200, .ssel = 0x02, .pena = 0, .pev = 0, .spb = 0, .clen = 1, .listen = 0, .mm = 0, .ckpl = 0, .urxse = 0, .urxeie = 1, .urxwie = 0, .utxe = 1, .urxe = 1 } };

static inline error_t TelosSerialP__StdControl__start(void );






static inline void TelosSerialP__Resource__granted(void );

static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/Putchar.nc"
static int PutcharP__Putchar__putchar(int c);
# 98 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/PutcharP.nc"
static inline error_t PutcharP__Init__init(void );








int putchar(int c) __attribute((noinline))   ;
# 397 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 196 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x2b32ae8ec8b0){
#line 39
  switch (arg_0x2b32ae8ec8b0) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x2b32ae8ec8b0);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 126 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 39
}
#line 39
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4323 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4324 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4325 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 45
}
#line 45
# 131 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 134
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 39
}
#line 39
# 126 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 39
}
#line 39
# 196 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 114 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 177 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 82 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
}
#line 82
# 133 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 208 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void )
#line 208
{
}

# 82 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow();
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
}
#line 82
# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 48
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 81 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 82
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 78 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 162 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 78 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4326 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 97 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 81 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 71 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 130 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 57 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 44 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 41 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 43 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 192 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4327 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void )
{
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4328 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4329 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4330 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4331 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4332 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 131 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 134
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 39
}
#line 39
# 124 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 48
}
#line 48
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 40 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 40
}
#line 40
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 48
}
#line 48
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 40 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 40
}
#line 40
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 48
}
#line 48
# 48 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 40 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 40
}
#line 40
# 63 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 85 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = LedsP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
  __nop();
#line 12
  __nop();
}

# 89 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TAR;

#line 8
  while (TAR - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = MotePlatformC__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 163 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 151
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}

#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 145
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 43 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 43
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 43
}
#line 43
# 100 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;









  Msp430ClockP__TACTL = 0x0200 | 0x0002;
}

#line 140
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 42
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 42
}
#line 42
# 79 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~0x02;
}

#line 135
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 41 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 41
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 41
}
#line 41
# 181 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TAR;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 67
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TACTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 40 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 40
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 40
}
#line 40
# 229 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TACTL = 0x0004;
  Msp430ClockP__TAIV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 10 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  WDTCTL = 0x5A00 + 0x0080;
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 65 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 19 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosa/TelosSerialP.nc"
static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void )
#line 19
{
  return &TelosSerialP__msp430_uart_telos_config;
}

# 214 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id)
#line 214
{
  return &msp430_uart_default_config;
}

# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
inline static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(uint8_t arg_0x2b32aefc9890){
#line 39
  union __nesc_unnamed4298 *__nesc_result;
#line 39

#line 39
  switch (arg_0x2b32aefc9890) {
#line 39
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 39
      __nesc_result = TelosSerialP__Msp430UartConfigure__getConfig();
#line 39
      break;
#line 39
    default:
#line 39
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(arg_0x2b32aefc9890);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 359 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableIntr(void )
#line 359
{
  HplMsp430Usart1P__IE2 &= ~(0x20 | 0x10);
}

#line 347
static inline void HplMsp430Usart1P__Usart__clrIntr(void )
#line 347
{
  HplMsp430Usart1P__IFG2 &= ~(0x20 | 0x10);
}

#line 159
static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset)
#line 159
{
  if (reset) {
    U1CTL = 0x01;
    }
  else {
#line 163
    U1CTL &= ~0x01;
    }
}

# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 7);
}

# 99 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 6);
}

# 99 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc();
#line 99
}
#line 99
# 211 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUart(void )
#line 211
{
  /* atomic removed: atomic calls only */
#line 212
  {
    HplMsp430Usart1P__ME2 &= ~(0x20 | 0x10);
    HplMsp430Usart1P__UTXD__selectIOFunc();
    HplMsp430Usart1P__URXD__selectIOFunc();
  }
}

# 65 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 6;
}

# 92 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc();
#line 92
}
#line 92
# 220 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartTx(void )
#line 220
{
  HplMsp430Usart1P__UTXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 0x20;
}

#line 236
static inline void HplMsp430Usart1P__Usart__disableUartRx(void )
#line 236
{
  HplMsp430Usart1P__ME2 &= ~0x10;
  HplMsp430Usart1P__URXD__selectIOFunc();
}

# 65 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 7;
}

# 92 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc();
#line 92
}
#line 92
# 231 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartRx(void )
#line 231
{
  HplMsp430Usart1P__URXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 0x10;
}

#line 225
static inline void HplMsp430Usart1P__Usart__disableUartTx(void )
#line 225
{
  HplMsp430Usart1P__ME2 &= ~0x20;
  HplMsp430Usart1P__UTXD__selectIOFunc();
}

#line 203
static inline void HplMsp430Usart1P__Usart__enableUart(void )
#line 203
{
  /* atomic removed: atomic calls only */
#line 204
  {
    HplMsp430Usart1P__UTXD__selectModuleFunc();
    HplMsp430Usart1P__URXD__selectModuleFunc();
  }
  HplMsp430Usart1P__ME2 |= 0x20 | 0x10;
}

#line 151
static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control)
#line 151
{
  U1MCTL = control;
}

#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control)
#line 140
{
  /* atomic removed: atomic calls only */
#line 141
  {
    U1BR0 = control & 0x00FF;
    U1BR1 = (control >> 8) & 0x00FF;
  }
}

#line 283
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config)
#line 283
{

  U1CTL = (config->uartRegisters.uctl & ~0x04) | 0x01;
  HplMsp430Usart1P__U1TCTL = config->uartRegisters.utctl;
  HplMsp430Usart1P__U1RCTL = config->uartRegisters.urctl;

  HplMsp430Usart1P__Usart__setUbr(config->uartRegisters.ubr);
  HplMsp430Usart1P__Usart__setUmctl(config->uartRegisters.umctl);
}

# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 3);
}

# 99 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 2);
}

# 99 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 1);
}

# 99 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc();
#line 99
}
#line 99
# 251 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableSpi(void )
#line 251
{
  /* atomic removed: atomic calls only */
#line 252
  {
    HplMsp430Usart1P__ME2 &= ~0x10;
    HplMsp430Usart1P__SIMO__selectIOFunc();
    HplMsp430Usart1P__SOMI__selectIOFunc();
    HplMsp430Usart1P__UCLK__selectIOFunc();
  }
}

#line 293
static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config)
#line 293
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 295
    {
      HplMsp430Usart1P__Usart__resetUsart(TRUE);
      HplMsp430Usart1P__Usart__disableSpi();
      HplMsp430Usart1P__configUart(config);
      if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 1) {
          HplMsp430Usart1P__Usart__enableUart();
        }
      else {
#line 301
        if (config->uartConfig.utxe == 0 && config->uartConfig.urxe == 1) {
            HplMsp430Usart1P__Usart__disableUartTx();
            HplMsp430Usart1P__Usart__enableUartRx();
          }
        else {
#line 304
          if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 0) {
              HplMsp430Usart1P__Usart__disableUartRx();
              HplMsp430Usart1P__Usart__enableUartTx();
            }
          else 
#line 307
            {
              HplMsp430Usart1P__Usart__disableUart();
            }
          }
        }
#line 310
      HplMsp430Usart1P__Usart__resetUsart(FALSE);
      HplMsp430Usart1P__Usart__clrIntr();
      HplMsp430Usart1P__Usart__disableIntr();
    }
#line 313
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 174 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config){
#line 174
  HplMsp430Usart1P__Usart__setModeUart(config);
#line 174
}
#line 174
# 377 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableIntr(void )
#line 377
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 378
    {
      HplMsp430Usart1P__IFG2 &= ~(0x20 | 0x10);
      HplMsp430Usart1P__IE2 |= 0x20 | 0x10;
    }
#line 381
    __nesc_atomic_end(__nesc_atomic); }
}

# 182 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void ){
#line 182
  HplMsp430Usart1P__Usart__enableIntr();
#line 182
}
#line 182
# 17 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosa/TelosSerialP.nc"
static inline void TelosSerialP__Resource__granted(void )
#line 17
{
}

# 218 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id)
#line 218
{
}

# 102 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(uint8_t arg_0x2b32aefcf020){
#line 102
  switch (arg_0x2b32aefcf020) {
#line 102
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 102
      TelosSerialP__Resource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(arg_0x2b32aefcf020);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 101 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id)
#line 101
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(id);
}

# 202 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x2b32af1524b0){
#line 102
  switch (arg_0x2b32af1524b0) {
#line 102
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 102
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x2b32af1524b0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 65 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 71 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 72
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 78 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 78
}
#line 78
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
inline static error_t PQC__insert__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(PQC__insert);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void )
#line 58
{
#line 58
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    * (volatile uint8_t * )49U ^= 0x01 << 4;
#line 58
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle();
#line 58
}
#line 58
# 50 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle();
}

# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__toggle(void ){
#line 42
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle();
#line 42
}
#line 42
# 84 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Toggle(void )
#line 84
{
  LedsP__Led0__toggle();
  ;
#line 86
  ;
}

# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Leds.nc"
inline static void PQC__Leds__led0Toggle(void ){
#line 67
  LedsP__Leds__led0Toggle();
#line 67
}
#line 67
# 51 "PQC.nc"
static inline void PQC__Timer__fired(void )
{
  PQC__Leds__led0Toggle();
  PQC__insert__postTask();
}

# 204 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x2b32aedfc3f0){
#line 83
  switch (arg_0x2b32aedfc3f0) {
#line 83
    case 0U:
#line 83
      PQC__Timer__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x2b32aedfc3f0);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 103 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 94
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 129 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 129
}
#line 129
# 91 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 93
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 93
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 93
        __nesc_atomic_end(__nesc_atomic); 
#line 93
        return __nesc_temp;
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 116 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 116
  unsigned long __nesc_result;
#line 116

#line 116
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 109 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 96 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 97
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 136 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 139 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 83 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 83
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 83
}
#line 83
# 57 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )49U &= ~(0x01 << 5);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr();
#line 53
}
#line 53
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr();
}

# 41 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__clr(void ){
#line 41
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr();
#line 41
}
#line 41
# 89 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1On(void )
#line 89
{
  LedsP__Led1__clr();
  ;
#line 91
  ;
}

# 72 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Leds.nc"
inline static void PQC__Leds__led1On(void ){
#line 72
  LedsP__Leds__led1On();
#line 72
}
#line 72
# 109 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Off(void )
#line 109
{
  LedsP__Led2__set();
  ;
#line 111
  ;
}

# 94 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Leds.nc"
inline static void PQC__Leds__led2Off(void ){
#line 94
  LedsP__Leds__led2Off();
#line 94
}
#line 94
# 57 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )49U &= ~(0x01 << 6);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr();
#line 53
}
#line 53
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr();
}

# 41 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__clr(void ){
#line 41
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr();
#line 41
}
#line 41
# 104 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2On(void )
#line 104
{
  LedsP__Led2__clr();
  ;
#line 106
  ;
}

# 89 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Leds.nc"
inline static void PQC__Leds__led2On(void ){
#line 89
  LedsP__Leds__led2On();
#line 89
}
#line 89
# 94 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Off(void )
#line 94
{
  LedsP__Led1__set();
  ;
#line 96
  ;
}

# 77 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Leds.nc"
inline static void PQC__Leds__led1Off(void ){
#line 77
  LedsP__Leds__led1Off();
#line 77
}
#line 77
# 25 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/FibonacciHeapC.nc"
static __inline void /*PQAppC.FH*/FibonacciHeapC__0__binHeapInit(binomial_heap_t *heap)
#line 25
{
  heap->tree = (void *)0;
  heap->prev = (void *)0;
  heap->next = (void *)0;
}

#line 106
static inline void /*PQAppC.FH*/FibonacciHeapC__0__ristruttura(void )
#line 106
{
  binomial_heap_t *tmp;
  heap_node_t *tree1;
#line 108
  heap_node_t *tree2;
  bool compose = TRUE;

#line 110
  /*PQAppC.FH*/FibonacciHeapC__0__heapPtr = /*PQAppC.FH*/FibonacciHeapC__0__heap;

  while (/*PQAppC.FH*/FibonacciHeapC__0__heapPtr != (void *)0) {
      tmp = /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->next;
      tree1 = /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->tree;

      if (tree1 == (void *)0) {
          /*PQAppC.FH*/FibonacciHeapC__0__heapPtr = /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->next;
          continue;
        }

      if (tree1->rsibling != (void *)0) {
          tmp = malloc(sizeof(binomial_heap_t ));
          /*PQAppC.FH*/FibonacciHeapC__0__binHeapInit(tmp);
          /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->next = tmp;
          tmp->prev = /*PQAppC.FH*/FibonacciHeapC__0__heapPtr;
        }
      else 
#line 126
        {
          /*PQAppC.FH*/FibonacciHeapC__0__heapPtr = /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->next;
          continue;
        }

      while (tree1->rsibling != (void *)0) {
          tree2 = tree1->rsibling;

          if (tree1->key < tree2->key) {
              tree1->rsibling = tree2->child;
              tree2->child = tree1;
              tree1->parent = tree2;

              /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->tree = tree2->rsibling;
              tree2->size += 1;
              tree2->rsibling = tmp->tree;
              tmp->tree = tree2;
            }
          else 
#line 143
            {
              /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->tree = tree2->rsibling;
              tree2->rsibling = tree1->child;
              tree1->child = tree2;
              tree2->parent = tree1;

              tree1->size += 1;
              tree1->rsibling = tmp->tree;
              tmp->tree = tree1;
            }

          tree1 = /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->tree;
        }

      /*PQAppC.FH*/FibonacciHeapC__0__heapPtr = /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->next;
    }
}

# 9 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/SimpleHashC.nc"
static inline uint8_t /*PQAppC.SimpleHashC*/SimpleHashC__0__Hash__getHash(uint8_t key)
#line 9
{
  return key % 10;
}

# 5 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HashFunction.nc"
inline static uint8_t /*PQAppC.HashMapC*/HashMapC__0__Hash__getHash(/*PQAppC.HashMapC*/HashMapC__0__Hash__key_type key){
#line 5
  unsigned char __nesc_result;
#line 5

#line 5
  __nesc_result = /*PQAppC.SimpleHashC*/SimpleHashC__0__Hash__getHash(key);
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 53 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HashMapC.nc"
static inline error_t /*PQAppC.HashMapC*/HashMapC__0__HashMap__insert(/*PQAppC.HashMapC*/HashMapC__0__el_type *element, /*PQAppC.HashMapC*/HashMapC__0__key_type key)
#line 53
{
  uint8_t index = /*PQAppC.HashMapC*/HashMapC__0__Hash__getHash(key);
  list_t *tmp = /*PQAppC.HashMapC*/HashMapC__0__hashmap[index];
  list_t newEl;

  newEl.element = (void *)element;
  newEl.key = (void *)&key;
  newEl.next = tmp;

  /*PQAppC.HashMapC*/HashMapC__0__hashmap[index] = &newEl;

  return SUCCESS;
}

# 4 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HashMap.nc"
inline static error_t /*PQAppC.FH*/FibonacciHeapC__0__HashMap__insert(/*PQAppC.FH*/FibonacciHeapC__0__HashMap__e *element, /*PQAppC.FH*/FibonacciHeapC__0__HashMap__k key){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = /*PQAppC.HashMapC*/HashMapC__0__HashMap__insert(element, key);
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 15 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/FibonacciHeapC.nc"
static __inline void /*PQAppC.FH*/FibonacciHeapC__0__heapNodeInit(heap_node_t *node)
#line 15
{
  node->element = (void *)0;
  node->key = 0;
  node->size = 0;
  node->parent = (void *)0;
  node->lsibling = (void *)0;
  node->rsibling = (void *)0;
  node->child = (void *)0;
}

#line 163
static inline void /*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__insert(/*PQAppC.FH*/FibonacciHeapC__0__elementType *element, uint8_t priority)
#line 163
{

  heap_node_t *b0;

#line 166
  b0 = malloc(sizeof(heap_node_t ));
  /*PQAppC.FH*/FibonacciHeapC__0__heapNodeInit(b0);
  b0->element = element;
  b0->key = priority;
  b0->size = 0;


  b0->rsibling = /*PQAppC.FH*/FibonacciHeapC__0__heap->tree;
  /*PQAppC.FH*/FibonacciHeapC__0__heap->tree = b0;

  if (/*PQAppC.FH*/FibonacciHeapC__0__max == (void *)0 || priority > /*PQAppC.FH*/FibonacciHeapC__0__max->key) {
    /*PQAppC.FH*/FibonacciHeapC__0__max = b0;
    }
  /*PQAppC.FH*/FibonacciHeapC__0__HashMap__insert(b0, (uint8_t )element);



  /*PQAppC.FH*/FibonacciHeapC__0__ristruttura();
}

# 10 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/WPriorityQueue.nc"
inline static void PQC__WPriorityQueue__insert(PQC__WPriorityQueue__t *element, PQC__WPriorityQueue__p priority){
#line 10
  /*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__insert(element, priority);
#line 10
}
#line 10
# 42 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/FibonacciHeapC.nc"
static inline void /*PQAppC.FH*/FibonacciHeapC__0__setMax(void )
#line 42
{
  heap_node_t *tmp;

#line 44
  /*PQAppC.FH*/FibonacciHeapC__0__heapPtr = /*PQAppC.FH*/FibonacciHeapC__0__heap;

  while (/*PQAppC.FH*/FibonacciHeapC__0__heapPtr != (void *)0) {
      tmp = /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->tree;

      while (tmp->rsibling != (void *)0) {
          if (/*PQAppC.FH*/FibonacciHeapC__0__max == (void *)0) {
            /*PQAppC.FH*/FibonacciHeapC__0__max = tmp;
            }
          else 
#line 52
            {
              if (tmp->key > /*PQAppC.FH*/FibonacciHeapC__0__max->key) {
                  /*PQAppC.FH*/FibonacciHeapC__0__max = tmp;
                }
            }
          tmp = tmp->rsibling;
        }

      /*PQAppC.FH*/FibonacciHeapC__0__heapPtr = /*PQAppC.FH*/FibonacciHeapC__0__heapPtr->next;
    }
}

static inline /*PQAppC.FH*/FibonacciHeapC__0__elementType */*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__findMax(void )
#line 64
{
  if (/*PQAppC.FH*/FibonacciHeapC__0__max == (void *)0) {
    /*PQAppC.FH*/FibonacciHeapC__0__setMax();
    }
  return (/*PQAppC.FH*/FibonacciHeapC__0__elementType *)/*PQAppC.FH*/FibonacciHeapC__0__max->element;
}

# 6 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/WPriorityQueue.nc"
inline static PQC__WPriorityQueue__t *PQC__WPriorityQueue__findMax(void ){
#line 6
  struct wids_state_trace *__nesc_result;
#line 6

#line 6
  __nesc_result = /*PQAppC.FH*/FibonacciHeapC__0__WPriorityQueue__findMax();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 69 "/home/loki/tinyos-release-tinyos-2_1_2/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 57 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4333 {
#line 57
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 47 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 216 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x2b32af14fc40){
#line 59
  switch (arg_0x2b32af14fc40) {
#line 59
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 59
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x2b32af14fc40);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 133 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 133
{
  /* atomic removed: atomic calls only */
#line 134
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
        if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
            /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 138
              SUCCESS;

#line 138
              return __nesc_temp;
            }
          }
        else {
#line 140
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 143
                SUCCESS;

#line 143
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 147
  return FAIL;
}

# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 105 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void )
#line 105
{
  return SUCCESS;
}

# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 74 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void )
#line 74
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release();
}

# 81 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested();
#line 81
}
#line 81
# 206 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 61 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x2b32af151840){
#line 61
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x2b32af151840);
#line 61
}
#line 61
# 93 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /* atomic removed: atomic calls only */
#line 95
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 100
        FAIL;

#line 100
        return __nesc_temp;
      }
  }
#line 102
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 107
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
  return FAIL;
}

# 212 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 212
{
#line 212
  return FAIL;
}

# 97 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(uint8_t arg_0x2b32aefca4e0){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  switch (arg_0x2b32aefca4e0) {
#line 97
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 97
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 97
      break;
#line 97
    default:
#line 97
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(arg_0x2b32aefca4e0);
#line 97
      break;
#line 97
    }
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 65 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id)
#line 65
{
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(id);
}

# 97 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP__Resource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 10 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__start(void )
#line 10
{
  return TelosSerialP__Resource__immediateRequest();
}

# 95 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/StdControl.nc"
inline static error_t SerialPrintfP__UartControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = TelosSerialP__StdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/SerialPrintfP.nc"
static inline error_t SerialPrintfP__StdControl__start(void )
{
  return SerialPrintfP__UartControl__start();
}

#line 50
static inline error_t SerialPrintfP__Init__init(void )
#line 50
{
  return SerialPrintfP__StdControl__start();
}

# 55 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 98 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/PutcharP.nc"
static inline error_t PutcharP__Init__init(void )
#line 98
{
  error_t rv = SUCCESS;



  return rv;
}

# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PutcharP__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, SerialPrintfP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 67 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 144 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/Timer.nc"
inline static void PQC__Timer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(0U, dt);
#line 64
}
#line 64
# 45 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/HashMapC.nc"
static inline error_t /*PQAppC.HashMapC*/HashMapC__0__Init__init(void )
#line 45
{
  uint8_t i = 0;

#line 47
  while (i < 10) {
      /*PQAppC.HashMapC*/HashMapC__0__hashmap[i] = (void *)0;
      i += 1;
    }
}

# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
inline static error_t /*PQAppC.FH*/FibonacciHeapC__0__HashMapInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*PQAppC.HashMapC*/HashMapC__0__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 38 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/wids/utility/FibonacciHeapC.nc"
static inline error_t /*PQAppC.FH*/FibonacciHeapC__0__Init__init(void )
#line 38
{
  /*PQAppC.FH*/FibonacciHeapC__0__HashMapInit__init();
}

# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Init.nc"
inline static error_t PQC__Init__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*PQAppC.FH*/FibonacciHeapC__0__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 19 "PQC.nc"
static inline void PQC__Boot__booted(void )
{
  PQC__Init__init();
  PQC__Timer__startPeriodic(3000);
}

# 60 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  PQC__Boot__booted();
#line 60
}
#line 60
# 175 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 75 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/TaskBasic.nc"
inline static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x2b32ae774e60){
#line 75
  switch (arg_0x2b32ae774e60) {
#line 75
    case PQC__insert:
#line 75
      PQC__insert__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 75
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 75
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 75
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x2b32ae774e60);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 391 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

# 63 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 63
{
  return MSP430_POWER_LPM3;
}

# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 74 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 74
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 77
  TACCTL0 & 0x0010 || 
  TACCTL1 & 0x0010) || 
  TACCTL2 & 0x0010) && (
  TACTL & 0x0300) == 0x0200) || (
  ME1 & (0x80 | 0x40) && U0TCTL & 0x20)) || (
  ME2 & (0x20 | 0x10) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 99
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 100
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 379 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 379
{
  return m1 < m2 ? m1 : m2;
}

# 112 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 112
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 117
{
  uint16_t temp;

#line 119
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 83
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 161
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 72 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 98 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 221 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte)
#line 221
{
}

# 79 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(uint8_t arg_0x2b32aefcc020, uint8_t byte){
#line 79
    /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(arg_0x2b32aefcc020, byte);
#line 79
}
#line 79
# 222 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 222
{
}

# 99 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(uint8_t arg_0x2b32aefcc020, uint8_t * buf, uint16_t len, error_t error){
#line 99
    /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(arg_0x2b32aefcc020, buf, len, error);
#line 99
}
#line 99
# 134 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data)
#line 134
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf) {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos++] = data;
      if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos >= /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len) {
          uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;

#line 139
          /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf = (void *)0;
          /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len, SUCCESS);
        }
    }
  else 
#line 142
    {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(id, data);
    }
}

# 65 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x2b32af10a020, uint8_t data){
#line 54
  switch (arg_0x2b32af10a020) {
#line 54
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 54
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x2b32af10a020, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 220 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 220
{
}

# 57 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(uint8_t arg_0x2b32aefcc020, uint8_t * buf, uint16_t len, error_t error){
#line 57
    /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(arg_0x2b32aefcc020, buf, len, error);
#line 57
}
#line 57
# 384 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__tx(uint8_t data)
#line 384
{
  HplMsp430Usart1P__U1TXBUF = data;
}

# 224 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart1P__Usart__tx(data);
#line 224
}
#line 224
# 162 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id)
#line 162
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner != id) {
      uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 165
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, FAIL);
    }
  else {
#line 168
    if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos < /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len) {
        /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos++]);
      }
    else {
        uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 173
        /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
        /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, SUCCESS);
      }
    }
}

# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x2b32af10a020){
#line 49
  switch (arg_0x2b32af10a020) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x2b32af10a020);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 370 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableTxIntr(void )
#line 370
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 371
    {
      HplMsp430Usart1P__IFG2 &= ~0x20;
      HplMsp430Usart1P__IE2 |= 0x20;
    }
#line 374
    __nesc_atomic_end(__nesc_atomic); }
}

# 181 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableTxIntr(void ){
#line 181
  HplMsp430Usart1P__Usart__enableTxIntr();
#line 181
}
#line 181
# 339 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__clrTxIntr(void )
#line 339
{
  HplMsp430Usart1P__IFG2 &= ~0x20;
}

# 202 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__clrTxIntr(void ){
#line 202
  HplMsp430Usart1P__Usart__clrTxIntr();
#line 202
}
#line 202
# 318 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline bool HplMsp430Usart1P__Usart__isTxIntrPending(void )
#line 318
{
  if (HplMsp430Usart1P__IFG2 & 0x20) {
      return TRUE;
    }
  return FALSE;
}

# 187 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__isTxIntrPending(void ){
#line 187
  unsigned char __nesc_result;
#line 187

#line 187
  __nesc_result = HplMsp430Usart1P__Usart__isTxIntrPending();
#line 187

#line 187
  return __nesc_result;
#line 187
}
#line 187
# 355 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableTxIntr(void )
#line 355
{
  HplMsp430Usart1P__IE2 &= ~0x20;
}

# 178 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableTxIntr(void ){
#line 178
  HplMsp430Usart1P__Usart__disableTxIntr();
#line 178
}
#line 178
# 177 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
static inline bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 179
          TRUE;

          {
#line 179
            __nesc_atomic_end(__nesc_atomic); 
#line 179
            return __nesc_temp;
          }
        }
      else 
#line 180
        {
          unsigned char __nesc_temp = 
#line 180
          FALSE;

          {
#line 180
            __nesc_atomic_end(__nesc_atomic); 
#line 180
            return __nesc_temp;
          }
        }
    }
#line 183
    __nesc_atomic_end(__nesc_atomic); }
}

# 210 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id)
#line 210
{
#line 210
  return FALSE;
}

# 128 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/Resource.nc"
inline static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(uint8_t arg_0x2b32aefca4e0){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x2b32aefca4e0) {
#line 128
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(arg_0x2b32aefca4e0);
#line 128
      break;
#line 128
    }
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 178 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartByte__send(uint8_t id, uint8_t data)
#line 178
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 181
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__clrTxIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableTxIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(data);
  while (!/*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__isTxIntrPending()) ;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__clrTxIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableTxIntr();
  return SUCCESS;
}

# 46 "/home/loki/tinyos-release-tinyos-2_1_2/tos/interfaces/UartByte.nc"
inline static error_t SerialPrintfP__UartByte__send(uint8_t byte){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UartByte__send(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, byte);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 69 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/SerialPrintfP.nc"
static inline int SerialPrintfP__Putchar__putchar(int c)
{
  return SUCCESS == SerialPrintfP__UartByte__send(c) ? c : -1;
}

# 49 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/Putchar.nc"
inline static int PutcharP__Putchar__putchar(int c){
#line 49
  int __nesc_result;
#line 49

#line 49
  __nesc_result = SerialPrintfP__Putchar__putchar(c);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 411 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (__read_status_register() & 0x0008) != 0;

#line 414
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000C)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 180 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000A)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(0x001A)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 146 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 39 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x2b32ae8ec8b0){
#line 39
  switch (arg_0x2b32ae8ec8b0) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 39
      break;
#line 39
    case 3:
#line 39
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 39
      break;
#line 39
    case 4:
#line 39
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 39
      break;
#line 39
    case 6:
#line 39
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 39
      break;
#line 39
    case 7:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x2b32ae8ec8b0);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 170 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
#line 172
      {
        unsigned char __nesc_temp = 
#line 172
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 107 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 80 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 87
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 62 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 69
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 72
        do {
#line 72
            t0 = t1;
#line 72
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 72
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 73
          t1;

#line 73
          return __nesc_temp;
        }
      }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )400U;
    }
}

# 14 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0018)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 63 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/RealMainP.nc"
  int main(void )
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 71
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 175 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/home/loki/tinyos-release-tinyos-2_1_2/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 56 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )49U |= 0x01 << 5;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

#line 56
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )49U |= 0x01 << 6;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 134 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 138
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

#line 142
          return __nesc_temp;
        }
      }
  }
#line 145
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

# 190 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 85 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/Msp430UartP.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id)
#line 85
{
  msp430_uart_union_config_t *config = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(id);

#line 87
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time = config->uartConfig.ubr / 2;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(config);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr();
}

# 100 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint16_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 135
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

#line 73
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint16_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 90
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 147 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 74 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/timer/AlarmToTimerC.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 25 "PQC.nc"
static void PQC__insert__runTask(void )
#line 25
{
  wids_state_trace_t *trace = malloc(sizeof(wids_state_trace_t ) + sizeof(wids_state_t ));

#line 27
  if (PQC__s == TRUE) {
      PQC__s = FALSE;
      ((wids_state_t *)trace)->attack = DECEPTIVE_JAMMING;
      PQC__Leds__led1On();
      PQC__Leds__led2Off();
    }
  else {
      PQC__s = TRUE;
      ((wids_state_t *)trace)->attack = DECEPTIVE_JAMMING;
      PQC__Leds__led2On();
      PQC__Leds__led1Off();
    }
  trace->alarm_value = PQC__score;

  PQC__WPriorityQueue__insert(trace, PQC__score);
  PQC__score <<= 1;


  trace = PQC__WPriorityQueue__findMax();

  printf("TRACE ALARM: %d, SCORE %d\r\n", ((wids_state_t *)trace)->attack, trace->alarm_value);
  printfflush();
}

# 64 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/SerialPrintfP.nc"
  int printfflush(void )
#line 64
{
  return SUCCESS;
}

# 96 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0006)))  void sig_UART1RX_VECTOR(void )
#line 96
{
  uint8_t temp = U1RXBUF;

#line 98
  HplMsp430Usart1P__Interrupts__rxDone(temp);
}

# 153 "/home/loki/tinyos-release-tinyos-2_1_2/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}






static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 101 "/home/loki/tinyos-release-tinyos-2_1_2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0004)))  void sig_UART1TX_VECTOR(void )
#line 101
{
  HplMsp430Usart1P__Interrupts__txDone();
}

# 107 "/home/loki/tinyos-release-tinyos-2_1_2/tos/lib/printf/PutcharP.nc"
__attribute((noinline))   int putchar(int c)
#line 107
{
#line 119
  return PutcharP__Putchar__putchar(c);
}

