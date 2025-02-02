#include <stdint.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <utime.h>
#include <unistd.h>

#include "hal.h"
#include "gps.h"
#include "sdlog.h"
#include "timesync.h"
#include "fifo.h"

#include "ctrl.h"
#include "proc.h"
#include "rf.h"

// ============================================================================================

static char  LogFileName[32];
static FILE *LogFile = 0;

static   uint16_t  LogDate = 0;                                   // [~days] date = FatTime>>16
static TickType_t  LogOpenTime;                                   // [msec] when was the log file (re)open
static const TickType_t LogReopen = 30000;                        // [msec] when to close and re-open the log file

const size_t FIFOsize = 16384;
static FIFO<char, FIFOsize> Log_FIFO;                             // 16K buffer for SD-log
       SemaphoreHandle_t Log_Mutex;                               // Mutex for the FIFO to prevent mixing between threads

void Log_Write(char Byte)                                         // write a byte into the log file buffer (FIFO)
{ if(Log_FIFO.Write(Byte)>0) return;                              // if byte written into FIFO return
  while(Log_FIFO.Write(Byte)<=0) vTaskDelay(1); }                 // wait while the FIFO is full - we have to use vTaskDelay not TaskYIELD

int Log_Free(void) { return Log_FIFO.Free(); }                    // how much space left in the buffer

static int Log_Open(void)
{ int32_t Day   =  GPS_DateTime.Day;                                 // get day, month, year
  int32_t Month =  GPS_DateTime.Month;
  int32_t Year  =  GPS_DateTime.Year;
  uint32_t Date = 0;
  if(Year>=20 && Year<70) Date = Year*10000 + Month*100 + Day;    // create YYMMDD number for easy printout and sort
  strcpy(LogFileName, "/sdcard/CONS/TR000000.LOG");
  Format_UnsDec(LogFileName+15, Date, 6);                         // format the date into the log file name
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "Log_Open() ");
  Format_String(CONS_UART_Write, LogFileName);
  Format_String(CONS_UART_Write, " Year:");
  Format_SignDec(CONS_UART_Write, Year);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  LogFile = fopen(LogFileName, "at");                             // try to open the file
  if(LogFile==0)                                                  // if this fails
  { if(mkdir("/sdcard/CONS", 0777)<0) return -1;                  // try to create the sub-directory
    LogFile = fopen(LogFileName, "at"); if(LogFile==0) return -1; } // and again attempt to open the log file
  LogOpenTime=xTaskGetTickCount();
  return 0; }

                                                                  // TaskYIELD would not give time to lower priority task like log-writer
static void Log_Check(void)                                       // time check:
{ if(!LogFile) return;                                            // if last operation in error then don't do anything
  TickType_t TimeSinceOpen = xTaskGetTickCount()-LogOpenTime;     // when did we (re)open the log file last time
  if(LogDate)
  { if(TimeSinceOpen<LogReopen) return; }                         // if fresh (less than 30 seconds) then nothing to do
  else
  { if(TimeSinceOpen<(LogReopen/4)) return; }
  fclose(LogFile); LogFile=0;                                     // close and reopen the log file when older than 10 seconds
  uint32_t Time = TimeSync_Time();
  struct stat LogStat;
  struct utimbuf LogTime;
  if(stat(LogFileName, &LogStat)>=0)                              // get file attributes (maybe not really needed)
  { LogTime.actime  = Time;                                       // set access and modification times of the dest. file
    LogTime.modtime = Time;
    utime(LogFileName, &LogTime); }
}

static int WriteLog(size_t MaxBlock=FIFOsize/2)                    // process the queue of lines to be written to the log
{ if(!LogFile) return 0;
  int Count=0;
  for( ; ; )
  { char *Block; size_t Len=Log_FIFO.getReadBlock(Block); if(Len==0) break;
    if(Len>MaxBlock) Len/=2;
    int Write=fwrite(Block, 1, Len, LogFile);
    Log_FIFO.flushReadBlock(Len);
    if(Write!=Len) { fclose(LogFile); LogFile=0; return -1; }
    Count+=Write; }
  return Count; }

// ============================================================================================

#ifdef WITH_SDLOG

const char   *IGC_Path = "/sdcard/IGC";                           //
const int     IGC_PathLen = 11;
// constexpr int IGC_PathLen = strlen(IGC_Path);
const uint32_t IGC_SavePeriod = 60;                               // [sec]
      char    IGC_Serial[4] = { 0, 0, 0, 0 };
      char    IGC_FileName[32];
static FILE  *IGC_File=0;                                         //
static uint32_t IGC_SaveTime=0;                                   // [sec]
       uint16_t IGC_FlightNum=0;                                  // flight counter
static uint32_t IGC_AcftID=0;                                     // to keep trackof possibly changing aircraft radio ID

static SHA256 IGC_SHA256, IGC_SHA256_bck;                         //
const int IGC_Digest_Size = 32;
static uint8_t IGC_Digest[IGC_Digest_Size];                       //

FIFO<OGN_RxPacket<OGN_Packet>, 32> IGClog_FIFO;

static void IGC_TimeStamp(void)
{ struct stat FileStat;
  struct utimbuf FileTime;
  if(stat(IGC_FileName, &FileStat)>=0)                            // get file attributes (maybe not needed really ?
  { FileTime.actime  = IGC_SaveTime;                              // set access and modification time
    FileTime.modtime = IGC_SaveTime;
    utime(IGC_FileName, &FileTime); }                             // write to the FAT
}

static void IGC_Close(void)
{ if(IGC_File)                                                      // if a file open, then close it and increment the flight number
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "IGC_Close: ");
    Format_String(CONS_UART_Write, IGC_FileName);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
    fclose(IGC_File); IGC_File=0; IGC_FlightNum++; }
}
/*
    IGC_SaveTime = TimeSync_Time();
    struct stat FileStat;
    struct utimbuf FileTime;
    if(stat(IGC_FileName, &FileStat)>=0)                            // get file attributes (maybe not needed really ?
    { FileTime.actime  = IGC_SaveTime;                                      // set access and modification tim$
      FileTime.modtime = IGC_SaveTime;
      utime(IGC_FileName, &FileTime); }                             // write to the FAT
  }
}
*/

static int IGC_Open(const GPS_Position &GPS)
{ IGC_Close();                                                      // close the previous file, if open
  if(!SD_isMounted()) return -1;                                    // -1 => SD not mounted
  memcpy(IGC_FileName, IGC_Path, IGC_PathLen);                      // copy path
  IGC_FileName[IGC_PathLen]='/';                                    // slash after the path
  Flight.ShortName(IGC_FileName+IGC_PathLen+1, GPS, IGC_FlightNum, IGC_Serial); // full name
  IGC_File=fopen(IGC_FileName, "rt");                               // attempt to open for read, just to try if the file is already there
  if(IGC_File) { IGC_Close(); return -2; }                          // -2 => file already exists
  IGC_File=fopen(IGC_FileName, "wt");                               // open for write
  if(IGC_File==0)                                                   // failed: maybe sub-dir does not exist ?
  { if(mkdir(IGC_Path, 0777)<0) return -3;                          // -3 => can't create sub-dir
    IGC_File=fopen(IGC_FileName, "wt"); }                           // retry to open for write
  if(IGC_File)
  { IGC_SaveTime = TimeSync_Time();
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "IGC_Open: ");
    Format_String(CONS_UART_Write, IGC_FileName);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex); }
  return IGC_File ? 0:-4; }                                         // -4 => can't open for write

static void IGC_Reopen(void)
{ if(IGC_File)
  { fclose(IGC_File);
    IGC_SaveTime = TimeSync_Time();
    IGC_TimeStamp();
    IGC_File=fopen(IGC_FileName, "at"); }
}

static int IGC_LogLine(const char *Line, int Len)
{ int Written = fwrite(Line, 1, Len, IGC_File);
  IGC_SHA256.Update((uint8_t *)Line, Written);
  return Written; }

static int IGC_LogLine(const char *Line)
{ return IGC_LogLine(Line, strlen(Line)); }

static char Line[512];

static void IGC_LogHeadParm(const char *Name, const char *Parm1, const char *Parm2=0, const char *Parm3=0)
{ int Len=Format_String(Line, Name);
  Len+=Format_String(Line+Len, Parm1);
  if(Parm2 && Parm2[0]) { Line[Len++]=','; Len+=Format_String(Line+Len, Parm2); }
  if(Parm3 && Parm3[0]) { Line[Len++]=','; Len+=Format_String(Line+Len, Parm3); }
  Line[Len++]='\n'; Line[Len]=0;
  IGC_LogLine(Line, Len); }

static void IGC_LogHeader(const GPS_Position &Pos)                      // write the top of the IGC file
{ IGC_LogLine("AGNE001Tracker\n");
  IGC_LogLine("HFFXA020\n");
  { int Len=Format_String(Line, "HFDTEDate:");                      // date
    Len+=Format_UnsDec(Line+Len, (uint16_t)Pos.Day  , 2);           // from the GPS position
    Len+=Format_UnsDec(Line+Len, (uint16_t)Pos.Month, 2);
    Len+=Format_UnsDec(Line+Len, (uint16_t)Pos.Year , 2);
    Line[Len++]=',';
    Len+=Format_UnsDec(Line+Len, (uint16_t)IGC_FlightNum, 2);       // flight number of the day
    Line[Len++]='\n'; Line[Len]=0;
    IGC_LogLine(Line, Len); }
  IGC_LogHeadParm("HFPLTPilotincharge:", Parameters.Pilot);            // Pilot
  IGC_LogHeadParm("HFGTYGliderType:",    Parameters.Type, Parameters.Manuf, Parameters.Model);             // aircraft type like ASK-21
  IGC_LogHeadParm("HFGIDGliderID:",      Parameters.Reg);              // aircraft registration like D-8329
  IGC_LogHeadParm("HFCM2Crew2:",         Parameters.Crew);             // Crew member
  IGC_LogHeadParm("HFCCLCompetitionClass:", Parameters.Class);         // competition class
  IGC_LogHeadParm("HFCIDCompetitionID:", Parameters.ID);               // competition ID
  {
#ifdef WITH_FollowMe
    int Len=Format_String(Line, "HFRHWHardwareVersion:FollowMe");
#else
    int Len=Format_String(Line, "HFRHWHardwareVersion:ESP32+LoRa"); // hardware version
#endif
    Line[Len++]='\n'; Line[Len]=0;
    IGC_LogLine(Line, Len); }
  IGC_LogLine("HFRFWFirmwareVersion:ESP32-OGN-TRACKER " __DATE__ " " __TIME__ "\n"); // firmware version, compile date/time
#ifdef WITH_FollowMe
  IGC_LogLine("HFGPSReceiver:L80\n");                              // GPS sensor
#else
#ifdef WITH_GPS_UBX
  IGC_LogLine("HFGPSReceiver:UBX\n");                              // GPS sensor
#endif
#ifdef WITH_GPS_MTK
  IGC_LogLine("HFGPSReceiver:MTK\n");                              // GPS sensor
#endif
#endif
#ifdef WITH_BMP180
  IGC_LogLine("HFPRSPressAltSensor:BMP180\n");                     // pressure sensor
#endif
#ifdef WITH_BMP280
  IGC_LogLine("HFPRSPressAltSensor:BMP280\n");                     // pressure sensor
#endif
#ifdef WITH_BME280
  IGC_LogLine("HFPRSPressAltSensor:BME280/BMP280\n");              // pressure sensor
#endif
}

void IGC_ID(void)
{ uint32_t Time = TimeSync_Time();
  int Len=Format_String(Line, "LOGN");
  Len+=Format_HHMMSS(Line+Len, Time);
  Len+=Format_String(Line+Len, "ID ");
  Len+=Format_Hex(Line+Len, Parameters.AcftID);                    // Radio ID
  Line[Len++]='\n'; Line[Len]=0;
  IGC_LogLine(Line, Len); }

void IGC_MAC(void)
{ uint64_t MAC = getUniqueID();
  int Len=4+6;
  Len+=Format_String(Line+Len, "MAC ");
  Len+=Format_Hex(Line+Len, (uint16_t)(MAC>>32));                  // ESP32 48-bit ID
  Len+=Format_Hex(Line+Len, (uint32_t) MAC     );
  Line[Len++]='\n'; Line[Len]=0;
  IGC_LogLine(Line, Len); }

static int IGC_Log(const GPS_Position &Pos)                         // log GPS position as B-record
{ int Len=Pos.WriteIGC(Line);                                       // write GPS position in the IGC format
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);
#endif
  int Written=IGC_LogLine(Line, Len);                               // write Line to the IGC log
  if(Written!=Len) IGC_Close();                                     // if not all data written then close the log
  return Written; }                                                 // return number of bytes written or (negative) error

static int IGC_FormatLOGN(char *Out, const char *Type, const GPS_Position &GPS)
{ int Len=Format_String(Out, "LOGN");
  if(GPS.isTimeValid()) Len+=GPS.WriteHHMMSS(Out+Len);
                   else Len+=Format_String(Out+Len, "      ");
  Len+=Format_String(Out+Len, Type); return Len; }

static void IGC_LogSig(const uint8_t *Dig, int DigLen, const uint8_t *Sig, int SigLen, bool Part, const GPS_Position &GPS) // write SHA and Signature to the IGC log
{ int Len=0;
  if(Part) Len+=IGC_FormatLOGN(Line+Len, "DIG ", GPS);
  Line[Len++]='G';                                               // produce G-record with SH256
  for(int Idx=0; Idx<DigLen; Idx++)                              // 32 SHA256 bytes
    Len+=Format_Hex(Line+Len, Dig[Idx]);                         // printed as hex
  Line[Len++]='\n'; Line[Len]=0;                                 // end-of-line, end-of-string
  IGC_LogLine(Line, Len);                                        // write the SHA256
  Len=0;                                                         // 2nd G-record with the signature
  if(Part) Len+=IGC_FormatLOGN(Line+Len, "SIG ", GPS);
  Line[Len++]='G';                                               // produce G-record with SH256
  for(int Idx=0; Idx<SigLen; Idx++)                              // signature bytes
    Len+=Format_Hex(Line+Len, Sig[Idx]);                         // printed as hex
  Line[Len++]='\n'; Line[Len]=0;                                 // end-of-line, end-of-string
  IGC_LogLine(Line, Len); }                                      // write the signature

static void IGC_LogBATstatus(const GPS_Position &GPS)
{ int Len=IGC_FormatLOGN(Line, "BAT ", GPS);
  // int Len=Format_String(Line, "LOGNBAT");
  // if(GPS.isTimeValid()) Len+=GPS.WriteHHMMSS(Line+Len);
  // Line[Len++]=' ';
  Len+=Format_UnsDec(Line+Len, BatteryVoltage>>8, 4, 3);        // print the battery voltage readout
  Len+=Format_String(Line+Len, "V ");
  Len+=Format_SignDec(Line+Len, (600*BatteryVoltageRate+128)>>8, 3, 1);
  Len+=Format_String(Line+Len, "mV/min");
  Line[Len++]='\n'; Line[Len]=0;                                 // end-of-line, end-of-string
  IGC_LogLine(Line, Len); }

static void IGC_LogRFMstatus(const GPS_Position &GPS)
{ int Len=IGC_FormatLOGN(Line, "RFM Tx:", GPS);
  // int Len=Format_String(Line, "LOGNRFM");
  // if(GPS.isTimeValid()) Len+=GPS.WriteHHMMSS(Line+Len);
  // Len+=Format_String(Line+Len, "Tx:");                                     //
  Len+=Format_SignDec(Line+Len, (int16_t)Parameters.TxPower);              // Tx power
  Len+=Format_String(Line+Len, "dBm ");
  Len+=Format_SignDec(Line+Len, (int16_t)TRX.chipTemp);                    // RF chip internal temperature (not calibrated)
  Len+=Format_String(Line+Len, "degC Rx:");                                     //
  Len+=Format_SignDec(Line+Len, -5*TRX.averRSSI, 2, 1);                    // noise level seen by the receiver
  Len+=Format_String(Line+Len, "dBm ");
  Len+=Format_UnsDec(Line+Len, RX_OGN_Count64);                            // received packet/min
  Len+=Format_String(Line+Len, "/min RxFIFO:");
  Len+=Format_UnsDec(Line+Len, RF_RxFIFO.Full());                          // how many packets wait in the RX queue
  Len+=Format_String(Line+Len, " Plan:");
  Len+=Format_String(Line+Len, RF_FreqPlan.getPlanName());                 // name of the frequency plan
  Len+=Format_String(Line+Len, " ");
  Len+=Format_UnsDec(Line+Len, (uint16_t)(RF_FreqPlan.getCenterFreq()/100000), 3, 1); // center frequency
  Len+=Format_String(Line+Len, "MHz");
  Line[Len++]='\n'; Line[Len]=0;                                 // end-of-line, end-of-string
  IGC_LogLine(Line, Len); }

static void IGC_LogRX(const GPS_Position &GPS)
{ uint32_t Time=0; if(GPS.isTimeValid()) Time=GPS.getUnixTime();
  while(IGClog_FIFO.Full())
  { int Len=IGC_FormatLOGN(Line, "RX ", GPS);
    // int Len=Format_String(Line, "LOGNRX");
    // if(GPS.isTimeValid()) Len+=GPS.WriteHHMMSS(Line+Len);
    // Line[Len++]=' ';
    OGN_RxPacket<OGN_Packet> *RxPacket=IGClog_FIFO.getRead();
    uint8_t RxErr = RxPacket->RxErr;
    if(RxErr<=8)
    { Len+=RxPacket->Packet.WriteAPRS(Line+Len, Time, "OGNTRK");
      if(RxErr) { Line[Len++]=' '; Line[Len++]='0'+RxErr; Line[Len++]='e'; }
      Line[Len++]=' ';
      Len+=Format_SignDec(Line+Len, -5*(int16_t)RxPacket->RxRSSI, 1, 1);
      Len+=Format_String(Line+Len, "dBm");
      Line[Len++]='\n'; Line[Len]=0;                                 // end-of-line, end-of-string
      IGC_LogLine(Line, Len);
    }
    IGClog_FIFO.Read();
  }
}

static void IGC_LogGPSstatus(const GPS_Position &GPS)
{ int Len=IGC_FormatLOGN(Line, "GPS ", GPS);
  // int Len=Format_String(Line, "LOGNGPS");
  // if(GPS.isTimeValid()) Len+=GPS.WriteHHMMSS(Line+Len);
  // Line[Len++]=' ';
  if(GPS.isValid())
    Len+=Format_UnsDec(Line+Len, GPS.Satellites);
  else
    Len+=Format_UnsDec(Line+Len, GPS_SatCnt);
  Len+=Format_String(Line+Len, "sat/");
  Line[Len++]='0'+GPS.FixQuality;
  Line[Len++]='/';
  Len+=Format_UnsDec(Line+Len, (GPS_SatSNR+2)>>2);
  Len+=Format_String(Line+Len, "dB");
  if(GPS.isValid())
  { Len+=Format_String(Line+Len, " DOP:"); ;
    Len+=Format_UnsDec(Line+Len, GPS.PDOP, 1, 1);
    Line[Len++]='/';
    Len+=Format_UnsDec(Line+Len, GPS.HDOP, 1, 1);
    Line[Len++]='/';
    Len+=Format_UnsDec(Line+Len, GPS.VDOP, 1, 1); }
  Line[Len++]='\n'; Line[Len]=0;                                 // end-of-line, end-of-string
  IGC_LogLine(Line, Len); }

static void IGC_CheckGPS(void)                                   // check if new GPS position
{ static uint8_t PrevPosIdx=0;
  if(GPS_PosIdx==PrevPosIdx) return;
  PrevPosIdx=GPS_PosIdx;
  const  uint8_t PosPipeIdxMask = GPS_PosPipeSize-1;                 // get the GPS position just before in the pipe
  uint8_t PosIdx = GPS_PosIdx-1; PosIdx&=PosPipeIdxMask;
  static bool PrevInFlight=0;
  const GPS_Position &GPS = GPS_Pos[PosIdx];
  bool inFlight = GPS.InFlight;                                      // in-flight or on-the-ground ?
  bool StopFile = PrevInFlight && !inFlight;
  PrevInFlight = inFlight;;
#ifdef DEBUG_PRINT
  GPS.PrintLine(Line);
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "IGC_Check() [");
  CONS_UART_Write('0'+GPS_PosIdx);
  CONS_UART_Write(inFlight?'^':'_');
  Format_String(CONS_UART_Write, "] ");
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);
#endif
  if(IGC_File)                                                       // if IGC file already open
  { if(Parameters.AcftID!=IGC_AcftID) { IGC_ID(); IGC_AcftID=Parameters.AcftID; }
    IGC_Log(GPS);                                                    // log position
    IGC_LogRX(GPS);
    if(StopFile)                                                     // if no longer in flight
    { IGC_SHA256.Finish(IGC_Digest);                                 // complete SHA256 digest
      uint8_t *Sig = (uint8_t *)Line+256;                            // space to write the SHA and signature
      int SigLen = IGC_SignKey.Sign_MD5_SHA256(Sig, IGC_Digest, IGC_Digest_Size); // produce signature
      IGC_LogSig(IGC_Digest, IGC_Digest_Size, Sig, SigLen, 0, GPS);
      IGC_Close(); IGC_TimeStamp(); }                                // then close the IGC file
    else                                                             // if (still) in flight
    { uint32_t Time=TimeSync_Time();                                 //
      if(Time-IGC_SaveTime>=IGC_SavePeriod)                          //
      { IGC_LogBATstatus(GPS);
        IGC_LogGPSstatus(GPS);
        IGC_LogRFMstatus(GPS);
        IGC_SHA256_bck.Clone(IGC_SHA256);
        IGC_SHA256_bck.Finish(IGC_Digest);                           // complete SHA256 digest
        uint8_t *Sig = (uint8_t *)Line+256;                          // space to write the SHA and signature
        int SigLen = IGC_SignKey.Sign_MD5_SHA256(Sig, IGC_Digest, IGC_Digest_Size); // produce signature
        IGC_LogSig(IGC_Digest, IGC_Digest_Size, Sig, SigLen, 1, GPS);
        IGC_Reopen(); }                                              // re-open IGC thus close it and open it back to save the current data
    }
  }
  else if(GPS.isDateValid())                                         // if IGC file is not open
  { for(int Try=0; Try<8; Try++)
    { int Err=IGC_Open(GPS); if(Err!=(-2)) break; }                  // try to open a new IGC file but don't overwrite the old ones
    if(IGC_File)                                                     // if open succesfully
    { IGC_SHA256.Start();                                            // start the SHA256 calculation
      IGC_LogHeader(GPS);                                            // then write header
      IGC_ID(); IGC_AcftID=Parameters.AcftID;
      IGC_MAC();
      IGC_Log(GPS); }                                                // log first B-record
  }
}

#endif // WITH_SDLOG

// ============================================================================================

// IGC_Key IGC_SignKey;

#ifdef WITH_SDLOG

/*
// Uncomment to force use of a specific curve
#define ECPARAMS    MBEDTLS_ECP_DP_SECP192R1

#if !defined(ECPARAMS)
#define ECPARAMS    mbedtls_ecp_curve_list()->grp_id
#endif

static mbedtls_ecdsa_context SignCtx;
static mbedtls_pk_context Key;
static mbedtls_x509write_csr Req;
static mbedtls_entropy_context Entropy;
static mbedtls_ctr_drbg_context CtrDrbgCtx;

static int IGC_GenKey(void)
{ const char *Pers = "ecdsa";
  mbedtls_x509write_csr_init(&Req);
  mbedtls_pk_init(&Key);
  mbedtls_ecdsa_init(&SignCtx);
  mbedtls_ctr_drbg_init(&CtrDrbgCtx);
  mbedtls_entropy_init(&Entropy);
  int Ret = mbedtls_ctr_drbg_seed( &CtrDrbgCtx, mbedtls_entropy_func, &Entropy,
                               (const unsigned char *)Pers, strlen(Pers) );
  if(Ret!=0) return Ret;
  Ret = mbedtls_ecdsa_genkey(&SignCtx, ECPARAMS, mbedtls_ctr_drbg_random, &CtrDrbgCtx);
  return Ret; }
*/
#endif // WITH_SDLOG

// ============================================================================================

#ifdef WITH_SDLOG

extern "C"
 void vTaskSDLOG(void* pvParameters)
{

/*
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "vTaskSDLOG() Start generating key pair\n");
  xSemaphoreGive(CONS_Mutex);
  int Ret=IGC_GenKey();
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "vTaskSDLOG() End generating key pair => ");
  Format_SignDec(CONS_UART_Write, Ret);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
*/
  uint32_t ID = getUniqueAddress();                                         // ID of the tracker (from CPU serial)
  IGC_Serial[2] = Flight.Code36(ID%36); ID/=36;                             // produce 3-char IGC serial for this tracker
  IGC_Serial[1] = Flight.Code36(ID%36); ID/=36;
  IGC_Serial[0] = Flight.Code36(ID%36);

  // IGC_SignKey.Init();
  // IGC_SignKey.Generate();
  // if(IGC_SignKey.ReadFromNVS()!=ESP_OK) IGC_SignKey.WriteToNVS();
  // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  // if(IGC_SignKey.Pub_Write((uint8_t *)Line, 512)==0)
  //   Format_String(CONS_UART_Write, Line);
  // xSemaphoreGive(CONS_Mutex);

  IGC_SHA256.Init();
  IGC_SHA256_bck.Init();

  Log_FIFO.Clear();

  for( ; ; )
  { if(!SD_isMounted())                                              // if SD ia not mounted:
    { vTaskDelay(5000); SD_Mount(); IGC_CheckGPS(); continue; }      // try to (Re)mount it after a delay of 5sec

    // if(GPS_Event)
    // { EventBits_t GPSevt = xEventGroupWaitBits(GPS_Event, GPSevt_NewPos, pdTRUE, pdFALSE, 100);
    //   if(GPSevt&GPSevt_NewPos) LogIGC(); }

    if(!LogFile)                                                     // when SD mounted and log file not open:
    { Log_Open();                                                    // try to (re)open it
      if(!LogFile) { IGC_CheckGPS(); SD_Unmount(); vTaskDelay(1000); continue; }  // if can't be open then unmount the SD and retry at a delay of 1sec
    }

    if(Log_FIFO.Full()<FIFOsize/4) { IGC_CheckGPS(); vTaskDelay(50); }  // if little data to copy, then wait 0.1sec for more data
    int Write;
    do { Write=WriteLog(); } while(Write>0);                         // write the console output to the log file
    if(Write<0) { SD_Unmount(); vTaskDelay(1000); continue; }        // if write fails then unmount the SD card and (re)try after a delay of 1sec
    // if(Write==0) vTaskDelay(100);
    IGC_CheckGPS();
    Log_Check(); }                                                   // make sure the log is well saved by regular close-reopen
}

#endif // WITH_SDLOG
