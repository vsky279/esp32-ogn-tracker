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

// ============================================================================================

static char  LogFileName[32];
static FILE *LogFile = 0;

static   uint16_t  LogDate = 0;                                   // [~days] date = FatTime>>16
static TickType_t  LogOpenTime;                                   // [msec] when was the log file (re)open
static const  TickType_t  LogReopen = 20000;                      // [msec] when to close and re-open the log file

const size_t FIFOsize = 16384;
static FIFO<char, FIFOsize> Log_FIFO;                             // 16K buffer for SD-log
       SemaphoreHandle_t Log_Mutex;                               // Mutex for the FIFO to prevent mixing between threads

void Log_Write(char Byte)                                         // write a byte into the log file buffer (FIFO)
{ if(Log_FIFO.Write(Byte)>0) return;                              // if byte written into FIFO return
  while(Log_FIFO.Write(Byte)<=0) vTaskDelay(1); }                 // wait while the FIFO is full - we have to use vTaskDelay not TaskYIELD

int Log_Free(void) { return Log_FIFO.Free(); }                    // how much space left in the buffer

static int Log_Open(void)
{ LogDate=GPS_FatTime>>16;                                        // get the FAT-time date part
  int32_t Day   =  LogDate    &0x1F;                              // get day, month, year
  int32_t Month = (LogDate>>5)&0x0F;
  int32_t Year  = (LogDate>>9)-20;
  uint32_t Date = 0;
  if(Year>=0) Date = Day*10000 + Month*100 + Year;                // create DDMMYY number for easy printout
  strcpy(LogFileName, "/sdcard/CONS/TR000000.LOG");
  Format_UnsDec(LogFileName+15, Date, 6);                         // format the date into the log file name
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

const char   *IGC_Path = "/sdcard/IGC";
const int     IGC_PathLen = 11;
const uint32_t IGC_SavePeriod = 20;
// constexpr int IGC_PathLen = strlen(IGC_Path);
      char    IGC_Serial[4] = { 0, 0, 0, 0 };
      char    IGC_FileName[32];
static FILE  *IGC_File=0;
static uint32_t IGC_SaveTime=0;
       uint16_t IGC_FlightNum=0;

static void IGC_TimeStamp(void)
{ struct stat FileStat;
  struct utimbuf FileTime;
  if(stat(IGC_FileName, &FileStat)>=0)                            // get file attributes (maybe not needed really ?
  { FileTime.actime  = IGC_SaveTime;                              // set access and modification tim$
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

static int IGC_Open(void)
{ IGC_Close();                                                      // close the previous file, if open
  if(!SD_isMounted()) return -1;                                    // -1 => SD not mounted
  memcpy(IGC_FileName, IGC_Path, IGC_PathLen);                      // copy path
  IGC_FileName[IGC_PathLen]='/';                                    // slash after the path
  Flight.ShortName(IGC_FileName+IGC_PathLen+1, IGC_FlightNum, IGC_Serial); // full name
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

static char Line[192];

static int IGC_HeadParm(const char *Name, const char *Parm)
{ int Len=Format_String(Line, Name);
  Len+=Format_String(Line+Len, Parm);
  Line[Len++]='\n'; Line[Len]=0;
  fputs(Line, IGC_File);
  return 0; }

static int IGC_Header(const GPS_Position &Pos)                      // write the top of the IGC file
{ fputs("AGNE001Tracker\n", IGC_File);
  { int Len=Format_String(Line, "HFDTEDate:");                      // date
    Len+=Format_UnsDec(Line+Len, (uint16_t)Pos.Day  , 2);
    Len+=Format_UnsDec(Line+Len, (uint16_t)Pos.Month, 2);
    Len+=Format_UnsDec(Line+Len, (uint16_t)Pos.Year , 2);
    Line[Len++]=',';
    Len+=Format_UnsDec(Line+Len, (uint16_t)IGC_FlightNum, 2);
    Line[Len++]='\n'; Line[Len]=0;
    fputs(Line, IGC_File); }
  IGC_HeadParm("HFPLTPilotincharge:", Parameters.Pilot);
  IGC_HeadParm("HFGTYGliderType:",    Parameters.Type);
  IGC_HeadParm("HFGIDGliderID:",      Parameters.Reg);
  IGC_HeadParm("HFCM2Crew2:",         Parameters.Crew);
  IGC_HeadParm("HFCCLCompetitionClass:", Parameters.Class);
  IGC_HeadParm("HFCIDCompetitionID:", Parameters.ID);
  { int Len=Format_String(Line, "HFGIDGliderid:");                  // unique ID
    uint64_t ID = getUniqueID();                                    // 48-bit ID
    Len+=Format_Hex(Line+Len, (uint16_t)(ID>>32));
    Len+=Format_Hex(Line+Len, (uint32_t)ID);
    Line[Len++]=',';
    Len+=Format_Hex(Line+Len, Parameters.AcftID);                   // stealth, no-track, aircraft-type, address-type, address
    Line[Len++]='\n'; Line[Len]=0;
    fputs(Line, IGC_File); }
  fputs("HFRFWFirmwareVersion:" __DATE__ " " __TIME__ "\n", IGC_File);
  fputs("HFGPSReceiver:L80\n", IGC_File);                           // GPS sensor
  fputs("HFPRSPressAltSensor:BMP280\n", IGC_File);                  // pressure sensor
  return 0; }

static int IGC_Log(const GPS_Position &Pos)                         // log GPS position as B-record
{ int Len=Pos.WriteIGC(Line);
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);
#endif
  int Written=fwrite(Line, 1, Len, IGC_File);
  if(Written!=Len) IGC_Close();
  return Written; }

static void IGC_Check(void)                                          // check if new GPS position
{ static uint8_t PrevPosIdx=0;
  if(GPS_PosIdx==PrevPosIdx) return;
  PrevPosIdx=GPS_PosIdx;
  const  uint8_t PosPipeIdxMask = GPS_PosPipeSize-1;                 // get the GPS position just before in the pipe
  uint8_t PosIdx = GPS_PosIdx-1; PosIdx&=PosPipeIdxMask;
  bool inFlight = Flight.inFlight();                                 // in-flight or on-the-ground ?
#ifdef DEBUG_PRINT
  GPS_Pos[PosIdx].PrintLine(Line);
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "IGC_Check() [");
  CONS_UART_Write('0'+GPS_PosIdx);
  CONS_UART_Write(inFlight?'^':'_');
  Format_String(CONS_UART_Write, "] ");
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);
#endif
  if(IGC_File)                                                       // if IGC file already open
  { IGC_Log(GPS_Pos[PosIdx]);                                        // log position
    if(!inFlight)                                                    // if no longer in flight
    { IGC_Close(); IGC_TimeStamp(); }                                                 // then close the IGC file
    else
    { uint32_t Time=TimeSync_Time();
      if(Time-IGC_SaveTime>=IGC_SavePeriod)
      { IGC_Reopen(); }
    }
  }
  else                                                               // if IGC file is not open
  { if(inFlight)                                                     // and in-flight
    { for(int Try=0; Try<8; Try++)
      { int Err=IGC_Open(); if(Err!=(-2)) break; }                   // try to open a new IGC file but don't overwrite the old ones
      if(IGC_File) { IGC_Header(GPS_Pos[PosIdx]); IGC_Log(GPS_Pos[PosIdx]); }       // if open succesfully then write header and first B-record
    }
  }
}

// ============================================================================================

#ifdef WITH_SDLOG

extern "C"
 void vTaskSDLOG(void* pvParameters)
{ uint32_t ID = getUniqueAddress();
  IGC_Serial[2] = Flight.Code36(ID%36); ID/=36;
  IGC_Serial[1] = Flight.Code36(ID%36); ID/=36;
  IGC_Serial[0] = Flight.Code36(ID%36);

  Log_FIFO.Clear();

  for( ; ; )
  { if(!SD_isMounted())                                              // if SD ia not mounted:
    { vTaskDelay(5000); SD_Mount(); IGC_Check(); continue; }         // try to (Re)mount it after a delay of 5sec

    // if(GPS_Event)
    // { EventBits_t GPSevt = xEventGroupWaitBits(GPS_Event, GPSevt_NewPos, pdTRUE, pdFALSE, 100);
    //   if(GPSevt&GPSevt_NewPos) LogIGC(); }

    if(!LogFile)                                                     // when SD mounted and log file not open:
    { Log_Open();                                                    // try to (re)open it
      if(!LogFile) { IGC_Check(); SD_Unmount(); vTaskDelay(1000); continue; }  // if can't be open then unmount the SD and retry at a delay of 1sec
    }

    if(Log_FIFO.Full()<FIFOsize/4) { IGC_Check(); vTaskDelay(50); }  // if little data to copy, then wait 0.1sec for more data
    int Write;
    do { Write=WriteLog(); } while(Write>0);                         // write the console output to the log file
    if(Write<0) { SD_Unmount(); vTaskDelay(1000); continue; }        // if write fails then unmount the SD card and (re)try after a delay of 1sec
    // if(Write==0) vTaskDelay(100);
    IGC_Check();
    Log_Check(); }                                                   // make sure the log is well saved by regular close-reopen
}

/*
extern "C"
 void vTaskIGC(void* pvParameters)
{
  for( ; ; )
  { EventBits_t GPSevt = xEventGroupWaitBits(GPS_Event, GPSevt_NewPos, pdTRUE, pdFALSE, 2000);
    if((GPSevt&GPSevt_NewPos)==0) continue;

  }
}
*/

#endif // WITH_SDLOG
