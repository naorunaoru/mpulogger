/**
   This program logs data to a binary file.  Functions are included
   to convert the binary file to a csv text file.

   Samples are logged at regular intervals.  The maximum logging rate
   depends on the quality of your SD card and the time required to
   read sensor data.  This example has been tested at 500 Hz with
   good SD card on an Uno.  4000 HZ is possible on a Due.

   If your SD card has a long write latency, it may be necessary to use
   slower sample rates.  Using a Mega Arduino helps overcome latency
   problems since 13 512 byte buffers will be used.

   Data is written to the file using a SD multiple block write command.
*/
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
//------------------------------------------------------------------------------
// User data functions.  Modify these functions for your data items.
#include "UserDataType.h" // Edit this include file to change data_t.
MPU6050 mpu;

// Acquire a data record.
void acquireData(data_t *data)
{
  data->time = micros();
  mpu.getRotation(&data->gx, &data->gy, &data->gz);
}

// setup AVR I2C
void setupData()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  // set I2C 400 kHz
  TWBR = (F_CPU / 400000 - 16) / 2;
  Serial.println(F("Using Wire"));
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  Serial.println(F("Using Fastwire"));
#endif
}

//==============================================================================
// Start of configuration constants.
//==============================================================================
//Interval between data records in microseconds.
const uint32_t LOG_INTERVAL_USEC = 2000;
//------------------------------------------------------------------------------
// Pin definitions.
//
// SD chip select pin.
const uint8_t SD_CS_PIN = 10;
//
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors and logging continues.
const int8_t ERROR_LED_PIN = 5;

// Start/stop button pin
const int8_t BTN_PIN = 8;
//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.
const uint32_t FILE_BLOCK_COUNT = 256000;

// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "data"
//------------------------------------------------------------------------------
// Buffer definitions.
//
// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT additional
// buffers.
//
#ifndef RAMEND
// Assume ARM. Use total of nine 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 8;
//
#elif RAMEND < 0X8FF
#error Too little SRAM
//
#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 1;
//
#elif RAMEND < 0X20FF
// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
//
#else  // RAMEND
// Use total of 13 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
#endif // RAMEND
//==============================================================================
// End of configuration constants.
//==============================================================================
// Temporary log file.  Will be deleted if a reset or power failure occurs.
#define TMP_FILE_NAME "tmp_log.bin"

// Size of file base name.  Must not be larger than six.
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

SdFat sd;

SdBaseFile binFile;

char binName[13] = FILE_BASE_NAME "00.bin";

// Number of data records in a block.
const uint16_t DATA_DIM = (512 - 4) / sizeof(data_t);

//Compute fill so block size is 512 bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = 512 - 4 - DATA_DIM * sizeof(data_t);

struct block_t
{
  uint16_t count;
  uint16_t overrun;
  data_t data[DATA_DIM];
  uint8_t fill[FILL_DIM];
};

const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 2;

block_t *emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t *fullQueue[QUEUE_DIM];
uint8_t fullHead;
uint8_t fullTail;

//------------------------------------------------------------------------------
//
void fatalBlink()
{
  while (true)
  {
    if (ERROR_LED_PIN >= 0)
    {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(200);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(200);
    }
  }
}

// Advance queue index.
inline uint8_t queueNext(uint8_t ht)
{
  return ht < (QUEUE_DIM - 1) ? ht + 1 : 0;
}
//==============================================================================
// Error messages stored in flash.
#define error(msg) errorFlash(F(msg))
//------------------------------------------------------------------------------
void errorFlash(const __FlashStringHelper *msg)
{
  sd.errorPrint(msg);
  fatalBlink();
}

int buttonState = 0;     // current state of the button
int lastButtonState = 0; // previous state of the button

//------------------------------------------------------------------------------
// log data
// max number of blocks to erase per erase call
uint32_t const ERASE_SIZE = 262144L;
void logData()
{
  uint32_t bgnBlock, endBlock;

  // Allocate extra buffer space.
  block_t block[BUFFER_BLOCK_COUNT];
  block_t *curBlock = 0;
  Serial.println();

  // Find unused file name.
  if (BASE_NAME_SIZE > 6)
  {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(binName))
  {
    if (binName[BASE_NAME_SIZE + 1] != '9')
    {
      binName[BASE_NAME_SIZE + 1]++;
    }
    else
    {
      binName[BASE_NAME_SIZE + 1] = '0';
      if (binName[BASE_NAME_SIZE] == '9')
      {
        error("Can't create file name");
      }
      binName[BASE_NAME_SIZE]++;
    }
  }
  // Delete old tmp file.
  if (sd.exists(TMP_FILE_NAME))
  {
    Serial.println(F("Deleting tmp file"));
    if (!sd.remove(TMP_FILE_NAME))
    {
      error("Can't remove tmp file");
    }
  }
  // Create new file.
  Serial.println(F("Creating new file"));
  binFile.close();
  if (!binFile.createContiguous(sd.vwd(),
                                TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT))
  {
    error("createContiguous failed");
  }
  // Get the address of the file on the SD.
  if (!binFile.contiguousRange(&bgnBlock, &endBlock))
  {
    error("contiguousRange failed");
  }
  // Use SdFat's internal buffer.
  uint8_t *cache = (uint8_t *)sd.vol()->cacheClear();
  if (cache == 0)
  {
    error("cacheClear failed");
  }

  // Flash erase all data in the file.
  Serial.println(F("Erasing all data"));
  uint32_t bgnErase = bgnBlock;
  uint32_t endErase;
  while (bgnErase < endBlock)
  {
    endErase = bgnErase + ERASE_SIZE;
    if (endErase > endBlock)
    {
      endErase = endBlock;
    }
    if (!sd.card()->erase(bgnErase, endErase))
    {
      error("erase failed");
    }
    bgnErase = endErase + 1;
  }
  // Start a multiple block write.
  if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT))
  {
    error("writeBegin failed");
  }
  // Initialize queues.
  emptyHead = emptyTail = 0;
  fullHead = fullTail = 0;

  // Use SdFat buffer for one block.
  emptyQueue[emptyHead] = (block_t *)cache;
  emptyHead = queueNext(emptyHead);

  // Put rest of buffers in the empty queue.
  for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++)
  {
    emptyQueue[emptyHead] = &block[i];
    emptyHead = queueNext(emptyHead);
  }
  Serial.println(F("Logging - type any character to stop"));
  // Wait for Serial Idle.
  Serial.flush();
  delay(10);
  uint32_t bn = 0;
  uint32_t t0 = millis();
  uint32_t t1 = t0;
  uint32_t overrun = 0;
  uint32_t overrunTotal = 0;
  uint32_t count = 0;
  uint32_t maxLatency = 0;
  int32_t diff;
  // Start at a multiple of interval.
  uint32_t logTime = micros() / LOG_INTERVAL_USEC + 1;
  logTime *= LOG_INTERVAL_USEC;
  bool closeFile = false;
  while (1)
  {
    // Time for next data record.
    logTime += LOG_INTERVAL_USEC;

    buttonState = digitalRead(BTN_PIN);

    if (buttonState != lastButtonState)
    {
      lastButtonState = buttonState;

      if (buttonState == LOW)
      {
        closeFile = true;
      }
    }

    if (closeFile)
    {
      if (curBlock != 0 && curBlock->count >= 0)
      {
        // Put buffer in full queue.
        fullQueue[fullHead] = curBlock;
        fullHead = queueNext(fullHead);
        curBlock = 0;
      }
    }
    else
    {
      if (curBlock == 0 && emptyTail != emptyHead)
      {
        curBlock = emptyQueue[emptyTail];
        emptyTail = queueNext(emptyTail);
        curBlock->count = 0;
        curBlock->overrun = overrun;
        overrun = 0;
      }
      do
      {
        diff = logTime - micros();
      } while (diff > 0);
      if (diff < -10)
      {
        error("LOG_INTERVAL_USEC too small");
      }
      if (curBlock == 0)
      {
        overrun++;
      }
      else
      {
        acquireData(&curBlock->data[curBlock->count++]);
        if (curBlock->count == DATA_DIM)
        {
          fullQueue[fullHead] = curBlock;
          fullHead = queueNext(fullHead);
          curBlock = 0;
        }
      }
    }

    if (fullHead == fullTail)
    {
      // Exit loop if done.
      if (closeFile)
      {
        break;
      }
    }
    else if (!sd.card()->isBusy())
    {
      // Get address of block to write.
      block_t *pBlock = fullQueue[fullTail];
      fullTail = queueNext(fullTail);
      // Write block to SD.
      uint32_t usec = micros();
      if (!sd.card()->writeData((uint8_t *)pBlock))
      {
        error("write data failed");
      }
      usec = micros() - usec;
      t1 = millis();
      if (usec > maxLatency)
      {
        maxLatency = usec;
      }
      count += pBlock->count;

      // Add overruns and possibly light LED.
      if (pBlock->overrun)
      {
        overrunTotal += pBlock->overrun;
        if (ERROR_LED_PIN >= 0)
        {
          digitalWrite(ERROR_LED_PIN, HIGH);
        }
      }
      // Move block to empty queue.
      emptyQueue[emptyHead] = pBlock;
      emptyHead = queueNext(emptyHead);
      bn++;
      if (bn == FILE_BLOCK_COUNT)
      {
        // File full so stop
        break;
      }
    }
  }
  if (!sd.card()->writeStop())
  {
    error("writeStop failed");
  }
  // Truncate file if recording stopped early.
  if (bn != FILE_BLOCK_COUNT)
  {
    Serial.println(F("Truncating file"));
    if (!binFile.truncate(512L * bn))
    {
      error("Can't truncate file");
    }
  }
  if (!binFile.rename(sd.vwd(), binName))
  {
    error("Can't rename file");
  }
  Serial.print(F("File renamed: "));
  Serial.println(binName);
  Serial.print(F("Max block write usec: "));
  Serial.println(maxLatency);
  Serial.print(F("Record time sec: "));
  Serial.println(0.001 * (t1 - t0), 3);
  Serial.print(F("Sample count: "));
  Serial.println(count);
  Serial.print(F("Samples/sec: "));
  Serial.println((1000.0) * count / (t1 - t0));
  Serial.print(F("Overruns: "));
  Serial.println(overrunTotal);
  Serial.println(F("Done"));
}
//------------------------------------------------------------------------------
void setup(void)
{
  if (ERROR_LED_PIN >= 0)
  {
    pinMode(ERROR_LED_PIN, OUTPUT);
  }

  pinMode(BTN_PIN, INPUT);
  digitalWrite(BTN_PIN, HIGH);

  Serial.begin(9600);

  Serial.print(F("Records/block: "));
  Serial.println(DATA_DIM);
  if (sizeof(block_t) != 512)
  {
    error("Invalid block size");
  }
  setupData();
  Serial.println("Initializing gyro...");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  mpu.setFullScaleGyroRange(3);

  // TODO: make autocalibration routine to store values in EEPROM
  mpu.setXAccelOffset(319);
  mpu.setYAccelOffset(48);
  mpu.setZAccelOffset(1142);
  mpu.setXGyroOffset(-18);
  mpu.setYGyroOffset(63);
  mpu.setZGyroOffset(-24);

  // initialize file system.
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED))
  {
    sd.initErrorPrint();
    fatalBlink();
  }
}
//------------------------------------------------------------------------------
void loop(void)
{
  buttonState = digitalRead(BTN_PIN);

  if (buttonState != lastButtonState)
  {
    lastButtonState = buttonState;

    if (buttonState == LOW)
    {
      Serial.println('Starting log...');
      logData();
    }
  }
}
