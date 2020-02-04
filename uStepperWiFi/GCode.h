#include <Arduino.h>

#define MAX_PACKET_SIZE 50
#define BUFFER_SIZE 10 // Max fifo buffer size

class GCode {

public:
  GCode(void);

  void setSerialPort(Stream *port);

  /* run() called in the main loop to read incoming Serial data and execute
   * commands  */
  bool run(void);

  void send(char *command);
  void send(char *command, bool checksum);

  bool value(char *name, double *var);

  bool check(char *identifier, char *command);

  char *getNextValue(void);

  bool bufferFull(void);

  uint8_t bufferSize(void);

  void setBufferSize(uint8_t size);

  void setPacket( char * p );

private:
  bool read(void);

  /* Variabel to hold the reference to the Serial object */
  Stream *serialPort = NULL;

  char packet[MAX_PACKET_SIZE];
  uint8_t packetLen = 0;
  uint32_t lastPacket = 0;
  bool newPacket = false;

  const uint16_t timeout = 500;

  char commandBuffer[BUFFER_SIZE + 1][MAX_PACKET_SIZE];

  uint8_t bufferHead = 0;
  uint8_t bufferTail = 0;

  uint8_t bufferMax = BUFFER_SIZE + 1;
};
