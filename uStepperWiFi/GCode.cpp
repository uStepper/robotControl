#include "GCode.h"

GCode::GCode(void) {}

void GCode::setSerialPort(Stream *port) { this->serialPort = port; }

void GCode::setBufferSize(uint8_t size) {

  if (size <= BUFFER_SIZE) {
    this->bufferMax = size + 1;
  } else {
    this->bufferMax = BUFFER_SIZE + 1;
  }
}

bool GCode::check(char *identifier, char *command) {

  if (strstr(command, identifier) != NULL) {

    // Reuse the packet buffer for parameter extraction
    strcpy(this->packet, command);

    return true;
  }

  return false;
}

uint8_t GCode::bufferSize(void) {

  uint8_t size = this->bufferMax - 1;

  if (!this->bufferFull()) {
    if (this->bufferHead >= this->bufferTail) {
      size = (this->bufferHead - this->bufferTail);
    } else {
      size = (this->bufferMax + this->bufferHead - this->bufferTail);
    }
  }

  return size;
}

bool GCode::bufferFull(void) {

  if ((bufferHead + 1) % this->bufferMax == bufferTail) {
    return true;
  }

  return false;
}

bool GCode::run(void) {

  if (this->read()) {

    // Check if the buffer is full
    if (this->bufferFull() == false) {

      // Buffer not full, add command to buffer

      /* Serial.print("Head: ");
      Serial.println(bufferHead); */
      // Serial.println( this->packet );

      strcpy(commandBuffer[bufferHead], this->packet);
      bufferHead = (bufferHead + 1) % this->bufferMax;
    }

    // A command is ready to be read.
    return true;

  } else if (this->bufferSize() > 0) {

    return true; // Some command is within the buffer waiting to be read
  }

  return false;
}

char *GCode::getNextValue(void) {

  // Check if anything is in the buffer
  if (bufferHead == bufferTail) {
    // No more in buffer
    return NULL;
  } else {
    /* Serial.print("Tail: ");
    Serial.println(bufferTail); */

    char *command = commandBuffer[bufferTail];
    bufferTail = (bufferTail + 1) % this->bufferMax;

    return command;
  }
}

void GCode::setPacket(char * p){
  strcpy(this->packet, p);
}

bool GCode::value(char *name, double *var) {

  char *start;
  char *end;
  size_t len;

  char buf[20] = {'\0'};

  // Find start of parameter value
  if (start = strstr(this->packet, name)) {

    start++; // Not interested in the param name

    // Find end of parameter value
    if (end = strpbrk(start, " \n")) {

      len = end - start;

      strncpy(buf, start, len);
      buf[len] = '\0';

      // Now convert the string in buf to a float
      *var = atof(buf);

      return true;
    }
  }

  return false;
}

bool GCode::read(void) {

  char buf;
  double checksum;

  if (this->newPacket == false && this->packetLen > 0) {
    this->packetLen = 0;
    memset(this->packet, 0, sizeof(this->packet));
  }

  if (this->serialPort->available() > 0) {
    this->lastPacket = micros();

    // Read the newest char from the UART buffer
    buf = this->serialPort->read();
    // Save the char in the buffer ( but limited by MAX_PACKET_SIZE )
    if (this->packetLen < MAX_PACKET_SIZE - 1) {
      this->packet[this->packetLen++] = buf;
    }

    this->newPacket = true;
  }

  // If the timeout has been reached, or a newline has been received the string
  // is complete
  if ((micros() - this->lastPacket >= this->timeout || buf == '\n') &&
      this->newPacket == true) {

    this->newPacket = false;

    // Check for checksum
    if (this->value("*", &checksum)) {

      // Checksum appended, check if it correct
      if ((uint8_t)checksum == 0xff) {
        return true;
      }
    }
  }

  return false;
}

void GCode::send(char *command) { this->send(command, true); }

void GCode::send(char *command, bool checksum) {

  char buf[MAX_PACKET_SIZE] = {'\0'};

  strcpy(buf, command);

  if (checksum) {
    strcat(buf,
           " *255"); // Append checksum, should be calculated from entire string
  }

  this->serialPort->println(
      buf); // Always append a newline to indicate end of command
}
