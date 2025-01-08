#include <SPI.h>


#define SCK_PIN   10
#define TX_PIN    11
#define RX_PIN    12
#define CS_PIN    13

// MCP2515 registers
#define REG_CANSTAT 0x0e
#define REG_CANCTRL 0x0f
#define REG_CNF3 0x28
#define REG_CNF2 0x29
#define REG_CNF1 0x2a
#define REG_TXBnCTRL(n) (0x30 + (n * 0x10))
#define REG_CANINTF 0x2c
#define REG_EFLG 0x2d

#define FLAG_RXnIF(n) (0x01 << n)

#define REG_RXFnSIDH(n) (0x00 + (n * 4))
#define REG_RXFnSIDL(n) (0x01 + (n * 4))
#define REG_RXFnEID8(n) (0x02 + (n * 4))
#define REG_RXFnEID0(n) (0x03 + (n * 4))

#define REG_RXBnCTRL(n) (0x60 + (n * 0x10))
#define REG_RXBnSIDH(n) (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n) (0x62 + (n * 0x10))
#define REG_RXBnEID8(n) (0x63 + (n * 0x10))
#define REG_RXBnEID0(n) (0x64 + (n * 0x10))
#define REG_RXBnDLC(n) (0x65 + (n * 0x10))
#define REG_RXBnD0(n) (0x66 + (n * 0x10))

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000)
// SPI Modes 1 and 3 work, I have no idea if that matches "0,0 and 1,1"
SPISettings SPIS_MCP_Port(2000000, MSBFIRST, SPI_MODE1);
int iCounter = 0;
uint8_t bSPI_Tx[8] = {0,0,0,0,0,0,0,0};
uint8_t bSPI_Rx[8] = {0,0,0,0,0,0,0,0};
uint8_t status = 0;
uint8_t errors = 0;
uint8_t delay_count = 0;
uint8_t data_length = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Sender test!");
  SPI1.setSCK(SCK_PIN);
  SPI1.setTX(TX_PIN);
  SPI1.setRX(RX_PIN);
  SPI1.setCS(CS_PIN);
  Serial.println("CS pin set...");
  SPI1.begin(true);
  Serial.println("SPI peripherial started");
  SPI1.beginTransaction(SPIS_MCP_Port);
  Serial.println("SPI Transaction begun");
  // reset state
  Serial.println("Attempting to reset MCP...");
  sendReset();
  writeRegister(REG_CANCTRL, 0x87);
  if (readRegister(REG_CANCTRL) != 0x87) {
    Serial.println("Failed to reset MCP!");
    Serial.print("got value: ");
    Serial.print(readRegister(REG_CANCTRL));
    Serial.println(", expected 0x87");
    while (true) {
      delay(100);
    }
  }
  Serial.println("Reset of MCP complete...");
  // set baud rate to 1mbps with 16mhz osc: {(long)16E6, (long)1000E3, {0x00, 0xd0, 0x82}},
  Serial.println("Attempting to set CAN Baud rate...");
  // 16mhz cryastal @ 1mbs : 0x00, 0xd0, 0x82
  // 8mhz crysttal @ 1mbps : 0x00, 0x80, 0x00
  writeRegister(REG_CNF1, 0x00);
  writeRegister(REG_CNF2, 0xD0);
  writeRegister(REG_CNF3, 0x82);
  if (readRegister(REG_CNF1) != 0x00) {
    Serial.println("Failed to set CNF1!");
  }
  if (readRegister(REG_CNF2) != 0xD0) {
    Serial.println("Failed to set CNF2!");
  }
  if (readRegister(REG_CNF3) != 0x82) {
    Serial.println("Failed to set CNF3!");
  }
  Serial.println("Setting CAN Baud complete...");

  Serial.println("Attempting to disable CAN filter...");
  writeRegister(REG_RXBnCTRL(0), 0x60);
  if (readRegister(REG_RXBnCTRL(0)) != 0x60) {
    Serial.println("Failed to disable filter!");
  }
  
  // Set active mode
  Serial.println("Attempting to set MCP to normal mode...");
  writeRegister(REG_CANCTRL, 0x07);
  if (readRegister(REG_CANCTRL) != 0x07) {
    Serial.println("Failed to activate MCP!");
    Serial.print("got value: ");
    Serial.print(readRegister(REG_CANCTRL));
    Serial.println(", expected 0x07");
    while (true) {
      delay(100);
    }
  }
  status = readRegister(REG_TXBnCTRL(0));
  Serial.print("Tx0 status: 0b");
  Serial.println(status, BIN);
  errors = readRegister(REG_EFLG);
  Serial.print("error flags: 0b");
  Serial.println(errors, BIN);

  //Set RX id filter


  //Serial.println("MCP2515 chip found");
}

void loop() {
  uint8_t data_buffer[8] = {0,0,0,0,0,0,0,0};
  uint32_t id_regs[4] = {0,0,0,0};
  uint32_t raw_id = 0;
  uint32_t raw_angle = 0;
  uint32_t raw_velocity = 0;
  uint32_t raw_torque = 0;
  uint32_t raw_temp = 0;
  float angle = 0;
  float velocity = 0;
  float torque = 0;
  float temp = 0;

  // send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending extended packet...");
  for( int i = 0; i < 8; i++) {
    data_buffer[i] = 0;
  }
  writeTxFromID(0, 0x400000B, data_buffer, 8);
  sendTx0();
  Serial.print("Sending...");
  while ((readRegister(REG_TXBnCTRL(0)) & 0x08) > 0) {
    delay(10);
  }
  Serial.print("sent...");
  
  Serial.print("Waiting...");
  delay_count = 0;
  while (delay_count++ < 10 && ((readRegister(REG_CANINTF) & 0x01) == 0)) {
    Serial.print("Interrupt flags: 0b");
    Serial.println(readRegister(REG_CANINTF), BIN);
    Serial.print(".");
    delay(10);
  }
  if((readRegister(REG_CANINTF) & 0x01) == 0) {
    Serial.println(".");
    Serial.println("Timeout while waiting.");
  }
  else {
    Serial.println("Recieved.");
    id_regs[0] = readRegister(REG_RXBnEID0(0));
    id_regs[1] = readRegister(REG_RXBnEID8(0));
    id_regs[2] = readRegister(REG_RXBnSIDL(0));
    id_regs[3] = readRegister(REG_RXBnSIDH(0));
    raw_id = id_regs[0];
    raw_id += (id_regs[1] << 8);
    raw_id += (id_regs[2] << 16) & 3;
    raw_id += (id_regs[2] << 13) & 22;
    raw_id += (id_regs[3] << 21);

    // Serial.print("Comm Type: ");
    // Serial.print((raw_id >> 24), DEC);
    // Serial.print("  Bytes: 0x");
    // Serial.print((raw_id >> 16) & 0xFF, HEX);
    // Serial.print(" 0x");
    // Serial.print((raw_id >> 8) & 0xFF, HEX);
    // Serial.print(" 0x");
    // Serial.println(raw_id & 0xFF, HEX);

    // Serial.print("Data: ");
    data_length = readRegister(REG_RXBnDLC(0));
    // Serial.print(data_length, DEC);
    // Serial.print(" bytes:");
    for( int i = 0; i < data_length; i++) {
      data_buffer[i] = readRegister(REG_RXBnD0(0) + i);
      // Serial.print(" 0x");
      // Serial.print(data_buffer[i], HEX);
    }

    raw_angle = data_buffer[1] + (256 * data_buffer[0]);
    raw_velocity = data_buffer[3] + (256 * data_buffer[2]);
    raw_torque = data_buffer[5] + (256 * data_buffer[4]);
    raw_temp = data_buffer[7] + (256 * data_buffer[6]);

    angle = (((float)raw_angle - 32767.f) / 32767.f) * 4.f * 3.14159f;
    velocity = (((float)raw_velocity - 32767.f) / 32767.f) * 30.0f;
    torque = (((float)raw_torque - 32767.f) / 32767.f) * 12.0f;
    temp = (float)raw_temp / 10.0f;

    Serial.print("Angle:");
    Serial.print(angle, 3);
    Serial.print("rad, ");
    Serial.print("Velocity:");
    Serial.print(velocity, 3);
    Serial.print("rad/s, ");
    Serial.print("Torque:");
    Serial.print(torque, 3);
    Serial.print("Nm, ");
    Serial.print("Temp:");
    Serial.print(temp, 2);
    Serial.println("°C ");

    status = 0;
    while(readRegister(REG_CANINTF) != 0) {
      status++;
      writeRegister(REG_CANINTF, 0);
    }
  }

  //Response should be ID 0x0BFE
  //Data will be 64 bit unique ID


  // mcp.beginExtendedPacket(0xabcdef);
  // mcp.write('w');
  // mcp.write('o');
  // mcp.write('r');
  // mcp.write('l');
  // mcp.write('d');
  // mcp.write(' ');
  // mcp.write('0' + (iCounter++ % 10));
  // mcp.endPacket();

  delay(100);
}

void writeRegister(uint8_t address, uint8_t value) {
  uint8_t buffer[3] = {0x02, address, value};
  SPI1.transfer(buffer, nullptr, 3);
}

uint8_t readRegister(uint8_t address) {
  uint8_t buffer[3] = {0x03, address, 0};
  uint8_t rx_buffer[3] = {0,0,0};
  SPI1.transfer(buffer, rx_buffer, 3);
  return rx_buffer[2];
}

void writeTxFromID(uint8_t buf_n, long id, const void *data_buf, size_t count) {
  uint8_t command = 0x40 + (buf_n << 1);
  uint8_t tx_buf[14] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0 };
  if (buf_n > 2 || buf_n < 0) return;
  if (count > 8) return;
  tx_buf[0] = command;
  //SIDH
  tx_buf[1] = (uint8_t)(id >> 21); // |SID10|...|SID3|
  // |SID2|SID1|SID0|-|EXIDE|-|EID17|EID16|
  tx_buf[2] = 0x08 | (uint8_t)(((id >> 18) & 0x07) << 5 ) | (uint8_t)((id >> 16) & 0x03);
  tx_buf[3] = (uint8_t)(0xFF & (id >> 8)); // |EID15|...|EID8|
  tx_buf[4] = (uint8_t)(0xFF & id); // |EID7|...|EID0|
  tx_buf[5] = (uint8_t)count; // |-|RTR|-|-|DLC3|DLC2|DLC1|DLC0|
  memcpy(&tx_buf[6], data_buf, count);
  //shove it all over SPI
  SPI1.transfer(tx_buf, nullptr, 6 + count);
}

void writeTxData(uint8_t buf_n, const void *data_buf, size_t count) {
  uint8_t command = 0x40 + (buf_n << 1) + 1;
  uint8_t tx_buf[9] = {0,0,0,0,0,0,0,0};
  if (buf_n > 2 || buf_n < 0) return;
  if (count > 8) return;
  tx_buf[0] = command;
  //TXnD0
  memcpy(&tx_buf[1], data_buf, count);
  //shove it all over SPI
  SPI1.transfer(tx_buf, nullptr, count + 1);
}

void writeRxFilter(uint8_t number, long id) {
  uint8_t filter_buf[4] = {0,0,0,0};
  //SIDH
  filter_buf[0] = (uint8_t)(id >> 3); // |SID10|...|SID3|
  // |SID2|SID1|SID0|-|EXIDE|-|EID17|EID16|
  filter_buf[1] = 0x08 | (uint8_t)((id & 0x07) << 5) | (uint8_t)((id >> 27) & 0x03);
  filter_buf[2] = (uint8_t)(0xFF & (id >> 19)); // |EID15|...|EID8|
  filter_buf[3] = (uint8_t)(0xFF & (id >> 11)); // |EID7|...|EID0|
  //shove it all over SPI
  writeRegister( REG_RXFnSIDH(number), filter_buf[0]);
  writeRegister( REG_RXFnSIDL(number), filter_buf[1]);
  writeRegister( REG_RXFnEID8(number), filter_buf[2]);
  writeRegister( REG_RXFnEID0(number), filter_buf[3]);
}

void modRegister(uint8_t address, uint8_t mask, uint8_t value) {
  uint8_t buffer[4] = {0x05, address, mask, value};
  SPI1.transfer(buffer, nullptr, 4);
}

void sendTx0() {
  SPI1.transfer(0x81);
}

void sendTx1() {
  SPI1.transfer(0x82);
}

void sendTx2() {
  SPI1.transfer(0x84);
}

void sendReset() {
  SPI1.transfer(0xC0);
}
