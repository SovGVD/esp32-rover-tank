int bytesToInt16(uint8_t byte1, uint8_t byte2) {
  return (byte1 << 8) | byte2;
}

void pMove(uint8_t* data) {
  if (data[1] == 0) setFailsafe();
  
  speed = float(bytesToInt16(data[2], data[3]))/10000-1;
  yaw = float(bytesToInt16(data[4], data[5]))/10000-1;
}
