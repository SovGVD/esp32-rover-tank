void initFailsafe () {
  
}

void updateFailsafe() {
  if (FS_WS_count >= FS_WS_THR) {
    setFailsafe();
    return;
  }
  
  FS_FAIL = false;
}

void setFailsafe() {
  speed = 0;
  yaw   = 0;

  FS_WS_count = FS_WS_THR;
  FS_FAIL     = true;
}
