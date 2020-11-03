struct RSLidar_RING
{
	int32_t nNumOfPoints;//number of points in each ring;
  float gfX[2200];//in m;
  float gfY[2200];//in m;
  float gfZ[2200];//in m;
  int8_t gnI[2200];//intensity;
}

struct RSLidar_32_FRAME
{
  HEADER stHeader;  
  RSLidar_RING gstRingPoints[32];
}
