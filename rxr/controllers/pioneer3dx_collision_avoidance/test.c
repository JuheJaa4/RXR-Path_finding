int* getGrid(WbDeviceTag tag) {
  int gridY = (y - 0.50 + 5);
  int gridX = (x - 0.50);
  int grid[2] = {gridX, gridY};

  return grid;
}

void getNextDirection(WbDeviceTag gps, WbDeviceTag cmps) {
  int nextGrids[4][2];
  int *tmp = getGrid(gps);

  //Upwards next grid
  printf("%d\n", tmp[1]);
  printf("%d\n", (tmp[1] - 1));
  nextGrids[0][1] = tmp[1] - 1;
  nextGrids[0][0] = tmp[0];
}