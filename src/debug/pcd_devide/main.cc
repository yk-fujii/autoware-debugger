#include <stdio.h>
#include <string>
#include <vector>

#define MAX_LEN 255

int main(void) {
  std::vector<std::string> header;
  char buf[MAX_LEN];

  FILE *moto = fopen("Laser-00005_-01232.pcd", "rb");
  FILE *saki;

  for (int i = 0; i < 11; i++) {
    fgets(buf, MAX_LEN, moto);
    header.push_back(std::string(buf));
  }

  int max = 1024 * 1024 * 15;
  int counter = 0;
  char write_name[244];
  do {
    if ((counter++ % max) == 0) {
      //      if (saki)
      //      fclose(saki);
      sprintf(write_name, "test-%d.pcd", counter / max);
      saki = fopen(write_name, "wb");
#if 0
      fprintf(saki, "# PCD7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F "
                    "F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 "
                    "0\nPOINTS %d\nDATA binary\n",
              max, max);
#else
      for (const auto &h : header) {
        if (h.find("POINTS") != std::string::npos)
          fprintf(saki, "POINTS %d\n", max);
        else if (h.find("WIDTH") != std::string::npos)
          fprintf(saki, "WIDTH %d\n", max);
        else
          fprintf(saki, "%s", h.c_str());
      }
#endif
    }
    float xyz[3];
    if (!fread(&xyz, sizeof(float), 3, moto))
      break;
    fwrite(&xyz, sizeof(float), 3, saki);

    unsigned int col;
    if (!fread(&col, sizeof(unsigned int), 1, moto))
      break;
    fwrite(&col, sizeof(unsigned int), 1, saki);
  } while (1 /*counter <= max*/);
  if (saki)
    fclose(saki);
  if (moto)
    fclose(moto);
}
