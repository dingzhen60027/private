#ifndef GLOBALFUN_H
#define GLOBALFUN_H

#ifndef PI
#define Pi 3.14159265358979323846
#endif // PI

#define _USE_MATH_DEFINES
#include <cmath>

int countMin(int R, int G, int B, int N);
double changeColorFromRGBtoHSI(int r, int g, int b);

#endif // GLOBALFUN_H
