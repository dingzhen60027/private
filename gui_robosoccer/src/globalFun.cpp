#include "globalFun.h"

int countMin(int R, int G, int B, int N)
{
    int temp = 0;
    if (N == 3)
    {
        if (R <= G && R <= B)
            temp = R;
        else if (G <= R && G <= B)
            temp = G;
        else if (B <= R && B <= G)
            temp = B;
    }
    else if (N == 2)
    {
        if (R <= G) temp = R;
        else temp = G;
    }
    return temp;
}

double changeColorFromRGBtoHSI(int r, int g, int b)
{
    double H = 0;
    double R, G, B;
    R = double(r);
    G = double(g);
    B = double(b);
    if (R == G && G == B)
    {
        H = 0;
    }
    else
    {
        if (G >= B)
        {
            H = acos(((R - G) + (R - B)) / 2 / sqrt(((R - G) * (R - G)) + (R - B) * (R - B))) * 180 / Pi;
        }
        else
            H = 360 - acos(((R - G) + (R - B)) / 2 / sqrt((R - G) * (R -G) + (R - B) * (R - B))) * 180 / Pi;
    }
    return H;
}
