#ifndef CGUI_H
#define CGUI_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "CRawImage.h"
#include "CTimer.h"
#include "CTransformation.h"

/**
@author ...
*/
class CGui
{
public:
    CGui(int wi, int he, int sc, const char* font_path);
    ~CGui();
    void drawImage(CRawImage* image);
    void drawTimeStats(int evalTime, int numBots);
    void guideCalibration(int calibNum, float dimX, float dimY);
    void displayHelp(bool displayHelp);
    void saveScreen(int a);
    void drawStats(int x, int y, STrackedObject o, bool D2);
    void drawLine(float sx1, float sx2, float sy1, float sy2);
    void drawEllipse(SSegment s, STrackedObject t);
    void update();
    void clearStats();

private:
    int width, height, scale;
    int num;
    SDL_Window* screen;
    SDL_Renderer* renderer;
    TTF_Font* smallFont;
    int averageTime, maxTime, numStats;
};

#endif
