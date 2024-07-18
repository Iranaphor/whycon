#include "CGui.h"

#define THICK_CROSS

CGui::CGui(int wi, int he, int sc, const char* font_path)
{
    averageTime = maxTime = numStats = 0;
    height = he / sc;
    width = wi / sc;
    scale = sc;
    SDL_Init(SDL_INIT_VIDEO);
    if (TTF_Init() == -1) printf("Unable to initialize SDL_ttf: %s\n", TTF_GetError());

    // Create window and renderer
    screen = SDL_CreateWindow("WHYCON", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_SHOWN);
    if (screen == NULL) fprintf(stderr, "Couldn't set SDL video mode: %s\r\n", SDL_GetError());

    renderer = SDL_CreateRenderer(screen, -1, SDL_RENDERER_ACCELERATED);

    smallFont = TTF_OpenFont(font_path, 16);
    if (!smallFont) printf("Unable to open font: %s\n", TTF_GetError());
    TTF_SetFontStyle(smallFont, TTF_STYLE_NORMAL);
    num = 0;
}

CGui::~CGui()
{
    TTF_CloseFont(smallFont);
    TTF_Quit();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(screen);
    SDL_Quit();
}

void CGui::clearStats()
{
    averageTime = maxTime = numStats = 0;
}

void CGui::drawImage(CRawImage* image)
{
    CRawImage *imageSrc = image;

    if (scale != 1){
        int wi = width;
        int he = height;
        imageSrc = new CRawImage(wi, he, 3);
        for (int j = 0; j < he; j++){
            int srp = (j * scale) * wi * scale * 3;
            int dep = j * wi * 3;
            for (int i = 0; i < wi; i++){
                int dp = dep + i * 3;
                int sp = srp + scale * i * 3;
                imageSrc->data[dp] = image->data[sp];
                imageSrc->data[dp + 1] = image->data[sp + 1];
                imageSrc->data[dp + 2] = image->data[sp + 2];
            }
        }
    }

    SDL_Surface *imageSDL = SDL_CreateRGBSurfaceFrom(imageSrc->data, imageSrc->width, imageSrc->height, imageSrc->bpp * 8, imageSrc->bpp * imageSrc->width, 0x000000ff, 0x0000ff00, 0x00ff0000, 0x00000000);
    if (imageSDL != NULL) {
        SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, imageSDL);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_DestroyTexture(texture);
    }
    SDL_FreeSurface(imageSDL);
    if (scale != 1) delete imageSrc;
}

void CGui::drawTimeStats(int evalTime, int numBots)
{
    char info[1000];
    SDL_Surface *text;
    SDL_Texture *texture;
    SDL_Rect rect;

    rect.x = 0;
    rect.y = 0;
    rect.w = 0;
    rect.h = 0;
    SDL_Color ok_col = { 255, 0, 0, 0 };

    if (evalTime > maxTime) maxTime = evalTime;
    averageTime += evalTime;
    numStats++;
    sprintf(info, "             Tracking of %i robots took %.3f ms.", numBots, evalTime / 1000.0);
    text = TTF_RenderUTF8_Blended(smallFont, info, ok_col);
    texture = SDL_CreateTextureFromSurface(renderer, text);
    SDL_QueryTexture(texture, NULL, NULL, &rect.w, &rect.h);
    SDL_RenderCopy(renderer, texture, NULL, &rect);
    SDL_DestroyTexture(texture);
    SDL_FreeSurface(text);
}

void CGui::guideCalibration(int calibNum, float dimX, float dimY)
{
    if (calibNum >= 0 && calibNum <= 4){
        char ttt[100];
        const char *texty[] ={
            " Click the circle at the [0.000,0.000].",
            " Click the circle at the [%.3f,0.000].",
            " Click the circle at the [0.000,%.3f].",
            " Click the circle at the [%.3f,%.3f].",
        };

        SDL_Surface *text;
        SDL_Texture *texture;
        SDL_Rect rect;
        rect.x = width / 2 - 130;
        rect.y = height / 2;
        rect.w = 260;
        rect.h = 14;
        SDL_Color ok_col = { 0, 255, 0, 0 };
        sprintf(ttt, texty[calibNum], dimX, dimY);
        if (calibNum == 2) sprintf(ttt, texty[calibNum], dimY);
        text = TTF_RenderUTF8_Blended(smallFont, ttt, ok_col);
        texture = SDL_CreateTextureFromSurface(renderer, text);
        SDL_QueryTexture(texture, NULL, NULL, &rect.w, &rect.h);
        SDL_RenderCopy(renderer, texture, NULL, &rect);
        SDL_DestroyTexture(texture);
        SDL_FreeSurface(text);
    }
}

void CGui::displayHelp(bool displayHelp)
{
    const char *helpText[] = {
        "[h] - to display/hide help,",
        "[l] - draw/hide coordinates,",
        "[s] - save current image,",
        "[d] - draw/hide segmentation outcome,",
        "[-] - decrease the number of tracked patterns,",
        "[+] - increase the number of tracked patterns,",
        "[3] - switch to 3D coordinage system,",
        "[2] - switch to planar coordinage system,",
        "[1] - switch to camera coordinage system,",
        "[r] - recalibrate,"
    };

    SDL_Surface *text;
    SDL_Texture *texture;
    SDL_Rect rect;
    int numStrings = 10;
    if (displayHelp == false) numStrings = 1;
    rect.x = 0;
    rect.y = height - 22 * numStrings;
    rect.w = 350;
    rect.h = 22 * numStrings;
    SDL_Color ok_col = { 0, 255, 0, 0 };
    for (int i = 0; i < numStrings; i++){
        rect.y = height - (i + 1) * 22;
        text = TTF_RenderUTF8_Blended(smallFont, helpText[i], ok_col);
        texture = SDL_CreateTextureFromSurface(renderer, text);
        SDL_QueryTexture(texture, NULL, NULL, &rect.w, &rect.h);
        SDL_RenderCopy(renderer, texture, NULL, &rect);
        SDL_DestroyTexture(texture);
        SDL_FreeSurface(text);
    }
}

void CGui::saveScreen(int a)
{
    SDL_Surface* saveSurface = SDL_CreateRGBSurface(0, width, height, 24, 0x000000ff, 0x0000ff00, 0x00ff0000, 0);
    SDL_RenderReadPixels(renderer, NULL, SDL_PIXELFORMAT_RGB24, saveSurface->pixels, saveSurface->pitch);

    CRawImage image((unsigned char*)saveSurface->pixels, width, height, 3); // added 3 in constructor because of change in CRawImage
    image.swapRGB();
    char name[1000];
    if (a == -1) num++; else num = a;
    sprintf(name, "output/%09i.bmp", num);
    image.saveBmp(name);

    SDL_FreeSurface(saveSurface);
}

void CGui::drawStats(int x, int y, STrackedObject o, bool D2)
{
    SDL_Surface *text;
    SDL_Texture *texture;
    SDL_Rect rect; // text position on the screen

    rect.x = x / scale;
    rect.y = y / scale;
    rect.w = 0;
    rect.h = 0;
    SDL_Color ok_col = { 255, 0, 0, 0 };
    char info[1000];

    if (D2){
        sprintf(info, "%02i %03i", o.ID, (int)(o.yaw / M_PI * 180));
        text = TTF_RenderUTF8_Blended(smallFont, info, ok_col);
        rect.y = y / scale + 14;
        texture = SDL_CreateTextureFromSurface(renderer, text);
        SDL_QueryTexture(texture, NULL, NULL, &rect.w, &rect.h);
        SDL_RenderCopy(renderer, texture, NULL, &rect);
        SDL_DestroyTexture(texture);
        SDL_FreeSurface(text);
    }

    if (D2) sprintf(info, "%03.0f %03.0f", 1000 * o.x, 1000 * o.y); else sprintf(info, "%.3f %.3f %i", o.x, o.y, o.ID);
    text = TTF_RenderUTF8_Blended(smallFont, info, ok_col);
    rect.y = y / scale;
    texture = SDL_CreateTextureFromSurface(renderer, text);
    SDL_QueryTexture(texture, NULL, NULL, &rect.w, &rect.h);
    SDL_RenderCopy(renderer, texture, NULL, &rect);
    SDL_DestroyTexture(texture);
    SDL_FreeSurface(text);

    if (D2 == false){
        sprintf(info, "%.3f %.3f %.3f", o.pitch, o.roll, o.yaw);
        text = TTF_RenderUTF8_Blended(smallFont, info, ok_col);
        rect.y = y / scale + 14;
        texture = SDL_CreateTextureFromSurface(renderer, text);
        SDL_QueryTexture(texture, NULL, NULL, &rect.w, &rect.h);
        SDL_RenderCopy(renderer, texture, NULL, &rect);
        SDL_DestroyTexture(texture);
        SDL_FreeSurface(text);
    }
}

void CGui::drawLine(float sx1, float sx2, float sy1, float sy2)
{
    float d, r;

    r = (sy1 - sy2) / (sx1 - sx2);
    if (fabs(r) < 1){
        if (sx1 > sx2){
            d = sx1;
            sx1 = sx2;
            sx2 = d;
            d = sy1;
            sy1 = sy2;
            sy2 = d;
        }
        for (float x = sx1; x < sx2; x++){
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            SDL_RenderDrawPoint(renderer, x, (x - sx1) * r + sy1);
        }
    }
    else{
        if (sy1 > sy2){
            d = sx1;
            sx1 = sx2;
            sx2 = d;
            d = sy1;
            sy1 = sy2;
            sy2 = d;
        }
        for (float y = sy1; y < sy2; y++){
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            SDL_RenderDrawPoint(renderer, (y - sy1) / r + sx1, y);
        }
    }
}

void CGui::drawEllipse(SSegment s, STrackedObject t)
{
    float sx1, sx2, sy1, sy2;
    int x, y;
    sx1 = s.x + s.v0 * s.m0 * 2;
    sx2 = s.x - s.v0 * s.m0 * 2;
    sy1 = s.y + s.v1 * s.m0 * 2;
    sy2 = s.y - s.v1 * s.m0 * 2;
    drawLine(sx1, sx2, sy1, sy2);
    sx1 = s.x + s.v1 * s.m1 * 2;
    sx2 = s.x - s.v1 * s.m1 * 2;
    sy1 = s.y - s.v0 * s.m1 * 2;
    sy2 = s.y + s.v0 * s.m1 * 2;
    drawLine(sx1, sx2, sy1, sy2);
    for (float a = 0; a < 6.28; a += 0.01){
        float fx = s.x + cos(a) * s.v0 * s.m0 * 2 + s.v1 * s.m1 * 2 * sin(a);
        float fy = s.y + s.v1 * s.m0 * 2 * cos(a) - s.v0 * s.m1 * 2 * sin(a);
        x = (int)(fx + 0.5);
        y = (int)(fy + 0.5);
        if (x > 0 && y > 0 && x < width && y < height){
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
            SDL_RenderDrawPoint(renderer, x, y);
        }
    }
}

void CGui::update()
{
    SDL_RenderPresent(renderer);
}
