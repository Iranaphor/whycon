#ifndef CIMAGE_H
#define CIMAGE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief Class to handle raw image data.
 * 
 * This class provides methods for handling raw image data, including loading, saving, and basic manipulation.
 */
class CRawImage
{
public:
    CRawImage(int wi, int he, int bppi);
    CRawImage(unsigned char *datai, int wi, int he, int bppi);
    ~CRawImage();

    void saveBmp(const char* name);
    void saveBmp();
    bool loadBmp(const char* name);
    void swap();
    void swapRGB();
    void plotLine(int x, int y);
    void plotCenter();
    int getSaveNumber();
    double getOverallBrightness(bool upperHalf);

    int width;    ///< Image width
    int height;   ///< Image height
    int palette;  ///< Image palette
    int size;     ///< Image size
    int bpp;      ///< Bits per pixel
    unsigned char header[122]; ///< BMP header
    unsigned char* data; ///< Image data
    bool ownData; ///< Flag to indicate ownership of data
    int numSaved; ///< Number of saved images
};

#endif // CIMAGE_H
