/*
============================================================================
= Sobel.c implements the Edge Detection with Sobel Algorithm               =
= ------------------------------------------------------------------------ =
= Inputs (both inputs are hardwired in the source code):                                                                  =
=   - Image: file bmp image                                                =
=   - Desired Threshold: T, integer from 0 to 255                          =
= Output:                                                                  =
=   - Image with Edges Detected: "Output_Sobel_<T>.bmp"                           =
= Run:                                                                     =
=  copy-paste the input image.bmp to _project_name_/_project_name_ folder  =
=  same folder where the .cpp file is located                              =
============================================================================
*/
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <string.h>

#define THRESHOLD	90   // input threshold for Sobel 

// bmp file header 
typedef struct tagBitmapFileHeader {
#pragma pack(push, 1)
	unsigned short bfType;
	unsigned int bfSize;
	unsigned short bfReserved1;
	unsigned short bfReserved2;
	unsigned int bfOffBits;
#pragma pack(pop)
} BitmapFileHeader;

// bmp extra file header
typedef struct tagBitmapInfoHeader {
#pragma pack(push, 1)
	unsigned int biSize;
	int biWidth;
	int biHeight;
	unsigned short biPlanes;
	unsigned short biBitCount;
	unsigned int biCompression;
	unsigned int biSizeImage;
	int biXPelsPerMeter;
	int biYPelsPerMeter;
	unsigned int biClrUsed;
	unsigned int biClrImportant;
#pragma pack(pop)
} BitmapInfoHeader;

typedef struct {                      /**** Colormap entry structure ****/
	unsigned char  rgbBlue;          /* Blue value */
	unsigned char  rgbGreen;         /* Green value */
	unsigned char  rgbRed;           /* Red value */
	unsigned char  rgbReserved;      /* Reserved */
} RGBQUAD;

RGBQUAD image[2048][2048];          // Image as input  - just use a big table to store the colors
unsigned char ee_image[2048][2048]; // To store the annotations (output) of the edge detection 
int gray_image[2048][2048];         // To hold the Grayscale image

// the following functions must be implemented in assembly
extern "C" int bmptogray_conversion(int, int, RGBQUAD input_color[2048][2048], int output_gray[2048][2048]);   // first function to be implemented in assembly 
extern "C" int sobel_detection(int, int, int input_gray_image[2048][2048], unsigned char output_ee_image[2048][2048], double threshold); // second function to be implemented in assembly 
extern "C" int border_pixel_calculation(int, int, unsigned char ee_image[2048][2048]);  // third  function to be implemented in assembly 

int main()
{
	FILE* fp;
	FILE* wp;
	unsigned int width, height;
	unsigned int x, y;
	BitmapFileHeader bmfh;
	BitmapInfoHeader bmih;

	/*
	=================================================================================================
	= It seems that visual studio believes that fopen is not safe anymore                           =
	= to turn-off this:                                                                             =
	=    i) project --> properties                                                                  =
	= 	 ii) in the dialog, chose Configuration Properties->C/C++->Preprocessor                     =
	= 	 iii) in the field PreprocessorDefinitions append the following: _CRT_SECURE_NO_WARNINGS    =
	= The above step will allow you to use the fopen function                                       =
	=================================================================================================
	*/

	// Opening the file: using "rb" mode to read this *binary* file
	fp = fopen("input.bmp", "rb");
	if (fp == NULL)
	{
		printf("error when reading the file");
		return 0;
	}

	printf("file opened \n");
	// Reading the file header and any following bitmap information...
	fread(&bmfh, sizeof(BitmapFileHeader), 1, fp);
	fread(&bmih, sizeof(BitmapInfoHeader), 1, fp);

	printf("Header Info\n");
	printf("--------------------\n");
	printf("Size:%i\n", bmfh.bfSize);
	printf("Offset:%i\n", bmfh.bfOffBits);
	printf("--------------------\n");
	printf("Size:%i\n", bmih.biSize);
	printf("biWidth:%i\n", bmih.biWidth);
	printf("biHeight:%i\n", bmih.biHeight);
	printf("biPlanes:%i\n", bmih.biPlanes);
	printf("biBitCount:%i\n", bmih.biBitCount);
	printf("biCompression:%i\n", bmih.biCompression);
	printf("biSizeImage:%i\n", bmih.biSizeImage);
	printf("biXPelsPerMeter:%i\n", bmih.biXPelsPerMeter);
	printf("biYPelsPerMeter:%i\n", bmih.biYPelsPerMeter);
	printf("biClrUsed:%i\n", bmih.biClrUsed);
	printf("biClrImportant:%i\n", bmih.biClrImportant);
	printf("--------------------\n");


	// Extract the width & height from bmp header info
	width = bmih.biWidth; if (width % 4 != 0) width += (4 - width % 4);
	height = bmih.biHeight;
	

	// Reading the pixels of input image
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			image[x][y].rgbBlue = fgetc(fp);
			image[x][y].rgbGreen = fgetc(fp);
			image[x][y].rgbRed = fgetc(fp);
		}

	fclose(fp);

	memset(gray_image, 0, width * height * sizeof(int));    // not really necessary

	// converting the input RGB bmp to grayscale image (not black and white)
	bmptogray_conversion(height, width, image, gray_image);

	// Edge Detection with Sobel  
	sobel_detection(height, width, gray_image, ee_image, (double)THRESHOLD);

	// Calculating the border pixels with replication
	border_pixel_calculation(height, width, ee_image);

	printf("The edges of the image have been detected with Sobel and a Threshold: %d\n", THRESHOLD);

	//-------------------------------------------------------------
	// Creating the final image in (pseudo)bmp format 
	//-------------------------------------------------------------
	char dst_name[80];  // Constructing output image name
	char str_T[3]; // Converting input threshold to string
	sprintf(str_T, "%d", THRESHOLD);

	strcpy(dst_name, "Output_Sobel_");
	strcat(dst_name, str_T);
	strcat(dst_name, ".bmp");

	// Writing new image
	wp = fopen(dst_name, "wb");
	// write the file and the bmp information to the file
	fwrite(&bmfh, 1, sizeof(BitmapFileHeader), wp);
	fwrite(&bmih, 1, sizeof(BitmapInfoHeader), wp);

	// write the Sobel annotations pixel-by-pixel
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			fputc(ee_image[x][y], wp);  // write the same value in all RGB channels 
			fputc(ee_image[x][y], wp);
			fputc(ee_image[x][y], wp);
		}

	fclose(wp);    // I am done here!!! 

	return 0;
}
