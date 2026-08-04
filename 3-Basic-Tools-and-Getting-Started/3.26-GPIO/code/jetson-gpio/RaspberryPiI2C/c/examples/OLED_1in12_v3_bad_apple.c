/*****************************************************************************
* | File      	:   OLED_1in12_v3_test.c
* | Author      :   Waveshare team
* | Function    :   1.3inch OLED Module (C) test demo
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2020-06-17
* | Info        :
* -----------------------------------------------------------------------------
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "test.h"
#include "OLED_1in12_v3.h"

int OLED_1in12_v3_bad_apple(void)
{
	printf("1.12inch OLED (C) Bad Apple demo\n");
	if(DEV_ModuleInit() != 0) {
		return -1;
	}
	  
	printf("OLED Init...\r\n");
	OLED_1in12_v3_Init();
	DEV_Delay_ms(500);	
	// 0.Create a new image cache
	UBYTE *BlackImage;
	UWORD Imagesize = ((OLED_1in12_v3_WIDTH%8==0)? (OLED_1in12_v3_WIDTH/8): (OLED_1in12_v3_WIDTH/8+1)) * OLED_1in12_v3_HEIGHT;
	if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
			printf("Failed to apply for black memory...\r\n");
			return -1;
	}
	printf("Paint_NewImage\r\n");
	Paint_NewImage(BlackImage, OLED_1in12_v3_WIDTH, OLED_1in12_v3_HEIGHT, 180, BLACK);	
	
	printf("Drawing\r\n");
	//1.Select Image
	Paint_SelectImage(BlackImage);
	DEV_Delay_ms(500);
	Paint_Clear(BLACK);
	while(1) {
		// Drawing on the image
		for(int frameNum = 1; frameNum < 6575; frameNum++){
			printf("Drawing: %d.bmp\r\n", frameNum);
			char *firstName = "./frame128/";
			char *lastName = ".bmp";
			char frameNumString[5] = {0};
			sprintf(frameNumString, "%d", frameNum);
			char *frameDir = (char *) malloc(strlen(firstName) + strlen(frameNumString) + strlen(lastName));
			strcpy(frameDir, firstName);
			strcat(frameDir, frameNumString);
			strcat(frameDir, lastName);
			printf("%s\r\n", frameDir);
			GUI_ReadBmp(frameDir, 0, 0);
			OLED_1in12_v3_Display(BlackImage);
			DEV_Delay_ms(24);	//for SPI
			//DEV_Delay_ms(1);	//for I2C
			Paint_Clear(BLACK);
		}	
	}
}

