#include <stdio.h>
#include "rc522.h"

uint8_t 	i;
uint8_t 	j;
uint8_t 	b;
uint8_t 	q;
uint8_t 	en;
uint8_t	SN[MFRC522_MAX_LEN];
uint8_t	lastID[4];
uint8_t 	rxBuffer[8];
uint8_t	txBuffer[10] = "00000000\r";
uint8_t 	retstr[10];

unsigned char addr=0x08;
unsigned char RFID1[16]= {0x10,0x20,0x30,0x40,0x50,0x60,0xff,0x07,0x80,0x29,0x01,0x02,0x03,0x04,0x05,0x06};
unsigned char RFID2[16]= {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x07,0x80,0x29,0xff,0xff,0xff,0xff,0xff,0xff};

unsigned char 	R_DATA[16];

uint8_t KEY_A[6]= {0xff,0xff,0xff,0xff,0xff,0xff};
uint8_t KEY_B[6]= {0xff,0xff,0xff,0xff,0xff,0xff};

// char number to string hex (FF) (Only big letters!)
void char_to_hex(uint8_t data) {
	uint8_t digits[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	
	if (data < 16) {
		retstr[0] = '0';
		retstr[1] = digits[data];
	} else {
		retstr[0] = digits[(data & 0xF0)>>4];
		retstr[1] = digits[(data & 0x0F)];
	}
}

void convertUnCharToStr(char* str, unsigned char* UnChar, int ucLen)
{
    int i = 0;
    for (i = 0; i < ucLen; i++)
    {
        sprintf(str + i * 2, "%02x", UnChar[i]);
    }
}

void rc522_test() {
	if(!MFRC522_Request(PICC_REQALL, SN)) {
		if(!MFRC522_Anticoll(SN)){
			HAL_Delay(10);
			printf(">>>");
			j = 0;
			q = 0;
			b = 0;
			en = 1;

			for (i=0; i<4; i++) if (lastID[i] != SN[i]) j = 1;								// Repeat test
			
			if (j && en) {
				q = 0;
				en = 0;
				for (i=0; i<4; i++) lastID[i] = SN[i];
				for (i=0; i<4; i++) {
					char_to_hex(SN[i]);
					txBuffer[b] = retstr[0];
					b++;
					txBuffer[b] = retstr[1];
					b++;
					//ToStr(str[i]);
				}					
				printf("%s\n",txBuffer);
			}
			uint8_t status = MI_ERR;
			status = MFRC522_SelectTag(SN);
			if(status){
				status = MI_ERR;
				status = MFRC522_Auth(KEYA, 0x0B, KEY_A, SN);
				if(status==MI_OK) {
					printf("auth(A) success\r\n");
				} else {
					printf("auth(A) failed\r\n");
				}
				status = MFRC522_Auth(KEYB, 0x0B, KEY_B, SN);
				if(status==MI_OK) {
					printf("auth(B) success\r\n");
				} else {
					printf("auth(B) failed\r\n");
				}
				
				if(status==MI_OK) {
					status = MI_ERR;
					status = MFRC522_Read(addr, R_DATA);
					if(status == MI_OK) {
						for(i=0; i<16; i++) {
							printf("%02x", R_DATA[i]);
						}
						
						printf("\r\n");
						MFRC522_Halt();
					} else {
						printf("read failed\r\n");
					}
				}
			}
			MFRC522_Init();
			HAL_Delay(10);
		}
	}

}



