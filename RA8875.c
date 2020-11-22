/*
 * RA8875.c
 * Version 0.5
 * Created: 12.11.2019 15:12:02
 *  Author: M02875
 */ 

#include "RA8875.h"

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RA_PI	3.14159265358979323846

#define DEFAULT_ANGLE_OFFSET -90

#define EN_DELAY()	asm("nop");asm("nop");
#define _swap(a, b)	{ uint16_t t = a; a = b; b = t; }

#define DIROUTS(PORT,bits)	PORT.DIRSET = (bits)
#define DIRINS(PORT,bits)	PORT.DIRCLR = (bits)
#define CLRBITS(PORT,bits)	PORT.OUTCLR = (bits)	// Bit-löschen
#define SETBITS(PORT,bits)	PORT.OUTSET = (bits)	// Bit-setzten
#define TSTBITS(PORT,bits)	PORT.IN & (bits)		// Bit-lesen

enum RA8875_sizes _size;
enum RA8875_dispMode _textMode;
uint16_t _width;
uint16_t _height;
uint16_t _area_x0,_area_y0,_area_x1,_area_y1;
int16_t _angleOffset = DEFAULT_ANGLE_OFFSET;
uint8_t _touchEnabed = 0;
uint8_t _adcClk, _adcSample;
uint16_t _xLowCal = 0, _xHighCal = 0, _yLowCal = 0, _yHighCal = 0;

long RA8875_map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float _cosDegrees(float angle)
{
	return cos(angle * DEG_TO_RAD);
}

float _sinDegrees(float angle)
{
	return sin(angle * DEG_TO_RAD);
}

void RA8875_ioInit(void)
{
	CLRBITS(RA8875_CTRL_PORT,(1<<RA8875_WAIT) | (1<<RA8875_INT) | (1<<RA8875_EN));
	SETBITS(RA8875_CTRL_PORT, 1<<RA8875_CS);
	DIROUTS(RA8875_CTRL_PORT, (1<<RA8875_CS) | (1<<RA8875_EN) | (1<<RA8875_RW) | (1<<RA8875_RS));
	DIRINS(RA8875_CTRL_PORT, (1<<RA8875_WAIT) | (1<<RA8875_INT));
	RA8875_CTRL_PORT.PIN4CTRL |= PORT_PULLUPEN_bm;
	RA8875_CTRL_PORT.PIN5CTRL |= PORT_PULLUPEN_bm;
	
	RA8875_DATA_DIR = DATA_OUT;
	
	RA8875_DATA_PORT.PIN0CTRL = PORT_PULLUPEN_bm;
	RA8875_DATA_PORT.PIN1CTRL = PORT_PULLUPEN_bm;
	RA8875_DATA_PORT.PIN2CTRL = PORT_PULLUPEN_bm;
	RA8875_DATA_PORT.PIN3CTRL = PORT_PULLUPEN_bm;
	RA8875_DATA_PORT.PIN4CTRL = PORT_PULLUPEN_bm;
	RA8875_DATA_PORT.PIN5CTRL = PORT_PULLUPEN_bm;
	RA8875_DATA_PORT.PIN6CTRL = PORT_PULLUPEN_bm;
	RA8875_DATA_PORT.PIN7CTRL = PORT_PULLUPEN_bm;
}

void RA8875_PLL_init(void)
{
	switch (_size)
	{
		// To-Do: Weitere Displays hinzufügen
		case RA8875_800x480:
			// Werte vom Democode des Buydisplay Herstellers: https://www.buydisplay.com/download/democode/ER-TFTM070-5_8-bit-8080_DemoCode.txt
			RA8875_writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 | 0x0C);
			RA8875_writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
			break;
	}
}

void RA8875_init(enum RA8875_sizes dispSize)
{
	_size = dispSize;
#if INTERFACE_PARALLEL_8BIT || INTERFACE_PARALLEL_16BIT
	RA8875_ioInit();
#endif	
	uint8_t pixclk,hsync_start,hsync_pw,hsync_finetune,hsync_nondisp,vsync_pw,_voffset,pll_div1,pll_div2;
	uint16_t vsync_nondisp,vsync_start;
	
	switch (_size)
	{
		case RA8875_800x480:
			// Werte vom Democode des Buydisplay Herstellers: https://www.buydisplay.com/download/democode/ER-TFTM070-5_8-bit-8080_DemoCode.txt
			pixclk			= RA8875_PCSR_PLCK_FALL | RA8875_PCSR_PLCKPER_2;
			hsync_nondisp   = 26;
			hsync_start     = 32;
			hsync_pw        = 96;
			hsync_finetune  = 0;
			vsync_nondisp   = 33;
			vsync_start     = 23;
			vsync_pw        = 2;
			_voffset		= 0;
			_width			= 800;
			_height			= 480;
			_adcClk			= RA8875_TPCR0_ADCCLK_DIV16;
			_adcSample		= RA8875_TPCR0_SMPL_TIME_4096CLK;
			pll_div1		= RA8875_PLLC1_PLLDIV1 | 0x0C;
			pll_div2		= RA8875_PLLC2_DIV4;
			break;
		default:
			// Nicht unterstütztes display!
			return;
	}
	
	RA8875_writeReg(RA8875_PWRR, RA8875_PWRR_SWRESET);
	
	RA8875_writeReg(RA8875_PLLC1, pll_div1);
	RA8875_writeReg(RA8875_PLLC2, pll_div2);
	
	RA8875_writeReg(RA8875_SYSR, RA8875_SYSR_8BIT_IF | RA8875_SYSR_256_COLORS);
	
	RA8875_writeReg(RA8875_PCSR, pixclk);
	
	/* Horizontal settings register */
	RA8875_writeReg(RA8875_HDWR, (_width / 8) - 1);
	RA8875_writeReg(RA8875_HNDFTR,hsync_finetune);
	RA8875_writeReg(RA8875_HNDR, (hsync_nondisp - hsync_finetune -2) / 8);
	RA8875_writeReg(RA8875_HSTR, (hsync_start / 8) - 1);
	RA8875_writeReg(RA8875_HPWR, (hsync_pw / 8) - 1);
	
	/* Vertical settings register */
	RA8875_writeReg(RA8875_VDHR0, (uint16_t)(_height - 1 + _voffset) & 0xFF);
	RA8875_writeReg(RA8875_VDHR1, (uint16_t)(_height - 1 + _voffset) >> 8);
	RA8875_writeReg(RA8875_VNDR0, vsync_nondisp - 1);
	RA8875_writeReg(RA8875_VNDR1, vsync_nondisp >> 8);
	RA8875_writeReg(RA8875_VSTR0, vsync_start - 1);
	RA8875_writeReg(RA8875_VSTR1, vsync_start >> 8);
	RA8875_writeReg(RA8875_VPWR, vsync_pw - 1);
	
	RA8875_writeReg(RA8875_FNCR1, RA8875_FNCR1_FONT_TRANS);
	
	RA8875_resetActiveWindow();
}


void RA8875_writeCMD(uint8_t _cmd)
{
#if INTERFACE_PARALLEL_8BIT
	SETBITS(RA8875_CTRL_PORT, 1<<RA8875_RS);
	CLRBITS(RA8875_CTRL_PORT, (1<<RA8875_RW) | (1<<RA8875_CS));
	RA8875_DATA_OUT = _cmd;
	SETBITS(RA8875_CTRL_PORT, 1<<RA8875_EN);
	EN_DELAY();
	CLRBITS(RA8875_CTRL_PORT, 1<<RA8875_EN);
	SETBITS(RA8875_CTRL_PORT, 1<<RA8875_CS);
#endif
}

void RA8875_writeData(uint8_t data)
{
#if INTERFACE_PARALLEL_8BIT
	CLRBITS(RA8875_CTRL_PORT, (1<<RA8875_CS));
	EN_DELAY();
	while (!(TSTBITS(RA8875_CTRL_PORT, 1<<RA8875_WAIT)))
	{
		EN_DELAY();
	}
	CLRBITS(RA8875_CTRL_PORT, (1<<RA8875_RS) | (1<<RA8875_RW));
	RA8875_DATA_OUT = data;
	SETBITS(RA8875_CTRL_PORT, 1<<RA8875_EN);
	EN_DELAY();
	CLRBITS(RA8875_CTRL_PORT, 1<<RA8875_EN);
	SETBITS(RA8875_CTRL_PORT, 1<<RA8875_CS);
#endif
}

uint8_t RA8875_readData()
{
	uint8_t temp = 0;
#if INTERFACE_PARALLEL_8BIT
	RA8875_DATA_OUT = 0;
	RA8875_DATA_DIR = DATA_IN;
	SETBITS(RA8875_CTRL_PORT, (1<<RA8875_RW));
	CLRBITS(RA8875_CTRL_PORT, (1<<RA8875_RS) |(1<<RA8875_CS));
	SETBITS(RA8875_CTRL_PORT, (1<<RA8875_EN));
	EN_DELAY();
	temp = RA8875_DATA_IN;
	CLRBITS(RA8875_CTRL_PORT, (1<<RA8875_EN));
	SETBITS(RA8875_CTRL_PORT, (1<<RA8875_CS));
	RA8875_DATA_DIR = DATA_OUT;
#endif
	return temp;
}

void RA8875_writeReg(uint8_t _reg, uint8_t _val)
{
	RA8875_writeCMD(_reg);
	RA8875_writeData(_val);
}

uint8_t RA8875_readReg(uint8_t _reg)
{
	RA8875_writeCMD(_reg);
	return RA8875_readData();
}

enum RA8875_status RA8875_readStatus(void)
{
	uint8_t _status;
#if INTERFACE_PARALLEL_8BIT
	RA8875_DATA_OUT = 0;
	RA8875_DATA_DIR = DATA_IN;
	SETBITS(RA8875_CTRL_PORT, (1<<RA8875_RW) | (1<<RA8875_RS));
	CLRBITS(RA8875_CTRL_PORT, (1<<RA8875_CS));
	SETBITS(RA8875_CTRL_PORT, (1<<RA8875_EN));
	EN_DELAY();
	_status = RA8875_DATA_IN;
	CLRBITS(RA8875_CTRL_PORT, (1<<RA8875_EN));
	SETBITS(RA8875_CTRL_PORT, (1<<RA8875_CS));
	RA8875_DATA_DIR = DATA_OUT;
#endif
	return _status;
}

void RA8875_waitReg(uint8_t reg, uint8_t flag)
{
	while (RA8875_readReg(reg) & flag);
}

void RA8875_displayOnOff(uint8_t _onoff)
{
	RA8875_writeReg(RA8875_PWRR, (_onoff ? RA8875_PWRR_LCDON : RA8875_PWRR_LCDOFF));
}

void RA8875_setMode(enum RA8875_dispMode textMode)
{
	/* Activate Text Mode */
	uint8_t temp = RA8875_readReg(RA8875_MWCR0) & ~RA8875_MWCR0_TEXTMODE;
		
	RA8875_writeReg(RA8875_MWCR0, temp | ((textMode) ? RA8875_MWCR0_TEXTMODE : RA8875_MWCR0_GRAPHMODE));
	_textMode = textMode;
}

enum RA8875_dispMode RA8875_getMode(void)
{
	return _textMode;
}

void RA8875_clearMemory(uint8_t fbg)
{	
	RA8875_writeReg(RA8875_MCLR, RA8875_MCLR_FULLWINDOW | RA8875_MCLR_STARTCLEAR | ((fbg) ? 1 : 0));
	RA8875_waitReg(RA8875_MCLR, RA8875_MCLR_ISCLEARING);
}

void RA8875_brightness(uint8_t val)
{
	RA8875_PWMout(1,val);
}

void RA8875_setActiveWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	RA8875_writeReg(RA8875_HSAW0, x0);
	RA8875_writeReg(RA8875_HSAW1, x0 >> 8);
	RA8875_writeReg(RA8875_HEAW0, x1);
	RA8875_writeReg(RA8875_HEAW1, x1 >> 8);
	RA8875_writeReg(RA8875_VSAW0, y0);
	RA8875_writeReg(RA8875_VSAW1, y0 >> 8);
	RA8875_writeReg(RA8875_VEAW0, y1);
	RA8875_writeReg(RA8875_VEAW1, y1 >> 8);
	_area_x0 = x0;	_area_y0 = y0;
	_area_x1 = x1;	_area_y1 = y1;
}

void RA8875_resetActiveWindow(void)
{
	RA8875_setActiveWindow(0,0,_width-1,_height-1);
}

void RA8875_getActiveWindow(uint16_t *x0, uint16_t *y0, uint16_t *x1, uint16_t *y1)
{
	*x0 = _area_x0; *y0 = _area_y0;
	*x1 = _area_x1; *y1 = _area_y1;
}

void RA8875_clearActiveWindow(uint8_t fbg)
{
	RA8875_writeReg(RA8875_MCLR, RA8875_MCLR_ACTIVEWINDOW | RA8875_MCLR_STARTCLEAR | ((fbg) ? 1 : 0));
	RA8875_waitReg(RA8875_MCLR, RA8875_MCLR_ISCLEARING);
}

uint16_t RA8875_width(void)
{
	return _width;
}
uint16_t RA8875_height(void)
{
	return _height;
}


void RA8875_setFColor(uint8_t fgColor)
{	
	RA8875_writeReg(RA8875_FGCR0, (fgColor >> 5) & 0x7);
	RA8875_writeReg(RA8875_FGCR1, (fgColor >> 2) & 0x7);
	RA8875_writeReg(RA8875_FGCR2, fgColor & 0x3);
}

void RA8875_setBColor(uint8_t bgColor)
{	
	RA8875_writeReg(RA8875_BGCR0, (bgColor >> 5) & 0x7);
	RA8875_writeReg(RA8875_BGCR1, (bgColor >> 2) & 0x7);
	RA8875_writeReg(RA8875_BGCR2, bgColor & 0x3);	
}

void RA8875_setBTColor(uint8_t bgColor)
{
	RA8875_writeReg(RA8875_BGCR0, (bgColor >> 5) & 0x7);
	RA8875_writeReg(RA8875_BGCR1, (bgColor >> 2) & 0x7);
	RA8875_writeReg(RA8875_BGCR2, bgColor & 0x3);	
}

void RA8875_transparentOnOff(uint8_t _transOn)
{
	uint8_t temp = RA8875_readReg(RA8875_FNCR1);
	
	if (_transOn)
	{
		RA8875_writeReg(RA8875_FNCR1, temp | RA8875_FNCR1_FONT_TRANS);
	}
	else
	{
		RA8875_writeReg(RA8875_FNCR1, temp & ~RA8875_FNCR1_FONT_TRANS);
	}
}

void RA8875_setColor(uint8_t fgColor, uint8_t bgColor, uint8_t Transparent)
{
	RA8875_setFColor(fgColor);
	RA8875_transparentOnOff(Transparent);
	if (Transparent)
	{
		RA8875_setBTColor(bgColor);
	}
	else
	{
		RA8875_setBColor(bgColor);
	}
}

void RA8875_uploadCustomChar(const uint8_t symbol[], uint8_t adr)
{
	uint8_t _temp = RA8875_readReg(RA8875_MWCR1);
	if (_textMode)	RA8875_setMode(GRAPHMODE);
	
	RA8875_writeReg(RA8875_CGSR, adr);
	RA8875_writeReg(RA8875_MWCR1, RA8875_MWCR1_WRITECGRAM);
	RA8875_writeReg(RA8875_FNCR0, RA8875_readReg(RA8875_FNCR0) & 0x7F);
	
	RA8875_writeCMD(RA8875_MRWC);
	for (uint8_t i = 0; i < 16; i++)
	{
		RA8875_writeData(symbol[i]);
	}
	
	RA8875_writeReg(RA8875_MWCR1, _temp);
}

void RA8875_printCustomChar(uint8_t adr, uint8_t _additionalChars)
{
	if (!_textMode)	RA8875_setMode(TEXTMODE);
	
	uint8_t _FNCR0 = RA8875_readReg(RA8875_FNCR0);
	//uint8_t _FNCR1 = RA8875_readReg(RA8875_FNCR1);
	RA8875_writeReg(RA8875_FNCR0, _FNCR0 | RA8875_FNCR0_CGRAMFONT);
		
	RA8875_writeCMD(RA8875_MRWC);
	
	for (uint8_t i = 0; i <= _additionalChars; i++)
	{
		RA8875_writeData(adr + i);
	}
	
	RA8875_writeReg(RA8875_FNCR0, _FNCR0);
}

void RA8875_cursorBlink(uint8_t _blinkRate, uint8_t _blinkOnOff, enum RA8875_cursor cursorType)
{
	uint8_t temp, cW=0, cH=0;
	
	temp = RA8875_readReg(RA8875_MWCR0) & 0x3F;
	temp |= (cursorType ? RA8875_MWCR0_CURSOR_EN : 0) | (_blinkOnOff ? RA8875_MWCR0_BLINK_EN : 0);
	RA8875_writeReg(RA8875_MWCR0, temp);
	
	RA8875_writeReg(RA8875_BTCR, _blinkRate);
	
	switch (cursorType)
	{
		case IBEAM:
		cW = 0x01;
		cH = 0x1F;
		break;
		case UNDER:
		cW = 0x07;
		cH = 0x01;
		break;
		case BLOCK:
		cW = 0x07;
		cH = 0x1F;
		break;
		case NOCURSOR:
		default:
		break;
	}
	
	RA8875_writeReg(RA8875_CURHS, cW);
	RA8875_writeReg(RA8875_CURVS, cH);
}

void RA8875_setTextCursor(uint16_t x0, uint16_t y0)
{
	/* Set cursor location */
	RA8875_writeReg(RA8875_F_CURXL, x0 & 0xFF);
	RA8875_writeReg(RA8875_F_CURXH, x0 >> 8);
	RA8875_writeReg(RA8875_F_CURYL, y0 & 0xFF);
	RA8875_writeReg(RA8875_F_CURYH, y0 >> 8);
}

void RA8875_setTextSizeEnlargement(uint8_t horizontal, uint8_t vertical)
{
	/* Set font size flags */
	uint8_t temp = RA8875_readReg(RA8875_FNCR1);
	temp &= ~(0xF); // Clears bits 0..3
	temp |= ((horizontal & 0x3) << 2) | (vertical & 0x3);
	RA8875_writeReg(RA8875_FNCR1,temp);
}

void RA8875_setTextSize(enum RA8875_TextSize txtSize)
{
	/* Set font size flags */
	uint8_t temp = RA8875_readReg(RA8875_FNCR1);
	temp &= ~(0xF); // Clears bits 0..3
	temp |= (txtSize & 0xF);
	RA8875_writeReg(RA8875_FNCR1,temp);
}

void RA8875_setTextInterline(uint8_t pixels)
{
	if (pixels > 0x3F)	pixels = 0x3F;
	
	//RA8875_writeReg(RA8875_)
}

void RA8875_setTextSpacing(uint8_t pixels)
{
	
}

void RA8875_printText(const char *text, uint16_t textLength)
{
	if (!(_textMode))	RA8875_setMode(TEXTMODE);
	
	RA8875_writeCMD(RA8875_MRWC);
	
	for (uint16_t txtCnt = 0; txtCnt < textLength; txtCnt++)
	{
		RA8875_writeData(text[txtCnt]);
	}
}

void RA8875_char(char c)
{
	RA8875_printText((const char *)&c, 1);
}

void RA8875_print(char *text)
{
	if (!(_textMode))	RA8875_setMode(TEXTMODE);
	
	RA8875_writeCMD(RA8875_MRWC);
	
	while(*text)
	{
		RA8875_writeData(*text++);
	}
}

void RA8875_setXY(uint16_t x0, uint16_t y0)
{
	RA8875_writeReg(RA8875_CURH0, x0),
	RA8875_writeReg(RA8875_CURH1, x0 >> 8);
	RA8875_writeReg(RA8875_CURV0, y0);
	RA8875_writeReg(RA8875_CURV1, y0 >> 8);
}

void RA8875_drawPixel(uint16_t x0, uint16_t y0, uint8_t color)
{
	if (_textMode)	RA8875_setMode(GRAPHMODE);
	RA8875_setXY(x0,y0);
	RA8875_writeReg(RA8875_MRWC, color);
}

void RA8875_drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color)
{
	if (_textMode)	RA8875_setMode(GRAPHMODE);
	if ( (x0 == x1) && (y0 == y1))
	{
		RA8875_drawPixel(x0,y0,color);
		return;
	}
	
	RA8875_writeReg(RA8875_DLHSR0, x0);
	RA8875_writeReg(RA8875_DLHSR1, x0 >> 8);
	
	RA8875_writeReg(RA8875_DLVSR0, y0);
	RA8875_writeReg(RA8875_DLVSR1, y0 >> 8);
	
	RA8875_writeReg(RA8875_DLHER0, x1);
	RA8875_writeReg(RA8875_DLHER1, x1 >> 8);
	
	RA8875_writeReg(RA8875_DLVER0, y1);
	RA8875_writeReg(RA8875_DLVER1, y1 >> 8);
	
	RA8875_writeReg(RA8875_FGCR0, (color >> 5) & 0x7);
	RA8875_writeReg(RA8875_FGCR1, (color >> 2) & 0x7);
	RA8875_writeReg(RA8875_FGCR2, color & 0x3);
	
	RA8875_writeReg(RA8875_DCR, RA8875_DCR_STARTDRAWING);
	
	RA8875_waitReg(RA8875_DCR, RA8875_DCR_ISDRAWING);
}

void RA8875_drawLineAngle(uint16_t x0, uint16_t y0, float angle, uint16_t start, uint16_t length, uint16_t color)
{
	if (start)
	{
		RA8875_drawLine(
		x0 + (uint16_t)(start*_cosDegrees(angle + _angleOffset)),
		y0 + (uint16_t)(start*_sinDegrees(angle + _angleOffset)),
		x0 + (uint16_t)((start + length)*_cosDegrees(angle + _angleOffset)),
		y0 + (uint16_t)((start + length)*_sinDegrees(angle + _angleOffset)), color);
	}
	else
	{
		RA8875_drawLine(
		x0,
		y0,
		x0 + length*_cosDegrees(angle + _angleOffset),
		y0 + length*_sinDegrees(angle + _angleOffset), color);
	}
}

void RA8875_drawRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t color, uint8_t filled)
{
	if (_textMode)	RA8875_setMode(GRAPHMODE);
	
	w += x0;
	h += y0;
	
	RA8875_writeReg(RA8875_DLHSR0, x0);
	RA8875_writeReg(RA8875_DLHSR1, x0 >> 8);
	
	RA8875_writeReg(RA8875_DLVSR0, y0);
	RA8875_writeReg(RA8875_DLVSR1, y0 >> 8);
	
	RA8875_writeReg(RA8875_DLHER0, w);
	RA8875_writeReg(RA8875_DLHER1, w >> 8);
	
	RA8875_writeReg(RA8875_DLVER0, h);
	RA8875_writeReg(RA8875_DLVER1, h >> 8);
		
	RA8875_writeReg(RA8875_FGCR0, (color >> 5) & 0x7);
	RA8875_writeReg(RA8875_FGCR1, (color >> 2) & 0x7);
	RA8875_writeReg(RA8875_FGCR2, color & 0x3);
	
	if (filled)
	{
		RA8875_writeReg(RA8875_DCR, RA8875_DCR_STARTDRAWING | RA8875_DCR_FILL | RA8875_DCR_DRAWSQUARE);
	}
	else
	{
		RA8875_writeReg(RA8875_DCR, RA8875_DCR_STARTDRAWING | RA8875_DCR_DRAWSQUARE);
	}
	
	RA8875_waitReg(RA8875_DCR, RA8875_DCR_ISDRAWING);
}

void RA8875_drawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint8_t color, uint8_t filled)
{
	if (_textMode)	RA8875_setMode(GRAPHMODE);
	
	RA8875_writeReg(RA8875_DCHR0, x0);
	RA8875_writeReg(RA8875_DCHR1, x0 >> 8);
	
	RA8875_writeReg(RA8875_DCVR0, y0);
	RA8875_writeReg(RA8875_DCVR1, y0 >> 8);
	
	RA8875_writeReg(RA8875_DCRR, r);
	
	RA8875_writeReg(RA8875_FGCR0, (color >> 5) & 0x7);
	RA8875_writeReg(RA8875_FGCR1, (color >> 2) & 0x7);
	RA8875_writeReg(RA8875_FGCR2, color & 0x3);
	
	if (filled)
	{
		RA8875_writeReg(RA8875_DCR, RA8875_DCR_STARTCIRCLE | RA8875_DCR_FILL);
	}
	else
	{
		RA8875_writeReg(RA8875_DCR, RA8875_DCR_STARTCIRCLE);
	}
	
	RA8875_waitReg(RA8875_DCR, RA8875_DCR_ISDRAWCIRCLE);
}

void RA8875_drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint16_t filled)
{
	if (_textMode)	RA8875_setMode(GRAPHMODE);
	
	// (DLHER, DLVER) (DLHSR, DLVSR) (DTPH, DTPV)
	
	RA8875_writeReg(RA8875_DLHER0, x0);
	RA8875_writeReg(RA8875_DLHER1, x0 >> 8);
	
	RA8875_writeReg(RA8875_DLVER0, y0);
	RA8875_writeReg(RA8875_DLVER1, y0 >> 8);
	
	RA8875_writeReg(RA8875_DLHSR0, x1);
	RA8875_writeReg(RA8875_DLHSR1, x1 >> 8);
	
	RA8875_writeReg(RA8875_DLVSR0, y1);
	RA8875_writeReg(RA8875_DLVSR1, y1 >> 8);
	
	RA8875_writeReg(RA8875_DTPH0, x2);
	RA8875_writeReg(RA8875_DTPH1, x2 >> 8);
	
	RA8875_writeReg(RA8875_DTPV0, y2);
	RA8875_writeReg(RA8875_DTPV1, y2 >> 8);
	
	RA8875_writeReg(RA8875_FGCR0, (color >> 5) & 0x7);
	RA8875_writeReg(RA8875_FGCR1, (color >> 2) & 0x7);
	RA8875_writeReg(RA8875_FGCR2, color & 0x3);
	
	if (filled)
	{
		RA8875_writeReg(RA8875_DCR, RA8875_DCR_STARTDRAWING | RA8875_DCR_DRAWTRIANGLE | RA8875_DCR_FILL);
	}
	else
	{
		RA8875_writeReg(RA8875_DCR, RA8875_DCR_STARTDRAWING | RA8875_DCR_DRAWTRIANGLE);
	}
	
	RA8875_waitReg(RA8875_DCR, RA8875_DCR_ISDRAWING);
}

void RA8875_drawEllipse(uint16_t x0, uint16_t y0, uint16_t r_w, uint16_t r_h, uint8_t color, uint8_t filled)
{
	if (_textMode)	RA8875_setMode(GRAPHMODE);
	
	// (ELL_B H, ELL_A W) (DEHR, DEVR)
	
	RA8875_writeReg(RA8875_DEHR0, x0);
	RA8875_writeReg(RA8875_DEHR1, x0 >> 8);
	
	RA8875_writeReg(RA8875_DEVR0, y0);
	RA8875_writeReg(RA8875_DEVR1, y0 >> 8);
	
	RA8875_writeReg(RA8875_ELL_A0, r_w);
	RA8875_writeReg(RA8875_ELL_A1, r_w >> 8);
	
	RA8875_writeReg(RA8875_ELL_B0, r_h);
	RA8875_writeReg(RA8875_ELL_B1, r_h >> 8);
	
	RA8875_writeReg(RA8875_FGCR0, (color >> 5) & 0x7);
	RA8875_writeReg(RA8875_FGCR1, (color >> 2) & 0x7);
	RA8875_writeReg(RA8875_FGCR2, color & 0x3);
	
	if (filled)
	{
		RA8875_writeReg(RA8875_DRE, RA8875_DCR_STARTDRAWING | RA8875_DRE_ELLIPSE | RA8875_DRE_FILL);
	}
	else
	{
		RA8875_writeReg(RA8875_DRE, RA8875_DCR_STARTDRAWING | RA8875_DRE_ELLIPSE);
	}
	
	RA8875_waitReg(RA8875_DRE, RA8875_DRE_ISDRAWING);
}

void RA8875_drawRoundedRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t r_w, uint16_t r_h, uint8_t color, uint8_t filled)
{
	if (_textMode)	RA8875_setMode(GRAPHMODE);
	
	w += x0;
	h += y0;
	// (ELL_B H, ELL_A W) (DLHSR, DLVSR) (DLHER, DLVER)
	
	RA8875_writeReg(RA8875_DLHSR0, x0);
	RA8875_writeReg(RA8875_DLHSR1, x0 >> 8);
	
	RA8875_writeReg(RA8875_DLVSR0, y0);
	RA8875_writeReg(RA8875_DLVSR1, y0 >> 8);
	
	RA8875_writeReg(RA8875_DLHER0, w);
	RA8875_writeReg(RA8875_DLHER1, w >> 8);
	
	RA8875_writeReg(RA8875_DLVER0, h);
	RA8875_writeReg(RA8875_DLVER1, h >> 8);
	
	RA8875_writeReg(RA8875_ELL_A0, r_w);
	RA8875_writeReg(RA8875_ELL_A1, r_w >> 8);
	
	RA8875_writeReg(RA8875_ELL_B0, r_h);
	RA8875_writeReg(RA8875_ELL_B1, r_h >> 8);
	
	RA8875_writeReg(RA8875_FGCR0, (color >> 5) & 0x7);
	RA8875_writeReg(RA8875_FGCR1, (color >> 2) & 0x7);
	RA8875_writeReg(RA8875_FGCR2, color & 0x3);
	
	if (filled)
	{
		RA8875_writeReg(RA8875_DRE, RA8875_DCR_STARTDRAWING | RA8875_DRE_ROUNDEDRECT | RA8875_DRE_FILL);
	}
	else
	{
		RA8875_writeReg(RA8875_DRE, RA8875_DCR_STARTDRAWING | RA8875_DRE_ROUNDEDRECT);
	}
	
	RA8875_waitReg(RA8875_DRE, RA8875_DRE_ISDRAWING);
}

void RA8875_drawXBitmap(uint16_t x0, uint16_t y0, const uint8_t bitmap[], uint16_t w, uint16_t h, uint8_t fgcolor, uint8_t transparent, uint8_t bgColor)
{
	int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	uint8_t byte = 0;

	for (int16_t j = 0; j < h; j++, y0++)
	{
		for (int16_t i = 0; i < w; i++)
		{
			if (i & 7)
			{
				byte >>= 1;
			}
			else
			{
				byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
			}
			
			if (byte & 0x01)
			{
				RA8875_drawPixel(x0 + i, y0, fgcolor);
			}
			else
			{
				if (!(transparent))
				{
					RA8875_drawPixel(x0 + i, y0, bgColor);
				}
			}
		}
	}
}

uint8_t RA8875_Touched(void)
{
	if (!_touchEnabed)	return 0;	
	if (RA8875_readReg(RA8875_INTC2) & RA8875_INTC2_TOUCH_INTF)	return 1;
	return 0;
}

void RA8875_touchEnable(uint8_t _enable)
{
	if (_enable)
	{
		// Touch-Panel einschalten
		RA8875_writeReg(RA8875_TPCR0, RA8875_TPCR0_TOUCH_ENABLE | _adcSample | _adcClk);
		// Automodus einstellen
		RA8875_writeReg(RA8875_TPCR1, RA8875_TPCR1_AUTOMODE | RA8875_TPCR1_DEBOUNCE_EN);
		// Interrupt einschalten (? Muss wohl auch für's Polling eingschaltet)
		RA8875_writeReg(RA8875_INTC1, RA8875_readReg(RA8875_INTC1) | RA8875_INTC1_EN_TOUCH_INT);
		_touchEnabed = 1;
	}
	else
	{
		// Interrupt ausschalten
		RA8875_writeReg(RA8875_INTC1, RA8875_readReg(RA8875_INTC1) & ~RA8875_INTC1_EN_TOUCH_INT);
		// Touch-Display ausschalten
		RA8875_writeReg(RA8875_TPCR0, RA8875_TPCR0_TOUCH_DISABLE);
		_touchEnabed = 0;
	}
}

RA8875_point RA8875_touchReadAdc(void)
{
	RA8875_point touchADC = {0,0};
	uint8_t temp;
	
	if (!_touchEnabed)	return touchADC;
	
	touchADC.x = RA8875_readReg(RA8875_TPXH) << 2;
	touchADC.y = RA8875_readReg(RA8875_TPYH) << 2;
	temp = RA8875_readReg(RA8875_TPXYL) & (RA8875_TPXYL_X_MASK | RA8875_TPXYL_Y_MASK);
	
	touchADC.x |= temp & RA8875_TPXYL_X_MASK;
	touchADC.y |= (temp & RA8875_TPXYL_Y_MASK) >> 2;
	
	RA8875_writeReg(RA8875_INTC2, RA8875_INTC2_TOUCH_INTF);	
	
	return touchADC;
}

RA8875_point RA8875_touchReadPixel(void)
{
	RA8875_point touchPixel = RA8875_touchReadAdc();
	
	if (!_touchEnabed)	return touchPixel;
	
	touchPixel.x = RA8875_map(touchPixel.x, _xLowCal, _xHighCal,0,_width);
	touchPixel.y = RA8875_map(touchPixel.y, _yLowCal, _yHighCal,0,_height);
	
	return touchPixel;
}

uint8_t RA8875_TouchNotCalibrated(void)
{
	if ( (_xLowCal == 0) && (_xHighCal == 0) && (_yLowCal == 0) && (_yHighCal == 0) )	return 1;
	return 0;
}

void RA8875_touchLoadCalibration(uint16_t x_low, uint16_t x_high, uint16_t y_low, uint16_t y_high)
{
	_xLowCal = x_low;	_xHighCal = x_high;
	_yLowCal = y_low;	_yHighCal = y_high;
}

uint8_t RA8875_touchFiltered(RA8875_touch *touchPointer)
{
	RA8875_point temp;
	
	if ((!_touchEnabed) || (touchPointer->maxSamples <= 5)	|| (touchPointer->touchTolerance <= 1))
		return 0;
	
	if (RA8875_Touched())
	{
		temp = RA8875_touchReadPixel();
		
		if (touchPointer->samplesCounter > 0)
		{
			// Oh boi, here we go
			if ( ((touchPointer->averageX + touchPointer->touchTolerance) > temp.x) \
			&& ((touchPointer->averageX - touchPointer->touchTolerance) < temp.x)   \
			&& ((touchPointer->averageY + touchPointer->touchTolerance) > temp.y)   \
			&& ((touchPointer->averageY - touchPointer->touchTolerance) < temp.y) )
			{
				touchPointer->sampledTouchX += temp.x;
				touchPointer->sampledTouchY += temp.y;
				
				touchPointer->samplesCounter++;
				
								
				touchPointer->averageX = touchPointer->sampledTouchX / touchPointer->samplesCounter;
				touchPointer->averageY = touchPointer->sampledTouchY / touchPointer->samplesCounter;
					
				
				if (touchPointer->samplesCounter >= touchPointer->maxSamples)
				{
					touchPointer->filteredTouch.x = touchPointer->averageX;
					touchPointer->filteredTouch.y = touchPointer->averageY;
					touchPointer->touched = 1;
					touchPointer->samplesCounter = 0;
				}
			}
		}
		else
		{
			touchPointer->averageX = touchPointer->sampledTouchX = temp.x;
			touchPointer->averageY = touchPointer->sampledTouchY = temp.y;
			touchPointer->samplesCounter++;
		}
	}
	else
	{
		touchPointer->samplesCounter = 0;
		touchPointer->touched = 0;
	}
	return touchPointer->touched;
}

void RA8875_GPIO(uint8_t on)
{
	RA8875_writeReg(RA8875_GPIOX, on);
}

void RA8875_PWMinit(uint8_t pwmOutput, enum RA8875_pwmDiv clkDiv, uint8_t onOff)
{
	uint8_t pwmReg = RA8875_P1CR;
	if (pwmOutput > 1)	pwmReg = RA8875_P2CR;
	
	if (onOff)
	{
		RA8875_writeReg(pwmReg, RA8875_P1CR_PWM1EN | (clkDiv & 0x0F));
	}
	else
	{
		RA8875_writeReg(pwmReg, 0);
	}
}

void RA8875_PWMout(uint8_t pwmOutput, uint8_t pwmValue)
{
	if (pwmOutput > 1)	// PWM Output 2
	{
		RA8875_writeReg(RA8875_P2DCR, pwmValue);
	}
	else // PWM Output 1
	{
		RA8875_writeReg(RA8875_P1DCR, pwmValue);		
	}
}