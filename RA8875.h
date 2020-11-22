/*
 * RA8875.h
 * RA8875 Library for Atmel (Microchip) AVR Micros
 * Version 0.5
 * Todo:
 * - SPI Interface
 * - I2C Interface
 * - 16bit Interface
 * - Frame switching
 * Created: 12.11.2019 15:12:15
 *  Author: M02875
 */ 


#ifndef RA8875_H_
#define RA8875_H_

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <math.h>

#define INTERFACE_PARALLEL_8BIT		1
//#define INTERFACE_PARALLEL_16BIT	1
//#define INTERFACE_SPI				1
//#define INTERFACE_I2C				1
#define RA8875_USETOUCHSCREEN		1

#if INTERFACE_PARALLEL_8BIT || INTERFACE_PARALLEL_16BIT
#define RA8875_CS			PIN0_bp
#define	RA8875_EN			PIN1_bp		// RD
#define RA8875_RW			PIN2_bp
#define RA8875_RS			PIN3_bp		// A0
#define RA8875_WAIT			PIN4_bp
#define RA8875_INT			PIN5_bp
#define RA8875_CTRL_PORT	PORTD

#define RA8875_DATA_PORT	PORTC
#define RA8875_DATA_OUT		PORTC.OUT
#define RA8875_DATA_DIR		PORTC.DIR
#define RA8875_DATA_IN		PORTC.IN
#define DATA_OUT			0xFF
#define DATA_IN				0x00
#endif

#if INTERFACE_PARALLEL_16BIT
// Nicht implementiert
#endif

#if INTERFACE_SPI
// Maximal 10MHz!
// Nicht implementiert
#endif 

#if INTERFACE_I2C
// Nicht implementiert
#endif

#if (INTERFACE_PARALLEL_8BIT && INTERFACE_PARALLEL_16BIT) || (INTERFACE_PARALLEL_8BIT && INTERFACE_SPI) || (INTERFACE_PARALLEL_8BIT && INTERFACE_I2C) || (INTERFACE_PARALLEL_16BIT && INTERFACE_I2C) || (INTERFACE_PARALLEL_16BIT && INTERFACE_SPI) || (INTERFACE_I2C && INTERFACE_SPI)
#error "Es koennen nicht mehrere Schnittstellen gleichzeitig ausgewaehlt sein!"
#endif

// aktuell nur 800x480 Displays implementiert
enum RA8875_sizes {RA8875_800x480};

enum RA8875_status {
	RA8875_SFROM_BUSY	= 0x1,
	RA8875_SLEEPMODE	= 0x10,
	RA8875_TOUCHDETECTED = 0x20,
	RA8875_BTEBUSY		= 0x40,
	RA8875_MEMBUSY		= 0x80	
};

enum RA8875_cursor {NOCURSOR=0,IBEAM,UNDER,BLOCK};

enum RA8875_dispMode {GRAPHMODE=0,TEXTMODE=1};

enum RA8875_TextSize {
	SIZE_08X16 = 0x00,
	SIZE_16X32 = 0x05,
	SIZE_32X64 = 0x0A,
	SIZE_64X128 = 0x0F
};

enum RA8875_pwmDiv {
	CLKDIV_1 = 0,
	CLKDIV_2 = 1,
	CLKDIV_4 = 2,
	CLKDIV_8 = 3,
	CLKDIV_16 = 4,
	CLKDIV_32 = 5,
	CLKDIV_64 = 6,
	CLKDIV_128 = 7,
	CLKDIV_256 = 8,
	CLKDIV_512 = 9,
	CLKDIV_1024 = 10,
	CLKDIV_2048 = 11,
	CLKDIV_4096 = 12,
	CLKDIV_8192 = 13,
	CLKDIV_16384 = 14,
	CLKDIV_32768 = 15
};

typedef struct {
	int16_t x;
	int16_t y;
}RA8875_point;

typedef struct {
	uint16_t maxSamples;	// Anzahl Messungen, muss mindestens 5 sein. Muss vom Benutzer gesetzt werden
	int16_t touchTolerance;	// Toleranz in Pixel, muss mindestens 1 sein. Muss vom Benutzer gesetzt werden
	uint8_t touched:1;		// Gibt an, ob der Touchscreen gedrückt ist und die Koordinaten gültig sind
	RA8875_point filteredTouch; // Die gefilterten Koordinaten in Pixel
	int32_t sampledTouchX;
	int32_t sampledTouchY;
	uint16_t averageX;
	uint16_t averageY;
	uint16_t samplesCounter;
}RA8875_touch;

//-------------- Initialisierung ---------------------------------------------------------
void RA8875_init(enum RA8875_sizes dispSize);	// Display und die I/Os initialisieren
//-------------- LOW LEVEL ---------------------------------------------------------------
void RA8875_writeCMD(uint8_t _cmd);				// Register/Befehl senden
void RA8875_writeData(uint8_t data);			// Daten senden
uint8_t RA8875_readData();						// Daten lesen
//-------------- HARDWARE ----------------------------------------------------------------
void RA8875_writeReg(uint8_t _reg, uint8_t _val);// Befehl/Register und Daten senden
uint8_t RA8875_readReg(uint8_t _reg);			// Daten aus Register auslesen
enum RA8875_status RA8875_readStatus(void);		// Statusregister lesen
void RA8875_waitReg(uint8_t reg, uint8_t flag);
void RA8875_displayOnOff(uint8_t _onoff);		// Display ein / ausschalten
void RA8875_setMode(enum RA8875_dispMode textMode);// Text oder Grafikmodus
enum RA8875_dispMode RA8875_getMode(void);		// Aktueller Modus zurückgeben
void RA8875_clearMemory(uint8_t fbg);			// Displayspeicher löschen. 
//-------------- AREA ----------------------------------------------------------------
void RA8875_setActiveWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);	// Zeichenfläche auswählen 
void RA8875_resetActiveWindow(void);			// Das gesamte Display auswählen
void RA8875_getActiveWindow(uint16_t *x0, uint16_t *y0, uint16_t *x1, uint16_t *y1);				// Gibt die aktuell ausgewählte Zeichenfläche zurück.
void RA8875_clearActiveWindow(uint8_t fbg);		// Zeichenfläche löschen
uint16_t RA8875_width(void);					// Displaybreite in Pixels
uint16_t RA8875_height(void);					// Displayhöhe in Pixels
//-------------- COLORS ---------------------------------------------------------------
void RA8875_setFColor(uint8_t fgColor);
void RA8875_setBColor(uint8_t bgColor);
void RA8875_setBTColor(uint8_t bgColor);
void RA8875_transparentOnOff(uint8_t _transOn);
void RA8875_setColor(uint8_t fgColor, uint8_t bgColor, uint8_t Transparent);	// (Text)Farben
//-------------- TEXT ----------------------------------------------------------------
void RA8875_uploadCustomChar(const uint8_t symbol[], uint8_t adr);	// Eigenes Symbol (8bit x 16, Addresse 0-255)
void RA8875_printCustomChar(uint8_t _adr, uint8_t _additionalChars);// Eigenes Symbol anzeigen. Bei breiten Symbolen, Anzahl zusätzliche Symbol angeben
void RA8875_cursorBlink(uint8_t _blinkRate, uint8_t _blinkOnOff, enum RA8875_cursor cursorType); // Cursor ein/ausschalten und konfigurieren
void RA8875_setTextCursor(uint16_t x0, uint16_t y0);				// TextCursor Position
void RA8875_setTextSizeEnlargement(uint8_t horizontal, uint8_t vertical);	// Multiplikator (0 = x1, 3 = x4)
void RA8875_setTextSize(enum RA8875_TextSize txtSize);
void RA8875_setTextInterline(uint8_t pixels);	// untested
void RA8875_setTextSpacing(uint8_t pixels);		// untested
void RA8875_printText(const char *text, uint16_t textLength);
void RA8875_char(char c);
void RA8875_print(char *text);
//-------------- Grafik ----------------------------------------------------------------
void RA8875_setXY(uint16_t x0, uint16_t y0);						// X und Y Position setzten
void RA8875_drawPixel(uint16_t x0, uint16_t y0, uint8_t color);		// Pixel an spezifischer Position zeichnen
void RA8875_drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color); // Eine Linie zeichnen
void RA8875_drawLineAngle(uint16_t x0, uint16_t y0, float angle, uint16_t start, uint16_t length, uint16_t color);
void RA8875_drawRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t color, uint8_t filled); // Ein Rechteck zeichnen
void RA8875_fillRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t color); // Ein gefülltes Rechteck zeichnen
void RA8875_drawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint8_t color, uint8_t filled); // Ein Kreis zeichnen
void RA8875_drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint16_t filled); // Ein Dreieck zeichnen
void RA8875_drawEllipse(uint16_t x0, uint16_t y0, uint16_t r_w, uint16_t r_h, uint8_t color, uint8_t filled);			// untested
void RA8875_drawRoundedRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t r_w, uint16_t r_h, uint8_t color, uint8_t filled);
void RA8875_drawXBitmap(uint16_t x0, uint16_t y0, const uint8_t bitmap[], uint16_t w, uint16_t h, uint8_t fgcolor, uint8_t transparent, uint8_t bgColor);
//-------------- Touchpanel ----------------------------------------------------------------
uint8_t RA8875_Touched(void);
void RA8875_touchEnable(uint8_t _enable);
RA8875_point RA8875_touchReadAdc(void);
RA8875_point RA8875_touchReadPixel(void);
uint8_t RA8875_TouchNotCalibrated(void);
void RA8875_touchLoadCalibration(uint16_t x_low, uint16_t x_high, uint16_t y_low, uint16_t y_high);
uint8_t RA8875_touchFiltered(RA8875_touch *touchPointer);
//-------------- GPIO & PWM ----------------------------------------------------------------
void RA8875_GPIO(uint8_t on);
void RA8875_PWMinit(uint8_t pwmOutput, enum RA8875_pwmDiv clkDiv, uint8_t onOff); // PWM 1 ist die Hintergrundbeleuchtung, CLKdiv von 1024 bei 20MHz empfohlen
void RA8875_PWMout(uint8_t pwmOutput, uint8_t pwmValue); // PWM-Wert von 0 bis 255 für PWM Ausgang 1 oder 2.

/*
 * 5-2 System & Configuration Registers
 */
#define RA8875_PWRR				0x01
#define RA8875_PWRR_SWRESET		0x01
#define RA8875_PWRR_SLEEPMODE	0x02
#define RA8875_PWRR_LCDON		0x80
#define RA8875_PWRR_LCDOFF		0x80

#define RA8875_MRWC				0x02

#define RA8875_PCSR				0x04
#define RA8875_PCSR_PLCKPERIOD	0x03
#define RA8875_PCSR_PLCKPER_0	0x00
#define RA8875_PCSR_PLCKPER_2	0x01
#define RA8875_PCSR_PLCKPER_4	0x02
#define RA8875_PCSR_PLCKPER_8	0x03
#define RA8875_PCSR_PLCK_INV	0x80
#define RA8875_PCSR_PLCK_FALL	0x80

#define RA8875_SYSR				0x10
#define RA8875_SYSR_MCUIF		0x03
#define RA8875_SYSR_8BIT_IF		0x00
#define RA8875_SYSR_16BIT_IF	0x02
#define RA8875_SYSR_COLORDEPTH	0x0C
#define RA8875_SYSR_256_COLORS	0x00
#define RA8875_SYSR_65K_COLORS	0x08

#define RA8875_HDWR				0x14
#define RA8875_HNDFTR			0x15
#define RA8875_HNDR				0x16
#define RA8875_HSTR				0x17
#define RA8875_HPWR				0x18
#define RA8875_VDHR0			0x19
#define RA8875_VDHR1			0x1A
#define RA8875_VNDR0			0x1B
#define RA8875_VNDR1			0x1C
#define RA8875_VSTR0			0x1D
#define RA8875_VSTR1			0x1E
#define RA8875_VPWR				0x1F

/*
 * 5-3 LCD Display Control Registers
 */
#define RA8875_DPCR				0x20
#define RA8875_DPCR_TWO_LAYERS	0x80
#define RA8875_DPCR_HDIR_SEGn	0x08
#define RA8875_DPCR_VDIR_COMn	0x04

#define RA8875_FNCR0			0x21
#define RA8875_FNCR0_CGRAMFONT	0x80
#define RA8875_FNCR0_EXCGROM	0x20
#define RA8875_FNCR0_ISO8859_1	0x00
#define RA8875_FNCR0_ISO8859_2	0x01
#define RA8875_FNCR0_ISO8859_3	0x02
#define RA8875_FNCR0_ISO8859_4	0x03

#define RA8875_FNCR1			0x22
#define RA8875_FNCR1_FULL_ALIG	0x80
#define RA8875_FNCR1_FONT_TRANS	0x40
#define RA8875_FNCR1_ROT_90		0x10
#define RA8875_FNCR1_HOR_SIZE_1	0x00
#define RA8875_FNCR1_HOR_SIZE_2	0x04
#define RA8875_FNCR1_HOR_SIZE_3 0x08
#define RA8875_FNCR1_HOR_SIZE_4 0x0C
#define RA8875_FNCR1_VER_SIZE_1	0x00
#define RA8875_FNCR1_VER_SIZE_2	0x01
#define RA8875_FNCR1_VER_SIZE_3 0x02
#define RA8875_FNCR1_VER_SIZE_4 0x03

#define RA8875_CGSR				0x23

#define RA8875_F_CURXL			0x2A
#define RA8875_F_CURXH			0x2B
#define RA8875_F_CURYL			0x2C
#define RA8875_F_CURYH			0x2D

/*
 * 5-4 Active Window & Scroll Window Setting Registers
 */
#define RA8875_HSAW0			0x30
#define RA8875_HSAW1			0x31
#define RA8875_VSAW0			0x32
#define RA8875_VSAW1			0x33
#define RA8875_HEAW0			0x34
#define RA8875_HEAW1			0x35
#define RA8875_VEAW0			0x36
#define RA8875_VEAW1			0x37
#define RA8875_HSSW0			0x38
#define RA8875_HSSW1			0x39
#define RA8875_VSSW0			0x3A
#define RA8875_VSSW1			0x3B
#define RA8875_HESW0			0x3C
#define RA8875_HESW1			0x3D
#define RA8875_VESW0			0x3E
#define RA8875_VESW1			0x3F

/*
 * 5-5 Cursor Setting Registers
 */
#define RA8875_MWCR0			0x40
#define RA8875_MWCR0_TEXTMODE	0x80
#define RA8875_MWCR0_GRAPHMODE	0x00
#define RA8875_MWCR0_CURSOR_EN	0x40
#define RA8875_MWCR0_BLINK_EN	0x20

#define RA8875_MWCR1				0x41
#define RA8875_MWCR1_GR_CURSOR		0x80
#define RA8875_MWCR1_GR_CUR_SET_1	0x00
#define RA8875_MWCR1_GR_CUR_SET_2	0x10
#define RA8875_MWCR1_GR_CUR_SET_3	0x20
#define RA8875_MWCR1_GR_CUR_SET_4	0x30
#define RA8875_MWCR1_GR_CUR_SET_5	0x40
#define RA8875_MWCR1_GR_CUR_SET_6	0x50
#define RA8875_MWCR1_GR_CUR_SET_7	0x60
#define RA8875_MWCR1_GR_CUR_SET_8	0x70
#define RA8875_MWCR1_WRITELAYER		0x00
#define RA8875_MWCR1_WRITECGRAM		0x04
#define RA8875_MWCR1_WRITECURSOR	0x08
#define RA8875_MWCR1_WRITEPATTERN	0x0C
#define RA8875_MWCR1_USE_LAYER_1	0x00
#define RA8875_MWCR1_USE_LAYER_2	0x01

#define RA8875_BTCR				0x44
#define RA8875_MRCD				0x45
#define RA8875_CURH0			0x46
#define RA8875_CURH1			0x47
#define RA8875_CURV0			0x48
#define RA8875_CURV1			0x49
#define RA8875_RCURH0			0x4A
#define RA8875_RCURH1			0x4B
#define RA8875_RCURV0			0x4C
#define RA8875_RCURV1			0x4D
#define RA8875_CURHS			0x4E
#define RA8875_CURVS			0x4F

/*
 * 5-6 Block Transfer Engine (BTE) Control Registers
 */


#define RA8875_BGCR0			0x60
#define RA8875_BGCR1			0x61
#define RA8875_BGCR2			0x62
#define RA8875_FGCR0			0x63
#define RA8875_FGCR1			0x64
#define RA8875_FGCR2			0x65

/*
 * 5-7 Touch Panel Control Registers
 */
#define RA8875_TPCR0				0x70
#define RA8875_TPCR0_TOUCH_ENABLE	0x80
#define RA8875_TPCR0_TOUCH_DISABLE	0x00
#define RA8875_TPCR0_SMPL_TIME_512CLK	0x00
#define RA8875_TPCR0_SMPL_TIME_1024CLK	0x10
#define RA8875_TPCR0_SMPL_TIME_2048CLK	0x20
#define RA8875_TPCR0_SMPL_TIME_4096CLK	0x30
#define RA8875_TPCR0_SMPL_TIME_8192CLK	0x40
#define RA8875_TPCR0_SMPL_TIME_16384CLK	0x50
#define RA8875_TPCR0_SMPL_TIME_32768CLK	0x60
#define RA8875_TPCR0_SMPL_TIME_65536CLK	0x70
#define RA8875_TPCR0_TPWAKE_ENABLE	0x08
#define RA8875_TPCR0_TPWAKE_DISABLE	0x00
#define RA8875_TPCR0_ADCCLK_DIV1	0x00
#define RA8875_TPCR0_ADCCLK_DIV2	0x01
#define RA8875_TPCR0_ADCCLK_DIV4	0x02
#define RA8875_TPCR0_ADCCLK_DIV8	0x03
#define RA8875_TPCR0_ADCCLK_DIV16	0x04
#define RA8875_TPCR0_ADCCLK_DIV32	0x05
#define RA8875_TPCR0_ADCCLK_DIV64	0x06
#define RA8875_TPCR0_ADCCLK_DIV128	0x07

#define RA8875_TPCR1				0x71
#define RA8875_TPCR1_AUTOMODE		0x00
#define RA8875_TPCR1_MANUALMODE		0x40
#define RA8875_TPCR1_INT_REF		0x00
#define RA8875_TPCR1_EXT_REF		0x20
#define RA8875_TPCR1_NO_DEBOUNCE	0x00
#define RA8875_TPCR1_DEBOUNCE_EN	0x04
#define RA8875_TPCR1_MODE_IDLE		0x00
#define RA8875_TPCR1_MODE_WAITEVENT	0x01
#define RA8875_TPCR1_MODE_LATCH_X	0x02
#define RA8875_TPCR1_MODE_LATCH_Y	0x03

#define RA8875_TPXH					0x72
#define RA8875_TPYH					0x73

#define RA8875_TPXYL				0x74
#define RA8875_TPXYL_IS_TOUCHED		0x00
#define RA8875_TPXYL_NOT_TOUCHED	0x80
#define RA8875_TPXYL_Y_MASK			0x0C
#define RA8875_TPXYL_X_MASK			0x03

/* 
 * 5-9 PLL Setting Registers
 */
#define RA8875_PLLC1			0x88
#define RA8875_PLLC1_PLLDIV1	0x00
#define RA8875_PLLC1_PLLDIV2	0x80

#define RA8875_PLLC2			0x89
#define RA8875_PLLC2_DIV1		0x00
#define RA8875_PLLC2_DIV2		0x01
#define RA8875_PLLC2_DIV4		0x02
#define RA8875_PLLC2_DIV8		0x03
#define RA8875_PLLC2_DIV16		0x04
#define RA8875_PLLC2_DIV32		0x05
#define RA8875_PLLC2_DIV64		0x06
#define RA8875_PLLC2_DIV128		0x07

/*
 * 5-10 PWM Control Registers
 */
#define RA8875_P1CR						0x8A
#define RA8875_P1CR_PWM1EN				0x80
#define RA8875_P1CR_PWM1_SLP_H			0x40
#define RA8875_P1CR_PWM1_FIXFREQ		0x10
#define RA8875_P1CR_PWM1_CLKDIV_1		0x00
#define RA8875_P1CR_PWM1_CLKDIV_2		0x01
#define RA8875_P1CR_PWM1_CLKDIV_4		0x02
#define RA8875_P1CR_PWM1_CLKDIV_8		0x03
#define RA8875_P1CR_PWM1_CLKDIV_16		0x04
#define RA8875_P1CR_PWM1_CLKDIV_32		0x05
#define RA8875_P1CR_PWM1_CLKDIV_64		0x06
#define RA8875_P1CR_PWM1_CLKDIV_128		0x07
#define RA8875_P1CR_PWM1_CLKDIV_256		0x08
#define RA8875_P1CR_PWM1_CLKDIV_512		0x09
#define RA8875_P1CR_PWM1_CLKDIV_1024	0x0A
#define RA8875_P1CR_PWM1_CLKDIV_2048	0x0B
#define RA8875_P1CR_PWM1_CLKDIV_4096	0x0C
#define RA8875_P1CR_PWM1_CLKDIV_8192	0x0D
#define RA8875_P1CR_PWM1_CLKDIV_16384	0x0E
#define RA8875_P1CR_PWM1_CLKDIV_32768	0x0F
#define RA8875_P1DCR			0x8B

#define RA8875_P2CR						0x8A
#define RA8875_P2CR_PWM2EN				0x80
#define RA8875_P2CR_PWM2_SLP_H			0x40
#define RA8875_P2CR_PWM2_FIXFREQ		0x10
#define RA8875_P2CR_PWM2_CLKDIV_1		0x00
#define RA8875_P2CR_PWM2_CLKDIV_2		0x01
#define RA8875_P2CR_PWM2_CLKDIV_4		0x02
#define RA8875_P2CR_PWM2_CLKDIV_8		0x03
#define RA8875_P2CR_PWM2_CLKDIV_16		0x04
#define RA8875_P2CR_PWM2_CLKDIV_32		0x05
#define RA8875_P2CR_PWM2_CLKDIV_64		0x06
#define RA8875_P2CR_PWM2_CLKDIV_128		0x07
#define RA8875_P2CR_PWM2_CLKDIV_256		0x08
#define RA8875_P2CR_PWM2_CLKDIV_512		0x09
#define RA8875_P2CR_PWM2_CLKDIV_1024	0x0A
#define RA8875_P2CR_PWM2_CLKDIV_2048	0x0B
#define RA8875_P2CR_PWM2_CLKDIV_4096	0x0C
#define RA8875_P2CR_PWM2_CLKDIV_8192	0x0D
#define RA8875_P2CR_PWM2_CLKDIV_16384	0x0E
#define RA8875_P2CR_PWM2_CLKDIV_32768	0x0F
#define RA8875_P2DCR					0x8D

#define RA8875_MCLR					0x8E
#define RA8875_MCLR_STARTCLEAR		0x80
#define RA8875_MCLR_ISCLEARING		0x80
#define RA8875_MCLR_FULLWINDOW		0x00
#define RA8875_MCLR_ACTIVEWINDOW	0x40

/*
 * 5-11 Drawing Control Registers
 */
#define RA8875_DCR				0x90
#define RA8875_DCR_STARTDRAWING	0x80
#define RA8875_DCR_ISDRAWING	0x80
#define RA8875_DCR_STARTCIRCLE	0x40
#define RA8875_DCR_ISDRAWCIRCLE	0x40
#define RA8875_DCR_FILL			0x20
#define RA8875_DCR_NOFILL		0x00
#define RA8875_DCR_DRAWSQUARE	0x10
#define RA8875_DCR_DRAWLINE		0x00
#define RA8875_DCR_DRAWTRIANGLE	0x01

#define RA8875_DLHSR0			0x91
#define RA8875_DLHSR1			0x92
#define RA8875_DLVSR0			0x93
#define RA8875_DLVSR1			0x94

#define RA8875_DLHER0			0x95
#define RA8875_DLHER1			0x96
#define RA8875_DLVER0			0x97
#define RA8875_DLVER1			0x98

#define RA8875_DCHR0			0x99
#define RA8875_DCHR1			0x9A
#define RA8875_DCVR0			0x9B
#define RA8875_DCVR1			0x9C
#define RA8875_DCRR				0x9D

#define RA8875_DRE				0xA0
#define RA8875_DRE_STARTDRAWING	0x80
#define RA8875_DRE_ISDRAWING	0x80
#define RA8875_DRE_FILL			0x40
#define RA8875_DRE_ROUNDEDRECT	0x20
#define RA8875_DRE_ELLIPSE		0x00
#define RA8875_DRE_ELLIPSE_CURV	0x10
#define RA8875_DRE_CURVEPART_LB	0x00
#define RA8875_DRE_CURVEPART_LU	0x01
#define RA8875_DRE_CURVEPART_RU	0x02
#define RA8875_DRE_CURVEPART_RB	0x03

#define RA8875_ELL_A0			0xA1
#define RA8875_ELL_A1			0xA2
#define RA8875_ELL_B0			0xA3
#define RA8875_ELL_B1			0xA4
#define RA8875_DEHR0			0xA5
#define RA8875_DEHR1			0xA6
#define RA8875_DEVR0			0xA7
#define RA8875_DEVR1			0xA8
#define RA8875_DTPH0			0xA9
#define RA8875_DTPH1			0xAA
#define RA8875_DTPV0			0xAB
#define RA8875_DTPV1			0xAC

/*
 * 5-13 Key & IO Control Registers
 */
#define RA8875_KSCR1
#define RA8875_KSCR1_ENABLE_KS	0x80
#define RA8875_KSCR1_LONGKEY_EN	0x40
#define RA8875_KSCR1_DEBOUNCE4	0x00
#define RA8875_KSCR1_DEBOUNCE8	0x10
#define RA8875_KSCR1_DEBOUNCE16	0x20
#define RA8875_KSCR1_DEBOUNCE32	0x30
#define RA8875_KSCR1_KS_128US	0x00
#define RA8875_KSCR1_KS_256US	0x01
#define RA8875_KSCR1_KS_512US	0x02
#define RA8875_KSCR1_KS_1024US	0x03
#define RA8875_KSCR1_KS_2048US	0x04
#define RA8875_KSCR1_KS_4096US	0x05
#define RA8875_KSCR1_KS_8192US	0x06
#define RA8875_KSCR1_KS_16384US	0x07

#define RA8875_KSCR2			0xC1
#define RA8875_KSCR2_KS_WAKE_EN	0x80
#define RA8875_KSCR2_LONG_125	0x00
#define RA8875_KSCR2_LONG_250	0x04
#define RA8875_KSCR2_LONG_375	0x08
#define RA8875_KSCR2_LONG_500	0x0C
#define RA8875_KSCR2_NKEYHIT_0	0x00
#define RA8875_KSCR2_NKEYHIT_1	0x01
#define RA8875_KSCR2_NKEYHIT_2	0x02
#define RA8875_KSCR2_NKEYHIT_3	0x03

#define RA8875_KSDR0			0xC2
#define RA8875_KSDR1			0xC3
#define RA8875_KSDR2			0xC4
#define RA8875_GPIOX			0xC7

/*
 *5-16 Interrupt Control Registers
 */
#define RA8875_INTC1				0xF0
#define RA8875_INTC1_EN_KEYSCAN_INT	0x10
#define RA8875_INTC1_EN_DMA_INT		0x08
#define RA8875_INTC1_EN_TOUCH_INT	0x04
#define RA8875_INTC1_EN_BTE_INT		0x02
#define RA8875_INTC1_EN_FONT_INT	0x01

#define RA8875_INTC2				0xF1
#define RA8875_INTC2_KEYSCAN_INTF	0x10
#define RA8875_INTC2_DMA_INTF		0x08
#define RA8875_INTC2_TOUCH_INTF		0x04
#define RA8875_INTC2_BTE_INTF		0x02
#define RA8875_INTC2_FONT_INTF		0x01

#endif /* RA8875_H_ */