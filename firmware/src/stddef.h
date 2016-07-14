//#ifndef _STDDEF_H_
//#define _STDDEF_H_


#define TRUE	1
#define FALSE	0

/**
	Возвращает старший байт слова
*/
#define HI_WORD(x) (((x) >> 8) & 0xFF)			

/**
	Возвращает младший байт слова
*/
#define LO_WORD(x) ((x) & 0xFF)

#define GLUE(a, b)     a##b
#define PORT(x)        GLUE(PORT, x)
#define PIN(x)         GLUE(PIN, x)
#define DDR(x)         GLUE(DDR, x)


//#endif // _STDDEF_H_
