#ifndef ADC_REGISTERS_H
#define ADC_REGISTERS_H


#define ADC_BASE_ADDRESS	0x40038000 //1073971200
#define ADC_MAX_ADDRESS	0x400380FC //1073971452

enum {
	DATA_WIDTH = 32
};

struct ADC_MEMORY{
	bool reg_data[DATA_WIDTH/4];
} adc_reg[(ADC_MAX_ADDRESS-ADC_BASE_ADDRESS)];

struct tlm_transaction
{
	int reg_cmd;
	int reg_address;
	bool reg_data[DATA_WIDTH];
};


#define ADC_CR_ 	(0x00) //1073971200
#define ADC_MR_ 	(0x04) //1073971204
#define ADC_CHER_ 	(0x10) //1073971216
#define ADC_CHDR_ 	(0x14) //1073971220
#define ADC_CHSR_ 	(0x18) //1073971224
#define ADC_CGR_ 	(0x48) //1073971272
#define ADC_COR_ 	(0x4C) //1073971276
#define ADC_CDR0_ 	(0x50) //1073971280
#define ADC_CDR1_ 	(0x54) //1073971284
#define ADC_CDR2_ 	(0x58) //1073971288
#define ADC_CDR3_ 	(0x5C) //1073971292
#define ADC_CDR4_ 	(0x60) //1073971296
#define ADC_CDR5_ 	(0x64) //1073971300
#define ADC_CDR6_ 	(0x68) //1073971304
#define ADC_CDR7_ 	(0x6C) //1073971308
#define ADC_CDR8_ 	(0x70) //1073971312
#define ADC_CDR9_ 	(0x74) //1073971316
#define ADC_CDR10_	(0x78) //1073971320
#define ADC_CDR11_	(0x7C) //1073971324
#define ADC_CDR12_	(0x80) //1073971328
#define ADC_CDR13_	(0x84) //1073971332
#define ADC_CDR14_	(0x88) //1073971336
#define ADC_CDR15_	(0x8C) //1073971340
#define ADC_WPMR_ 	(0xE4) //1073971428

// ADC_CR register's fields
#define ADC_CR_START_0		(1)
#define ADC_CR_SWRST_0		(0)
// ADC_MR register's fields
#define ADC_MR_TRANSFER_1	(29)
#define ADC_MR_TRANSFER_0	(28)
#define ADC_MR_TRACKTIM_3	(27)
#define ADC_MR_TRACKTIM_2	(26)
#define ADC_MR_TRACKTIM_1	(25)
#define ADC_MR_TRACKTIM_0	(24)
#define ADC_MR_ANACH_0		(23)
#define ADC_MR_SETTLING_1	(21)
#define ADC_MR_SETTLING_0	(20)
#define ADC_MR_STARTUP_3	(19)
#define ADC_MR_STARTUP_2	(18)
#define ADC_MR_STARTUP_1	(17)
#define ADC_MR_STARTUP_0	(16)
#define ADC_MR_PRESCAL_7	(15)
#define ADC_MR_PRESCAL_6	(14)
#define ADC_MR_PRESCAL_5	(13)
#define ADC_MR_PRESCAL_4	(12)
#define ADC_MR_PRESCAL_3	(11)
#define ADC_MR_PRESCAL_2	(10)
#define ADC_MR_PRESCAL_1	(9)
#define ADC_MR_PRESCAL_0	(8)
#define ADC_MR_LOWRES_0		(4)

#endif /* ADC_REGISTERS_H */
