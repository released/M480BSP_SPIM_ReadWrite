/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include	"project_config.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/

#define DATA_X											(64)
#define DATA_Y											(32)

//M487 EVM SPI FLASH : W25Q32
#define FLASH_BLOCK_SIZE            						(4*1024*1024)    	/* Flash block size. Depend on the physical flash. */

#define DATA_BLOCK_LEN             						(2*1024*1024)      	/* Test block address on SPI flash , last 512k */
#define DATA_BLOCK_INDEX             					(0x00)
#define BUFFER_SIZE                 						(2048)

#define SPI_ERASE_FLASH_BLOCK_SIZE						(64*1024)
#define IS_4BYTES_ADDR            						(0) 


#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  g_buff[BUFFER_SIZE];
#else
uint8_t  g_buff[BUFFER_SIZE] __attribute__((aligned(4)));
#endif

enum
{
	_SPIM_READ_DMA										= 0x00 ,	
	_SPIM_READ_DMM										= 0x01 ,		
	_SPIM_WRITE												= 0x02 ,
	_SPIM_ERASE												= 0x03 ,	
};

uint8_t rambuffer[DATA_Y][DATA_X] = {0};


/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}


void SPIM_write_read(uint32_t start_addr , uint32_t target_len , uint8_t* buffer , uint8_t wr)
{
    uint32_t i, offset;             /* variables */
//    uint32_t *pData;
	uint32_t EraseCount = 0;
	uint32_t counter = 0;
	
    SPIM_Enable_4Bytes_Mode(IS_4BYTES_ADDR, 1);

    SPIM_SET_DCNUM(8);

	switch(wr)
	{
		case _SPIM_ERASE:
		    /*
		     *  Erase flash page
		     */
			EraseCount = (target_len / SPI_ERASE_FLASH_BLOCK_SIZE) + 1;

			#if (_debug_log_SPIM_ERASE_ == 1)	//debug	
		    printf("Erase SPI flash block (0x%6x)...\r\n", start_addr);
			counter = get_tick();				
			#endif

		    for(i = 0; i < EraseCount; i++) 
			{
		        SPIM_EraseBlock(start_addr + (SPI_ERASE_FLASH_BLOCK_SIZE*i), 0, OPCODE_BE_64K, 1, 1);
//				#if (_debug_log_SPIM_ERASE_ == 1)	//debug			
//				printf("addr (ERASE) : 0x%6X : %2d \r\n" , start_addr + (SPI_ERASE_FLASH_BLOCK_SIZE*i) , i );
//				#endif
		    }

			#if (_debug_log_SPIM_ERASE_ == 1)	//debug	
		    printf("done. (%6d bytes: %d ms)\n" , target_len , get_tick() - counter);
			#endif
			
			break;

		case _SPIM_WRITE:
		    /*
		     *  Program data to flash block
		     */
			#if (_debug_log_SPIM_PROGRAMMING_ == 1)	//debug	     
		    printf("Program sequential data to flash block (0x%6x)...\r\n", start_addr);
			counter = get_tick();			
			#endif
		
		    for (offset = start_addr; offset < target_len; offset += BUFFER_SIZE)
		    {
		        SPIM_DMA_Write(offset, IS_4BYTES_ADDR, BUFFER_SIZE, buffer, CMD_NORMAL_PAGE_PROGRAM);
//				#if (_debug_log_SPIM_PROGRAMMING_ == 1)				
//				printf("addr (DMA_W) : 0x%6X \r\n" , offset);				
//				#endif				
		    }
			
			#if (_debug_log_SPIM_PROGRAMMING_ == 1)	//debug		
		    printf("done. (%6d bytes: %d ms)\n" , target_len , get_tick() - counter);
			#endif
			break;

		case _SPIM_READ_DMA:

		    /*
		     *  Verify flash block data with DMA read
		     */
//		    if ((u32RdCmd == CMD_DMA_NORMAL_QUAD_READ) || (u32RdCmd == CMD_DMA_FAST_QUAD_READ) ||
//		            (u32RdCmd == CMD_DMA_FAST_READ_QUAD_OUTPUT))
//		        SPIM_SetQuadEnable(1, 1);

			#if (_debug_log_SPIM_DMA_READ_ == 1)	//debug	
		    printf("Verify SPI flash block data with DMA read (0x%6x)...\r\n", start_addr);
			counter = get_tick();				
			#endif
		    for (offset = start_addr; offset < target_len; offset += BUFFER_SIZE)
		    {
		        SPIM_DMA_Read(offset, IS_4BYTES_ADDR, BUFFER_SIZE, buffer, CMD_DMA_FAST_READ, 1);
//				#if (_debug_log_SPIM_DMA_READ_ == 1)				
//				printf("addr (DMA_R) : 0x%6X \r\n" , offset);				
//				#endif
		    }
			#if (_debug_log_SPIM_DMA_READ_ == 1)	//debug		
		    printf("done. (%6d bytes: %d ms)\n" , target_len , get_tick() - counter);
			#endif
		
			break;
			
		case _SPIM_READ_DMM:
		    /*
		     *  Verify flash block data with DMM read
		     */
			#if (_debug_log_SPIM_DMM_READ_ == 1)	//debug	     
		    printf("Verify SPI flash block (0x%6x) data with DMM read...\r\n", start_addr);
			counter = get_tick();				
			#endif

//		    if ((u32RdCmd == CMD_DMA_NORMAL_QUAD_READ) || (u32RdCmd == CMD_DMA_FAST_QUAD_READ) ||
//		            (u32RdCmd == CMD_DMA_FAST_READ_QUAD_OUTPUT))
//		        SPIM_SetQuadEnable(1, 1);

		    SPIM_EnterDirectMapMode(IS_4BYTES_ADDR, CMD_DMA_FAST_READ, 8);

		    for (offset = start_addr; offset < target_len; offset += BUFFER_SIZE)
		    {
		        memcpy(buffer, (uint8_t *)(SPIM_DMM_MAP_ADDR+offset), BUFFER_SIZE);
//				#if (_debug_log_SPIM_DMM_READ_ == 1)				
//				printf("addr (DMM_R) : 0x%6X \r\n" , offset);				
//				#endif				
		    }
			#if (_debug_log_SPIM_DMM_READ_ == 1)	//debug		
		    printf("done. (%6d bytes: %d ms)\n" , target_len , get_tick() - counter);
			#endif

		    SPIM_ExitDirectMapMode();
		    SPIM_SetQuadEnable(0, 1);			
			break;			
	}

}


void SPIM_Init(void)
{
    uint8_t     idBuf[3];

	SYS_UnlockReg();                   /* Unlock protected registers                      */

    SPIM_SET_CLOCK_DIVIDER(2);        /* Set SPIM clock as HCLK divided by 4 */

    SPIM_SET_RXCLKDLY_RDDLYSEL(0);    /* Insert 0 delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_SET_RXCLKDLY_RDEDGE();       /* Use SPI input clock rising edge to sample received data. */

    SPIM_SET_DCNUM(8);                /* 8 is the default value. */

    if (SPIM_InitFlash(1) != 0)        /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
//        while (1);
    }

    SPIM_ReadJedecId(idBuf, sizeof (idBuf), 1);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    SPIM_DISABLE_CCM();

    SPIM_ENABLE_CACHE();

}

void data_creation(void)
{
	uint8_t x = 0;
	uint8_t y = 0;
	
	for (y = 0 ; y < DATA_Y; y++)
	{
		for (x = 0 ; x < DATA_X; x++)
		{
			rambuffer[y][x] = y*(0x10) + x;
			g_buff[x+DATA_X*y] = rambuffer[y][x];				
		}
	}

	#if 0	// data dump
	for (y = 0 ; y < DATA_Y; y++)
	{
		for (x = 0 ; x < DATA_X; x++)
		{
			printf("0x%2X ," ,rambuffer[y][x]);	
	        if ((x+1)%8 ==0)
	        {
	            printf("\r\n");
	        }			
		}			
	}
    printf("\r\n\r\n");
	#endif

	SPIM_write_read(DATA_BLOCK_INDEX, BUFFER_SIZE, NULL ,  _SPIM_ERASE);	//DATA_BLOCK_LEN , BUFFER_SIZE
	SPIM_write_read(DATA_BLOCK_INDEX, BUFFER_SIZE, g_buff , _SPIM_WRITE);	//DATA_BLOCK_LEN	


}

void data_verify(void)
{
    memset(g_buff, 0, BUFFER_SIZE);
	SPIM_write_read(DATA_BLOCK_INDEX, BUFFER_SIZE, g_buff ,  _SPIM_READ_DMA);	//DATA_BLOCK_LEN	
	dump_buffer_hex(g_buff , BUFFER_SIZE);	
}


void TMR1_IRQHandler(void)
{
//	static uint32_t LOG = 0;
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PH0 ^= 1;
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}


void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif
}

void Custom_Init(void)
{	
	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk|CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk|CLK_STATUS_HIRCSTB_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk|CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk|CLK_STATUS_LIRCSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM_MODULE);

	#if defined (ENABLE_M487_ETM_SPIM)
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk |
                       SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPIM_MOSI | SYS_GPA_MFPL_PA1MFP_SPIM_MISO |
                     SYS_GPA_MFPL_PA2MFP_SPIM_CLK | SYS_GPA_MFPL_PA3MFP_SPIM_SS;
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    PA->SLEWCTL = (PA->SLEWCTL & 0xFFFFF000) |
                  (0x1<<GPIO_SLEWCTL_HSREN0_Pos) | (0x1<<GPIO_SLEWCTL_HSREN1_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN2_Pos) | (0x1<<GPIO_SLEWCTL_HSREN3_Pos);
	#elif defined (ENABLE_M487_PFM_SPIM)
    /* Init SPIM multi-function pins, MOSI(PC.0), MISO(PC.1), CLK(PC.2), SS(PC.3), D3(PC.4), and D2(PC.5) */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk |
                       SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk);
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_SPIM_MOSI | SYS_GPC_MFPL_PC1MFP_SPIM_MISO |
                     SYS_GPC_MFPL_PC2MFP_SPIM_CLK | SYS_GPC_MFPL_PC3MFP_SPIM_SS |
                     SYS_GPC_MFPL_PC4MFP_SPIM_D3 | SYS_GPC_MFPL_PC5MFP_SPIM_D2;
    PC->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    PC->SLEWCTL = (PC->SLEWCTL & 0xFFFFF000) |
                  (0x1<<GPIO_SLEWCTL_HSREN0_Pos) | (0x1<<GPIO_SLEWCTL_HSREN1_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN2_Pos) | (0x1<<GPIO_SLEWCTL_HSREN3_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN4_Pos) | (0x1<<GPIO_SLEWCTL_HSREN5_Pos);
	#endif

	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	UART0_Init();
	Custom_Init();	
	TIMER1_Init();

	SPIM_Init();

	data_creation();
	data_verify();	
	
    /* Got no where to go, just loop forever */
    while(1)
    {


    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
