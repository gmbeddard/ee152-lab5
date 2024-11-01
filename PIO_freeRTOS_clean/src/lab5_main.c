// Pin assignment:
//	DAC1 drives PA4(Nano A3).
//	TIM2 channel 1 drives PA5(Nano A4).

#include "stm32l432xx.h"
#include "lib_ee152.h"

//**********************************
// DMA
//**********************************
// Configure the DMA controller for MEM to CSR (where we read from consecutive
// memory locations but always write the same CSR).
// You supply the memory & CSR addresses.
// DMA_data_size is the total bytes of memory to transfer, and 'circular' is
// whether to do the transfer just once or to loop forever.
// We always read memory & write the CSR in 32-bit chunks, use channel 3 of DMA
// controller #1, and do a new transfer every time DAC #1 says it's ready.
static void setup_DMA (uint8_t *DMA_mem_addr,
		volatile uint32_t *DMA_periph_addr,
		unsigned int DMA_data_size, int circular) {
    // The DMA controller is chapter 11 of the RM0394 reference manual.

    // Turn on the clocks to the DMA controller.
    RCC->AHB1ENR = 4; // reset clock

    // Step 1; enableing clock 3 different ways 
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // same as RCC->AHB1ENR |= (0x1UL  <<  (0U)); same as RCC->AHB1ENR |= 1;

    // Configuration register for DMA channel x (DMA_CCRx); one CSR for each
    // channel.
    // PSize is pretty much irrelevant (at least for 8b DAC writes). E.g., 2B
    // MSize and 1B PSize just seems to mean losing the 8 most-significant bits
    // of each memory read, rather than very efficiently having 1 mem read ->
    // 2 DAC writes.

    // link: https://controllerstech.com/how-to-setup-dma-using-registers/
    // Step 2: enable DMA interrrupts 
    // skipping this bc "DMA ISR (interrupt status reg) needs no programming."

    // Step 3: setting priority level to low (00) aka clearing the bit to 0
    DMA1_Channel3->CCR &= ~DMA_CCR_PL;  // same as writing DMA1_Channel3->CCR |= 0; i think
    // Step 3: set data direction
    DMA1_Channel3->CCR &= ~DMA_CCR_MEM2MEM; // also written as DMA1_Channel3->CCR &= ~(1 << 14);  

    // Step 4: set the memory data size (MSIZE)
    // Joel is requesting 16 bits which is 01 (0 in DMA_CCRx[11] and 1 in [10])
    DMA1_Channel3->CCR &= ~(1 << 10);     // left shifts "1" to the 10th spot
    //TODO: not sure if this is right below
    DMA1_Channel3->CCR |= (1 << 10);    //16 bits - not sure if this is right

    // Step 5: Set the peripheral data size (PSIZE)
        // resetting the PSIZE to default (00)
        // DMA_CCRx[9] and [8]
    DMA1_Channel3->CCR &= ~(1 << 8);      // same as writing DMA1_Channel3->CCR &= ~DMA_CCR_PSIZE;

    // Step 6: enable memory incrememnt (MINC) to 1
        // setting to 1
        // DMA_CCRx[7] so shift 1 bit left by 7 spaces
    DMA1_Channel3->CCR |= (1 << 7);       // same as writing DMA1_Channel3->CCR |=  DMA_CCR_MINC

    // Step 7: disable PINC
        // setting to 0
        // DMA_CCRx[6] so &~ a shifted bit of 1 by 6 spaces
    DMA1_Channel3->CCR &= ~(1 << 6);      // same as writing DMA1_Channel3->CCR &= ~DMA_CCR_PINC

    // Step 6: enable circular mode
    if (circular) {
        // make sure MEM2MEM is cleared since DMA can't be both circ and mem2mem!! (p. 312)
        // clear MEM2MEM bit of register
        DMA1_Channel3->CCR &= ~DMA_CCR_MEM2MEM; // also written as DMA1_Channel3->CCR &= ~(1 << 14);
        // set circular to true
        DMA1_Channel3->CCR |= DMA_CCR_CIRC; // Can also be written as (pg 312): DMA1_Channel3->CCR |= 1 << 5;
    }
    else {
        // disable CIRC in case MEM2MEM is on
        DMA1_Channel3->CCR &= ~DMA_CCR_CIRC;
        // enable MEM2MEM bit
        DMA1_Channel3->CCR |= DMA_CCR_MEM2MEM;

    }

    // Step 7: enable DIR (read from memory p. 314)
    DMA1_Channel3->CCR |= (1 << 4);           // same as: DMA1_Channel3->CCR |= DMA_CCR_DIR;

    // Number of data to transfer register for "DMA channel x" (DMA_CNDTRx). One
    // CSR for each DMA channel; it contails the number of data to transfer.
    // Set to the appropriate number. And we can read it (after disabling the
    // channel).
    DMA1_Channel3->CNDTR = DMA_data_size;

    // Peripheral address register for "DMA channel x" (DMA_CPARx)
        // setting it equal to our argument of peripheral data
        // cast pointer as a unsigned integer of 32 bits to make sure it actually is that
    DMA1_Channel3->CPAR = (uint32_t) DMA_periph_addr;

    // Memory address register for "DMA channel x" (DMA_CMARx)
        // setting it equal to our argument of peripheral data
        // cast pointer as a unsigned integer of 32 bits to make sure it actually is that
    DMA1_Channel3->CMAR = (uint32_t) DMA_mem_addr;

    // DMA channel selection register (DMA_CSELR)
    // This is one CSR for all seven channels of one DMA controller.
    // Bits for request mapping: so,
        // let's enable our channel 3! which are DMA1_CSELR[11:8]
        // first clear all 4 spots
    DMA1_CSELR->CSELR &= ~(1 << 8);
    DMA1_CSELR->CSELR &= ~(1 << 9);
    DMA1_CSELR->CSELR &= ~(1 << 10);
    DMA1_CSELR->CSELR &= ~(1 << 11);
        // then, let's set them to enable channel 3
    DMA1_CSELR->CSELR |= (1 << 8);
    DMA1_CSELR->CSELR |= (1 << 9);
    DMA1_CSELR->CSELR |= (1 << 10);
    DMA1_CSELR->CSELR |= (1 << 11);


    // Finally: enable the DMA channel.
    DMA1_Channel3->CCR |=  DMA_CCR_EN;
}

// Set up timer 2, channel 'chan' to drive its output with a frequency divider.
// For the main counter:
// - Prescaler set to 'presc' parameter.
// - Counter counts up, and reloads when it hits 'reload'-1.
// - Generate TrgO when the counter reloads.
// For output channel 'chan'
// - It's an output channel, and toggles on each counter=0 event.
// - We don't set up the GPIO pin to listen to the output channel; you must
//   do that separately.
static void setup_TIM2_channel
	(unsigned int presc, unsigned int reload, int chan){
    // Clock enable for timer 2, on the APB1 clock.
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // First, the logic that applies for all four channels; mostly, the main
    // counter and auto-reload register.
    // Control register 1 (CR1). Set timer to count up.
    TIM2->CR1 &= ~TIM_CR1_DIR;

    // Prescaler (PSC). Note we divide by val+1, not by val.
    TIM2->PSC = presc-1;			// Prescale by 1000

    // Counter auto-reload register. Again, the count sequence has val+1
    // numbers.
    TIM2->ARR = reload-1;			// Auto-reload register.

    // Set CR2.MMS[2:0] = 0b010, which generates a trigger-output (TRGO) on
    // every update event (i.e., the counter wrapping around).
    TIM2->CR2 &= ~TIM_CR2_MMS_Msk;
    TIM2->CR2 |= (0x2UL << TIM_CR2_MMS_Pos);

    // Program a few more CSRs.
    // CCR1 = compare/capture register for our channel. This says which
    // main-counter value causes (in our case) a toggle. It doesn't really
    // what it is for our application, as long as it's in [0,ARR]. So we keep
    // it at 0, which is its reset value.

    // CCRM1, CCRM2 - compare-capture mode register for channels 1,2/3,4.
    // For each channel #:
    //	- CC#S=00 => this is an output channel.
    //	- OC#M=0011 => OCREF toggles on each counter match.

    // CCER = capture/Compare Enable Register. This both enables the output
    // channel to drive an output pin, and sets the polarity of CC and CCN vs.
    // OCREF. It's one register for four output channels. If channel # is an
    // output channel, then CC#E=1, CC#NP=0, and CC#P sets the polarity (which
    // for our use is irrelevant).

    switch (chan) {
	case 1: TIM2->CCR1 = 0;
		TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
		TIM2->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1;
		TIM2->CCER &= ~TIM_CCER_CC1NP;
		TIM2->CCER |= TIM_CCER_CC1E;
		break;
	case 2: TIM2->CCR2 = 0;
		TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
		TIM2->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1;
		TIM2->CCER &= ~TIM_CCER_CC2NP;
		TIM2->CCER |= TIM_CCER_CC2E;
		break;
	case 3: TIM2->CCR3 = 0;
		TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
		TIM2->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1;
		TIM2->CCER &= ~TIM_CCER_CC3NP;
		TIM2->CCER |= TIM_CCER_CC3E;
		break;
	case 4: TIM2->CCR4 = 0;
		TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;
		TIM2->CCMR2 |= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1;
		TIM2->CCER &= ~TIM_CCER_CC4NP;
		TIM2->CCER |= TIM_CCER_CC4E;
		break;
    }

    // Finally, enable timer #2.
    TIM2->CR1 |= TIM_CR1_CEN;
}

// Based on DAC1_Init(). But it:
//  - sets the DAC to trigger from timer2 TrgO
//  - enables DMA (and sends the DMA controller a signal when we can take
//    new data)
static void DAC1_Init_with_DMA (){
    // First, set GPIO pin PA4 (the DAC-1 output) to be an analog output.
    // Note we don't have to program ASCR; it doesn't exist on the 432.
    // Enable the clock of GPIO Port A
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Configure PA4 (DAC1_OUT1) as Analog
    GPIOA->MODER |=   3U << (2*4);  // 2 bits of mode per pin; 11b = Analog
    // GPIO port pup/pulldown register. It has 2 bits per pin, and we set 
    // 00=>No pull-up or pull-down (after all, it's an analog output).
    GPIOA->PUPDR &= ~(3U << (2*4));

    // Turn on the DAC clocks, set DAC1 to drive PA4 via the DAC
    // buffer, and disable triggering.

    // APB1 Peripheral clock Enable Register 1 (APB1ENR1)
    // It looks like this one bit enables the clock for both DACs.
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

    // DAC mode control register (DAC_MCR). Each of the two DACs has three bits
    // of mode. We set 000, or DAC1 driving its external pin (PA4) via a buffer.
    // The buffer allows higher drive current.
    // This value of 000 also turns off sample-and-hold mode.
    DAC->MCR &= ~DAC_MCR_MODE1;

    // DAC control register (DAC_CR)
    // DAC channel2 trigger enable. It's needed to get our trigger from the
    // timer. Then (from RM0394, Table 75 in Chapter 17), tim2 trigger output
    // (TRGO) = 4
    DAC->CR |=  DAC_CR_TEN1;		// Trigger enable
    DAC->CR &= ~DAC_CR_TSEL1;		// TrgO from timer 2.
    DAC->CR |= 0x4UL << DAC_CR_TSEL1_Pos;	// TrgO from timer 2.

    // Enable DMA (DMAEN1=1).
    // Don't enable interrupt on underrun (DMAUDRIE1=0); if it ever happens
    // there is a DAC status-reg bit to check for it.
    DAC->CR |=  DAC_CR_DMAEN1;		// Enable DMA
    DAC->CR &= ~DAC_CR_DMAUDRIE1;	// No DMA underrun interrupt.

    // Same register again: enable DAC #1.
    DAC->CR |=  DAC_CR_EN1;       // Enable DAC Channel 1

    delay(1);			// time is in ms.
}

// Various waveforms.
// Sawtooth
//static uint8_t wave_mem[] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130};
// Reverse sawtooth
//static uint8_t wave_mem[] = {130,120,110,100,90,80,70,60,50,40,30,20,10,0};
// TODO: get pictures of sine in lab!
// // Square wave
// static uint8_t wave_mem[] = {0, 130, 0, 130, 0, 130, 0, 130, 0, 130, 
//                              0, 130, 0, 130, 0, 130, 0, 130, 0, 130};
// TODO: get pictures of triangle in lab!
// Triangle
//static uint8_t wave_mem[] = {0,10,20,30,40,50,60,70,60,50,40,30,20,10};
// Sine
static uint8_t wave_mem[]= {100, 131, 159, 181, 195, 200, 195, 181, 159, 131,
			100,  69,  41,  19,   5,   0,   5,  19,  41,  69};

int main(void) {
    clock_setup_80MHz();	// 80MHz = 12.5ns phase

    // Setup for the GPIO port A, pin 5 to be alternate-function #1.
    // This allows DAC #1 to drive it. Usually analogWrite() does this, but
    // we're using a custom DAC driver in this lab rather than analogWrite().
    set_gpio_alt_func (GPIOA, 5, 1);	// Channel 1 = PA5 = Nano A4

    // Now set up timer 2, channel 1. We'll use this to trigger the DAC.
    setup_TIM2_channel (100, 100, 1);	// prescaler, reload #, channel

    DAC1_Init_with_DMA ();
    DAC->DHR8R1 = 0x80;	// Get an initial DAC value in before turning on DMA.

    int n_datum = sizeof (wave_mem) / sizeof (uint8_t);
    setup_DMA (&wave_mem[0], &DAC->DHR8R1, n_datum, 1);

    // Dead loop & program hangs here
    while(1) {
    }
}