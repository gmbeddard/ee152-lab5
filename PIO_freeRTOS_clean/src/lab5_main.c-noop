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
static void setup_DMA (uint16_t *DMA_mem_addr,
		volatile uint32_t *DMA_periph_addr,
		unsigned int DMA_data_size, int circular) {
    // The DMA controller is chapter 11 of the RM0394 reference manual.

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
    
}

// Based on just DAC1_Init(). But it:
//  - sets the DAC to trigger from timer2 TrgO
//  - enables DMA (and sends the DMA controller a signal when we can take
//    new data)
static void DAC1_Init_with_DMA (){
    
}

// Various waveforms.
// Sawtooth
//static uint16_t wave_mem[] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130};
// Reverse sawtooth
//static uint16_t wave_mem[] = {130,120,110,100,90,80,70,60,50,40,30,20,10,0};
// Triangle
//static uint16_t wave_mem[] = {0,10,20,30,40,50,60,70,60,50,40,30,20,10};
// Sine
static uint16_t wave_mem[]= {100, 131, 159, 181, 195, 200, 195, 181, 159, 131,
			100,  69,  41,  19,   5,   0,   5,  19,  41,  69};

int main(void){
    // At 80 MHz()
    //    prescale=1000, match=1000 -> cycle = 25ms (2*1e6/80MHz)
    //    prescale=500,  match=1000 -> cycle = 12.5ms
    // At 16 MHz()
    //    prescale=1000, match=1000 -> cycle = 125ms (2*1e6/16MHz)
    //    So I'll conclude that the APB1 clock (which is what timer #2 uses) is
    //    running at 4MHz (remembering that our toggle-mode timer output means
    //    that one toggle = 1 phase).
    // With clock_setup() turned on, we're up to 8MHz.
    // At AHB=1x, APB1=1x -> 125ms = 8MHz.

    //clock_setup_16MHz();	// gives us 62.5ms phase -> 16MHz
    clock_setup_80MHz();	// gives us 12.5ms phase -> 80MHz
 
    // Setup for the GPIO port B, pin 3 to be alternate-function #1.
    // This allows DAC #1 to drive it.
    set_gpio_alt_func (GPIOA, 5, 1);	// Channel 1 = PA5 = Nano A4

    // Now set up timer 2, channel 2. The params are prescaler, reload dividers.
    // With /1000, the trigger will fire at 80KHz. We'll use this to trigger
    // the DAC.
    setup_TIM2_channel (100, 100, 1);

    DAC1_Init_with_DMA ();
    DAC->DHR8R1 = 0x80;	// Get an initial DAC value in before turning on DMA.

    //setup_DMA (&wave_mem[0], &DAC->DHR8R1, 14, 1); // triangle has 14 points.
    setup_DMA (&wave_mem[0], &DAC->DHR8R1, 20, 1); // sine wave has 20 points.

    // Dead loop & program hangs here
    while(1) {
    }
}

