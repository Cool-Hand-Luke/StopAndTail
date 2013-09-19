/* Avoid polluting the global namespace with aliases for each pin. */
#define NO_BIT_DEFINES

/* Include SDCC-specific device header. */
#include "pic12f675.h"

#define TRUE  1
#define FALSE 0

#define BTST(a,b) ((a) & (b))
#define BSET(a,b) ((a) |= (b))
#define BCLR(a,b) ((a) &= ~(b))
#define BCHG(a,b) ((a) = (a) ^ (b))

/* Define some useful types -- might #include "inttypes.h" instead. */
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;


  /* Setup chip configuration */
typedef unsigned int config;
config at 0x2007 __CONFIG =  _CP_OFF & _CPD_OFF & _WDT_ON & _PWRTE_OFF & _MCLRE_ON & _INTRC_OSC_NOCLKOUT;

/* Adjust to your clock frequency (in Hz). */
#define CLOCKFREQ 4000000U

/* Instructions per millisecond. */
#define INSNS_PER_MS (CLOCKFREQ / 4000U)

/* Delay loop is about 10 cycles per iteration. */
#define LOOPS_PER_MS (INSNS_PER_MS / 10U)

// Output Ports
#define STATUS_LED  GPIO_bits.GP0
#define OUTPUT_LED  GPIO_bits.GP1
#define INPUT       GPIO_bits.GP5

// PWM control
#define DUTY_FREQ 50
#define MAX_DUTY  30
uint8_t step = 0;                                 // Current time slice (step) of the duty cycle.
uint8_t duty = 0;                                 // Current duty cycle (copied from led_tail or led_brake)

// Stop/Tail light control
static volatile uint8_t led_on = 1;
static volatile uint8_t led_tail = 0;
static volatile uint8_t led_brake = 0;

// Constants and variables for runtime configuration.
#define   PWM_FREQUENCY   422                     // Measured HZ of the PWM circuit.
#define   END_CONFIG      PWM_FREQUENCY * 10      // Holding the brake for 10 seconds will exit the config utility
#define   NEXT_CONFIG     PWM_FREQUENCY * 5       // Holding the brake for 5 seconds will cycle to the next edit phase.
#define   CONFIG_TIMEOUT  PWM_FREQUENCY * 60      // How long before the config times out.
#define   INIT_TIMEOUT    PWM_FREQUENCY * 2       // Timeout in seconds timeout between presses
#define   CONFIG_PRESSES  15                      // How many presses to enter system config.
#define   EDIT_NONE       0                       // Not in system config.
#define   EDIT_TAILLIGHT  1                       // Configuring the taillight brightness.
#define   EDIT_BRAKELIGHT 2                       // Configuring the brake light brightness.
#define   PULSE_NOTIFY    1                       // How many pulses for notify.
#define   PULSE_CONFIG    4                       // Pulses for entering config.
#define   PULSE_NONE      3                       // How many pulses for exiting config.
#define   PULSE_TAIL      1                       // How many pulses for entering edit taillight.
#define   PULSE_BRAKE     2                       // Pulses entering edit brake light.
#define   SAVE_TAIL       0x04                    // Where to save the taillight setting.
#define   SAVE_BRAKE      0x08                    // Where to save the brakelight setting.
#define   SAVE_INIT       0x16                    // Used to say we have initialised.
#define   DEFAULT_TAIL    5                       // Default taillight setting.
#define   DEFAULT_BRAKE   25                      // Default brakelight setting.
unsigned char gBraking  = 0;                      // Are we braking? (true/false)
unsigned char gMode     = 0;                      // Current configuration mode.
unsigned char gTrigger  = 0;                      // keep count of trigger pulses (by pressing the brake lever)

// Timer (for use during code)
static volatile unsigned long timer;

void
delay_ms (uint16_t ms)
{
  uint16_t u;

  /* reset watchdog timer...
   * (Check your call doesn't exceed the timeout) */
  __asm clrwdt __endasm;

  while (ms--)
  {
    /* Inner loop takes about 10 cycles per iteration + 4 cycles setup. */
    for (u = 0; u < LOOPS_PER_MS; u++)
    {
      __asm nop __endasm;
    }
  }
}

void
pulse_light (unsigned char pulses)
{
  // Flash the rear light to let the user know we have entered config mode.
  for (; pulses > 0; pulses--)
  {
    led_on = 0;
    delay_ms (30);
    led_on = 1;
    delay_ms (30);
  }
}

// Timer 0 used for creating the PWM output
void
ISR (void)
  __interrupt
{
  ///////////////////////////////////////////////
  // Timer 0
  if (INTCON_bits.T0IF == 0x1)
  {
    INTCON_bits.T0IF = 0;       // clear timer0 overflow bit.
    TMR0 = 255 - DUTY_FREQ;     // Adjust Hertz

    // Increment and check for new cycle
    if (step++ == 0)
    {
      // For use by the main() program
      timer++;

      // Only turn on the LED if duty > 0 (so we can have 0-100% duty cycle)
      if (duty > 0)
      {
        OUTPUT_LED = led_on;    // Turn on the LED(s) if enabled.
      }
    }

    // Duty cycle
    if (step > duty)
      OUTPUT_LED = 0x0;         // Turn the LED(s) off when duty cycle limit reached.

    if (step > MAX_DUTY)
      step = 0;                 // One cycle has completed.  Start over.
  }
}

void
set_editmode (unsigned char mode)
{
  gMode = mode;                 // Save the mode.

  // Set the status LED.
  if (mode > EDIT_NONE)
  {
    STATUS_LED = 0x1;
  }
  else
  {
    STATUS_LED = 0x0;
  }

  if (mode < EDIT_BRAKELIGHT)
  {
    duty = led_tail;
  }
  else
  {
    duty = led_brake;
  }
}

void
set_nextmode (void)
{
  switch (gMode)
  {
  case EDIT_TAILLIGHT:
    gMode = EDIT_BRAKELIGHT;
    pulse_light (PULSE_BRAKE);
    break;
  case EDIT_BRAKELIGHT:
    gMode = EDIT_TAILLIGHT;
    pulse_light (PULSE_TAIL);
    break;
  default:
    gMode = EDIT_NONE;
    pulse_light (PULSE_NONE);
    break;
  }

  set_editmode (gMode);
}

#define RP0                  STATUS_bits.RP0
#define RD                   EECON1_bits.RD
#define WR                   EECON1_bits.WR
#define WREN                 EECON1_bits.WREN
#define GIE                  INTCON_bits.GIE
void
EEPROM_putc (unsigned char address, unsigned char _data)
{
  EEDATA = _data;
  EEADR = address;

  // start write sequence as described in datasheet, page 91
  //RP0                   = 0;
  WREN = 1;                     // enable writes to data EEPROM
  GIE = 0;                      // disable interrupts
  EECON2 = 0x55;
  EECON2 = 0x0AA;
  WR = 1;                       // start writing
  while (WR)
  {
    _asm nop _endasm;
  }
  WREN = 0;
  GIE = 1;                      // enable interrupts
}

void
EEPROM_getc (unsigned char address, unsigned char *_data)
{
  EEADR = address;
  RD = 1;
  *_data = EEDATA;
}

// Load our saved values (or create them if they don't exist)
void
_init_def_values (void)
{
  unsigned char initialised;

  EEPROM_getc (SAVE_INIT, &initialised);

  if (initialised != 0xF1)
  {
    led_tail = DEFAULT_TAIL;
    led_brake = DEFAULT_BRAKE;
    initialised = 0xF1;

    STATUS_LED = 0x1;
    EEPROM_putc (SAVE_TAIL, led_tail);
    EEPROM_putc (SAVE_BRAKE, led_brake);
    EEPROM_putc (SAVE_INIT, initialised);
  }
  else
  {
    EEPROM_getc (SAVE_TAIL, &led_tail);
    EEPROM_getc (SAVE_BRAKE, &led_brake);
  }

  if (led_tail < 1 || led_tail > led_brake || led_tail >= led_brake)
  {
    led_tail = DEFAULT_TAIL;
    EEPROM_putc (SAVE_TAIL, led_tail);
  }

  if (led_brake <= led_tail || led_brake > (MAX_DUTY + 1))
  {
    led_brake = DEFAULT_BRAKE;
    EEPROM_putc (SAVE_BRAKE, led_brake);
  }

  // Set the default duty to taillight
  duty = led_tail;
}

uint8_t
increment_mode (uint8_t min, uint8_t cur, uint8_t max)
{
  if (cur++ > max)
  {
    return min;
  }

  return cur;
}

unsigned char flashed = 0x0;
void
main ()
{
  __asm banksel OSCCAL;
  call 0x3FF;                                   // Calibration instruction stored at last location of program memory
  movwf OSCCAL;                                 // move calibaration value to OSCCAL
  __endasm;

  TRISIO  = 0b100000;                           // Set I/O (0 == output)
  WPU     = 0b000000;                           // Enable weak pull-ups on selected ports

  // bit 7 = GPIO Pull-up enable bit (0 = pull-ups are eneabled by individual port latch values)
  // bit 6 = Interrupt edge select bit (0 = falling edge of GP2/INT, 1 = rising edge of GP2/INT)
  // bit 5 = TMR0 Clock Source (0 = internal, 1 = transistion on GP2/TOCK1 pin)
  // bit 4 = TMR0 Source Edge Select (0 = low-to-high transition on GP2/TOCK1 / 1 = high-to-low)
  // bit 3 = Prescaler assignment (0 = PSA assigned to TIMER0, 1 = PSA assigned to WDT)
  // bit 2 - 0 = Prescaler rate select bits
  // Bit Value    TMR0     WDT
  //  000         1:2      1:1
  //  001         1:4      1:2
  //  010         1:8      1:4  (and so on)
  OPTION_REG = 0b01001101;                      // Assign prescaler to WDT

  PIE1    = 0x00;                               // Turn off peripheral interrupts
  IOC     = 0b000000;                           // Turn off GPIO on all ports
  VRCON   = 0x00;                               // Turn off comparator v.ref.
  ANSEL   = 0x00;                               // All pins are Digital
  GPIO    = 0x00;                               // Reset input/output port
  CMCON   = 0b00000111;                         // Disable the comparator
  ADCON0  = 0x00;                               // Turn off A to D convertor

  // Configure Timer 0
  TMR0    = 0x00;                               // clear the timer

  // Enable timer 0 and global intterupts
  // bit 7 = GIE, bit 6 = PEIE, bit 5 = T0IE
  // bit 4 = INTE, bit 3 = GPIE, bit 2 =T0IF
  // bit 1 = INTF, bit 0 = GPIF
  INTCON = 0b10100000;                          // Enable global intterupts and Timer 0

  delay_ms (100);                               // Hardware prevents writing to EEPROM for 72ms.
  _init_def_values ();                          // Initialise the Stop/Tail brightness levels

  while (1)                                     // Loop forever (well, a long time)
  {
    __asm clrwdt __endasm;

    // Check if we are in config mode...
    if (gMode > 0)
    {
      // Check if we should time out...
      if (timer > CONFIG_TIMEOUT)
      {
        // On time-out we will restore our vales from EEPROM
        EEPROM_getc (SAVE_TAIL, &led_tail);
        EEPROM_getc (SAVE_BRAKE, &led_brake);

        set_editmode (EDIT_NONE);
        pulse_light (PULSE_NONE);               // Let the user know something has been recognised.
      }
      // Check if we are doing something
      else if (INPUT == 1)
      {
        // Reset the timer as we have done something
        timer = 0;
        flashed = FALSE;                        // Used to keep track of our flash when NEXT_CONFIG has been reached.
        delay_ms (100);                         // Debounce I
        pulse_light (PULSE_NOTIFY);             // Let the user know something happened.

        // Stay in this loop until the user releases the brake
        while (INPUT == 1 && gMode)
        {
          __asm clrwdt __endasm;

          if (timer > END_CONFIG)
          {
            set_editmode (EDIT_NONE);
            pulse_light (PULSE_NONE);           // Let the user know something has been recognised.

            // The only time we save our config
            EEPROM_putc (SAVE_TAIL, led_tail);
            EEPROM_putc (SAVE_BRAKE, led_brake);
          }
          else if (timer > NEXT_CONFIG)
          {
            if (flashed == FALSE)
            {
              flashed = TRUE;
              set_nextmode ();
            }
          }
        }

        // Process a press if we didn't do anything else
        if (flashed == FALSE)
        {
          if (gMode == EDIT_TAILLIGHT)
          {
            duty = increment_mode (1, led_tail, led_brake - 2);
            led_tail = duty;
          }
          else if (gMode == EDIT_BRAKELIGHT)
          {
            duty = increment_mode (led_tail + 1, led_brake, MAX_DUTY);
            led_brake = duty;
          }
        }
      }
    }
    // Check if the brake lever/pedal pressed
    else if (INPUT == 1)
    {
      // Was it just pressed?
      if (gBraking == FALSE)
      {
        duty = led_brake;                       // Turn on the brake light first!
        gBraking = TRUE;                        // Set that we have entered braking mode.

        delay_ms (20);                          // Debounce (part I)

        if (timer > INIT_TIMEOUT)
        {
          gTrigger = 1;                         // Reset the trigger count (starts at 1)
        }
        else if (timer > 200)                   // Debounce (part II).
        {
          if (++gTrigger == CONFIG_PRESSES)     // Increment the trigger count.
          {                                     // And enter config mode.
            set_editmode (EDIT_TAILLIGHT);      // Enter config mode for tail light.
            pulse_light (PULSE_CONFIG);         // Let the user know something has been recognised.
            gBraking = FALSE;                   // Clear this (so brake light comes on if held down when exiting config)
          }
        }

        timer = 0;                              // Set the timer to zero (use this for configuration)
      }
    }
    // Else if brake was just released
    else if (gBraking == TRUE)
    {
      duty = led_tail;                          // Not so important on release, but still turn the light off first.
      gBraking = FALSE;                         // Set that we have left braking mode.
    }
  }
}
