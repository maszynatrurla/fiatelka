/**
 * Fiatelka
 * Christmas light controller. Timer controlled lights triggered by dark or button.
 */
#define F_CPU 128000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


/* pinout:
 *
 *                       _____
 *   1 - \reset | pb5  -|o    |-  vcc  - 8
 *   2 - pb3           -|     |-  pb2  - 7
 *   3 - pb4           -|     |-  pb1  - 6
 *   4 - gnd           -|_____|-  pb0  - 5
 *
 *
 *  PBB3 (aka. LED_PIN) - output - to transistor gate (through resistor)
 *        LEDs off when output state is LOW
 *
 *  PB4 (aka. LUX_PIN) - analog input - to voltage divider between photoresistor and 100kOhm
 *  PB1 (INT0, BUT_PIN) - external interrupt, external pull up 47kOhm, button connects it to ground
 *
 * programming pins:
 *  \reset - pb5
 *  sck    - pb2
 *  miso   - pb1
 *  mosi   - pb0
 */
#define LED_PIN PB3
#define LUX_PIN PB4
#define BUT_PIN PB1

/* EEPROM
 * +------+------------------------------------------------------------------------------+
 * | 0x00 | Threshold value of voltage measurement for "dark". Low byte.                 |
 * | 0x01 | Threshold value of voltage measurement for "dark". High byte.                |
 * | 0x02 | Threshold value of voltage measurement for "bright". Low byte.               |
 * | 0x03 | Threshold value of voltage measurement for "bright". High byte.              |
 * | 0x04 | Length of time LEDS will be lit once triggered, in multiplies of 15 minutes. |
 * +------+------------------------------------------------------------------------------+
 */
#define EEPROM_DIM_L    0
#define EEPROM_DIM_H    1
#define EEPROM_BRIGHT_L 2
#define EEPROM_BRIGHT_H 3
#define EEPROM_LIT_LEN  4

/* Default values of configuration
 * If EEPROM is erased, then configuration options have following values
 */
#define THRESHOLD_DIM    150    /**< Voltage threshold for 'dark' condition. */
#define THRESHOLD_BRIGHT 630    /**< Voltage threshold for 'light' condition. */
#define LIT_LENGTH       1      /**< Length of time LEDs will be lit in multiples of 15 min. */

/* FSM states.
 *
 *  rst-> ( BRIGHT ) -> ( FADE_IN ) -> ( SETUP ) -> ( GLOW )
 *             ^                                       |
 *             |                                       |
 *             +----  (  NIGHT )  <- ( FADE_OUT )  <---+
 *
 */
#define STATE_BRIGHT    0   /**< Waiting to become dark enough. */
#define STATE_FADE_IN   1   /**< Slowly turning up brightness of LED. */
#define STATE_SETUP     2   /**< Setting timeout value. */
#define STATE_GLOW      3   /**< Waiting for timeout. */
#define STATE_FADE_OUT  4   /**< Slowly dialing down brightness of LED. */
#define STATE_NIGHT     5   /**< Waiting to become bright. */

/* SW PWM max delay. */
#define FAKE_PWM_MAX 255

/**
 * Configuration programs.
 */
enum EConfigProgram 
{
    ECfgAbort           = 0,    /**< Abort config, before it even starts. */
    ECfgExit            = 1,    /**< Exit config. */
    ECfgCalibrateDark   = 2,    /**< Setting up 'dark' threshold. */
    ECfgCalibrateBright = 3,    /**< Setting up 'bright' threshold. */
    ECfgGetLitLength    = 4,    /**< Showing current LED lit timeout. */
    ECfgSetLitLength    = 5,    /**< Setting value of LED lit timeout. */
    ECfgFactoryDefaults = 6,    /**< Restoring factory defaults. */
    
    ECfgEnd,                    /**< Guard. */
};

/**
 * Global application state variable.
 * Stores current state of fiatelka FSM.
 */
static uint8_t state;


/**
 * Helper for waiting to complete (and de-bounce) button click.
 */
void debounce(void)
{
    /* Make substantial delays, and wait until button is un-clicked. */
    do
    {
        _delay_ms(100);
    }
    while (0 == (PINB & _BV(BUT_PIN)));
}

/**
 * Interrupt routine for watchdog timer interrupt.
 * It is supposed to be called when device is in sleep mode
 * (powerdown).
 */
ISR(WDT_vect)
{
    /* Disable further watchdog timer interrupts. */
    MCUSR = 0;
    /* Disable watchdog - if we need it again, we will ask. */
    wdt_disable(); 
}

/**
 * Interrupt routine for external interrupt (button).
 * It is supposed to be called when device is in sleep mode
 * (powerdown).
 * During normal operation (not-configuration mode), button allows
 * user to override FSM state transitions:
 * - make LEDs lit when they are off;
 * - make LEDS to go out, when they are lit.
 */
ISR(INT0_vect)
{
    /* Disable further button interrupts */
    GIMSK = 0;
    /* Wait for button to unclick. */
    debounce();

    /* We assume we are called during one of two
     * sleep states - glow or not-glow */
    if (state == STATE_GLOW)
    {
        /* If LEDs are lit - turn them down. 
         * Go to state "NIGHT". LEDs will be out
         * until next bright-dark transition happens,
         * or another button click. */
        PORTB = 0;
        state = STATE_NIGHT;
    }
    else
    {
        /* If LEDS are off - turn them on.
         * Go to the state "SETUP". It will reset the lit timer. */
        state = STATE_SETUP;
    }
}

/**
 * Enables ADC and takes single conversion value.
 * \return voltage value normalized to 0-1024
 */
uint16_t get_adc(void)
{
    uint16_t vlt;
    /* request conversion start*/
    ADCSRA = 0b11000111;
    
    /* wait for completion */
    do 
    {
        // nothing
    }
    while (ADCSRA & 0b01000000);
    
    /* get conversion result */
    vlt = ADCL;
    vlt |= ((ADCH & 3) << 8);
        
    /* clear interrupt */
    ADCSRA = 0b10010111;
    
    return vlt;
}

/**
 * Blink LEDs given amount of times.
 * Bliker used for visual feedback during configuration mode.
 * \param count how many blinks
 */
void blink_blink(uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i)
    {
        PORTB = _BV(LED_PIN);
        _delay_ms(400);
        PORTB = 0;
        _delay_ms(600);
    }
}

/**
 * Write byte to EEPROM.
 * \param ucAddress address
 * \param ucData data
 */
void EEPROM_write(uint8_t ucAddress, uint8_t ucData)
{
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE))
    ;
    /* Set Programming mode */
    EECR = (0 << EEPM1) | (0 >> EEPM0);
    /* Set up address and data registers */
    EEARL = ucAddress;
    EEDR = ucData;
    /* Write logical one to EEMPE */
    EECR |= (1<<EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1<<EEPE);
}

/**
 * Read byte from EEPROM.
 * \param ucAddress address
 * \return data
 */
uint8_t EEPROM_read(uint8_t ucAddress)
{
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE))
    ;
    /* Set up address register */
    EEARL = ucAddress;
    /* Start eeprom read by writing EERE */
    EECR |= (1<<EERE);
    /* Return data from data register */
    return EEDR;
}

/**
 * Configuration mode - measure voltage and save new threshold value.
 * \param addr_l low byte address of threshold to save
 */
void calibrate(uint8_t addr_l)
{
    /* Wait for button press - signalizing when
     * measurement should be made. */
    do
    {
        _delay_ms(100);
    }
    while (0 != (PINB & _BV(BUT_PIN)));
    
    uint16_t value = get_adc();
    
    EEPROM_write(addr_l, value & 0xFF);
    EEPROM_write(addr_l + 1, value >> 8);
    
    debounce();
}

/**
 * Configuration mode - store LED timer value.
 */
void set_lit_len(void)
{
    uint8_t click_count = 0;
    uint8_t pause_count = 0;
    
    /* Count number of button clicks. 
     * There must be at least one click.
     * No clicks for ~5 seconds - end counting. */
    while ((click_count == 0) || (pause_count < 50))
    {
        _delay_ms(100);
        ++pause_count;
     
        if (0 == (PINB & _BV(BUT_PIN)))
        {
            ++click_count;
            pause_count = 0;
            debounce();
        }
    }
    
    /* Acknowledge to user how many clicks were counted. */
    blink_blink(click_count);
    
    EEPROM_write(EEPROM_LIT_LEN, click_count);
}

/**
 * Start sleep period.
 * Enables power save and enters deep sleep (powerdown) for ~2 seconds.
 * Before sleeping, interrupts are setup that will wake up processor:
 * - watchdog timer;
 * - button press;
 */
void sleeper(void)
{
    /* disable ADC to save power during sleep */
    ADCSRA = 0;
    PRR = 3;
    
    cli();
    wdt_reset();
    WDTCR |= (1<<WDCE) | (1<<WDE);
    WDTCR = (1<<WDTIE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    MCUCR &= 0b11111100;
    GIMSK = 1 << 6;
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();
    
    /* re-enable ADC after sleep */
    PRR = 0;
    ADCSRA = 0b10000111;
}

/**
 * Loop of configuration mode.
 * Configuration mode allow user to change application settings stored
 * in non-volatile memory (EEPROM). User gives his input using button,
 * application gives feedback by blinking LEDs.
 */
void configuration(void)
{
    uint8_t program = 0;
    
    for (;;)
    {
        /* Wait for ~2 seconds, periodically checking if button was clicked. */
        
        for (uint8_t i = 0; i < 20; ++i)
        {
            _delay_ms(100);
            
            if (0 == (PINB & _BV(BUT_PIN)))
            {
                /* wait for button to unclick */
                debounce();
                /* acknowledge which configuration program was selected
                 * it is value of 'program' variable at the moment,
                 * user clicked the button */
                blink_blink(program);
                
                /* perform actions depending on selected program */
                switch (program)
                {
                    case ECfgCalibrateDark:
                        calibrate(EEPROM_DIM_L);
                        break;
                    
                    case ECfgCalibrateBright:
                        calibrate(EEPROM_BRIGHT_L);
                        break;
                    
                    case ECfgGetLitLength:
                    {
                        /* blink stored value */
                        _delay_ms(3000);
                        uint8_t litLength = EEPROM_read(EEPROM_LIT_LEN);
                        blink_blink(litLength);
                        /* in this case we don't do additional blinks on exit
                         * we probably blinked enough already */
                        break;
                    }
                    
                    case ECfgSetLitLength:
                        set_lit_len();
                        /* in this case we don't do additional blinks on exit
                         * we probably blinked enough already */
                        break;
                    
                    case ECfgFactoryDefaults:
                    {
                        /* EEPROM erase */
                        EEPROM_write(EEPROM_DIM_L, 0xFF);
                        EEPROM_write(EEPROM_DIM_H, 0xFF);
                        EEPROM_write(EEPROM_BRIGHT_L, 0xFF);
                        EEPROM_write(EEPROM_BRIGHT_H, 0xFF);
                        EEPROM_write(EEPROM_LIT_LEN, 0xFF);
                        break;
                    }
                    case ECfgAbort:
                    case ECfgExit:
                    default:
                        return;
                }
                
                /* after configuration command is complete, go back
                 * to blinking config menu, starting from begining.*/
                program = 255; // aka. -1
                /* there will be extra 2 seconds of dark because we start
                 * from 'program = 0' */
            }
        }
        
        /* after two seconds change program to the next one */
        ++program;
        if (program >= ECfgEnd)
        {
            program = ECfgExit;
        }
        
        /* tell user which program can be selected next */
        blink_blink(program);
    }
}

/**
 * Application main loop.
 */
int main(void)
{
    /* Reuseable memory:
     * counters and limits*/
    uint16_t counter;
    union {         /* union used to save stack space */
        uint16_t threshold;
        struct{
            uint8_t quater;
            uint8_t length;
        };
    } tl;
    
    /* port configuration
     * - set LED_PIN as output, everything else as input 
     * - set LED_PIN low */
    DDRB = 0b1000;
    PORTB = 0x0;
    
    /* configure ADC for measurement */
    
    /* ADC sra - hue hue hue */
    ADCSRA = 0b10000111;
    /* Vref - Vcc
     * channel 2 - PB4 */
    ADMUX = 0b00000010;
    
   /* check if button is pressed during start, if so
    * enter configuration mode. */
    if (0 == (PINB & _BV(BUT_PIN)))
    {
        debounce();
        configuration();
    }

    state = STATE_BRIGHT;    
    counter = 0;
        
    while (1)
    {

        if (STATE_BRIGHT == state)
        {
            /* sleep ~2 seconds */
            sleeper();
        
            tl.threshold = EEPROM_read(EEPROM_DIM_L);
            tl.threshold |= (EEPROM_read(EEPROM_DIM_H) << 8);
            if (tl.threshold == 0xFFFF)
            {
                tl.threshold = THRESHOLD_DIM;
            }
        
            /* check if dark enough to turn on LEDs */
            uint16_t vlt = get_adc();
            if (vlt <= tl.threshold)
            {
                state = STATE_FADE_IN;
                counter = 1;
            }
        }
        else if (STATE_FADE_IN == state)
        {
            PORTB &= ~(_BV(LED_PIN));
            _delay_loop_1(FAKE_PWM_MAX - counter);
            PORTB |= _BV(LED_PIN);
            _delay_loop_1(counter);
            
            PORTB &= ~(_BV(LED_PIN));
            _delay_loop_1(FAKE_PWM_MAX - counter);
            PORTB |= _BV(LED_PIN);
            _delay_loop_1(counter);
            
            PORTB &= ~(_BV(LED_PIN));
            _delay_loop_1(FAKE_PWM_MAX - counter);
            PORTB |= _BV(LED_PIN);
            _delay_loop_1(counter);
            
            PORTB &= ~(_BV(LED_PIN));
            _delay_loop_1(FAKE_PWM_MAX - counter);
            PORTB |= _BV(LED_PIN);
            _delay_loop_1(counter);
            
            ++counter;
            if (counter == FAKE_PWM_MAX)
            {
                state = STATE_SETUP;
            }
        }
        else if (STATE_SETUP == state)
        {
            PORTB |= _BV(LED_PIN);
            state = STATE_GLOW;
            tl.quater = 0;
            tl.length = EEPROM_read(EEPROM_LIT_LEN);
            if (tl.length == 0xFF)
            {
                tl.length = LIT_LENGTH;
            }
            counter = 0;
        }
        else if (STATE_GLOW == state)
        {            
            sleeper();

            ++counter;
            if (counter >= 449)
            {
                ++tl.quater;
                
                if (tl.quater >= tl.length)
                {
                    state = STATE_FADE_OUT;
                    counter = 1;
                }
                else
                {
                    counter = 0;
                }
            }
        }
        else if (STATE_FADE_OUT == state)
        {
            PORTB &= ~(_BV(LED_PIN));
            _delay_loop_1(counter);
            PORTB |= _BV(LED_PIN);
            _delay_loop_1(FAKE_PWM_MAX - counter);
            
            PORTB &= ~(_BV(LED_PIN));
            _delay_loop_1(counter);
            PORTB |= _BV(LED_PIN);
            _delay_loop_1(FAKE_PWM_MAX - counter);
            
            PORTB &= ~(_BV(LED_PIN));
            _delay_loop_1(counter);
            PORTB |= _BV(LED_PIN);
            _delay_loop_1(FAKE_PWM_MAX - counter);
            
            ++counter;
            if (counter == FAKE_PWM_MAX)
            {
                PORTB &= ~(_BV(LED_PIN));
                state = STATE_NIGHT;
            }
        }
        else if (STATE_NIGHT == state)
        {
            sleeper();
            
            tl.threshold = EEPROM_read(EEPROM_BRIGHT_L);
            tl.threshold |= (EEPROM_read(EEPROM_BRIGHT_H) << 8);
            if (tl.threshold == 0xFFFF)
            {
                tl.threshold = THRESHOLD_BRIGHT;
            }
            
            uint16_t vlt = get_adc();
            if (vlt >= tl.threshold)
            {
                state = STATE_BRIGHT;
            }
        }
        else
        {
            state = STATE_BRIGHT;
        }
    }
}
