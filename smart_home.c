#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>


#define ALARM_TIMEOUT_MS 60000
#define KEYPAD_DEBOUNCE_MS 200

#define ROW_MASK 0b00001111      // PB0–PB3
#define COL_MASK 0b11100000      // PB5–PB7

#define TEMP_HYSTERESIS 2        // ±2 °C


typedef enum {
    HOME_IDLE,
    HOME_ARMED,
    HOME_ALARM
} system_state_t;

volatile system_state_t system_state = HOME_IDLE;


// Alarm
volatile uint16_t alarm_timer = ALARM_TIMEOUT_MS;
volatile uint8_t buzzer_active = 0;

uint8_t passcode[4] = {5, 1, 2, 0};
uint8_t user_input[4];
uint8_t input_index = 0;

// Thermostat
volatile uint16_t adc_value = 0;
volatile int current_temp = 0;
volatile int set_temp = 25;

// Display
uint8_t display_digit = 0;


void display(char pos, char val) {
    PORTA &= 0b11110000;
    PORTC = 0;

    switch (val) {
        case 0: PORTC = 0b00111111; break;
        case 1: PORTC = 0b00000110; break;
        case 2: PORTC = 0b01011011; break;
        case 3: PORTC = 0b01001111; break;
        case 4: PORTC = 0b01100110; break;
        case 5: PORTC = 0b01101101; break;
        case 6: PORTC = 0b01111101; break;
        case 7: PORTC = 0b00000111; break;
        case 8: PORTC = 0b01111111; break;
        case 9: PORTC = 0b01100111; break;
        case '*': PORTC = 0b01000000; break;
        case 'A': PORTC = 0b01110111; break;
        case 'L': PORTC = 0b00111000; break;
    }

    PORTA |= (1 << (pos - 1));
    _delay_ms(3);
}

//ADC
void init_adc(void) {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t read_adc(uint8_t ch) {
    ADMUX = (ADMUX & 0xE0) | ch;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

//buzzer
void init_buzzer(void) {
    DDRD |= (1 << PD4);
    TCCR1A = (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
}

void buzzer_on(uint16_t freq) {
    uint16_t top = F_CPU / (8 * freq) - 1;
    ICR1 = top;
    OCR1B = top / 2;
}

void buzzer_off(void) {
    OCR1B = 0;
}


void init_timer0(void) {
    TCCR0 = (1 << CS01) | (1 << CS00); // prescaler 64
    TIMSK |= (1 << TOIE0);
}

//keypad
void init_keypad(void) {
    DDRB = COL_MASK;
    DDRB &= ~ROW_MASK;
    PORTB |= ROW_MASK | COL_MASK;
}

uint8_t scan_keypad(void) {
    for (uint8_t col = 0; col < 3; col++) {
        PORTB &= ~(1 << (col + 5));
        _delay_us(5);

        uint8_t row = PINB & ROW_MASK;
        if (row != ROW_MASK) {
            PORTB |= COL_MASK;

            if (row == 0b1110) return col + 1;
            if (row == 0b1101) return col + 4;
            if (row == 0b1011) return col + 7;
            if (row == 0b0111) return (col == 1) ? 0 : '*';
        }
    }
    return 0xFF;
}


ISR(TIMER0_OVF_vect) {
    static uint16_t ms = 0;
    ms++;

    if (ms >= 16) {
        ms = 0;

        // Alarm countdown
        if (system_state == HOME_ARMED && alarm_timer > 0) {
            alarm_timer--;
            if (alarm_timer == 0) {
                system_state = HOME_ALARM;
                buzzer_active = 1;
            }
        }

        // Thermostat sampling
        adc_value = read_adc(7);
        current_temp = (adc_value * 5.0 / 1024.0) * 100;

        adc_value = read_adc(6);
        set_temp = 20 + (adc_value / 20);

        if (current_temp < set_temp - TEMP_HYSTERESIS)
            PORTD |= (1 << PD5);
        if (current_temp > set_temp + TEMP_HYSTERESIS)
            PORTD &= ~(1 << PD5);
    }
}

//password
void check_passcode(void) {
    uint8_t ok = 1;
    for (uint8_t i = 0; i < 4; i++) {
        if (user_input[i] != passcode[i]) {
            ok = 0;
            break;
        }
    }

    if (ok) {
        system_state = HOME_IDLE;
        alarm_timer = ALARM_TIMEOUT_MS;
        buzzer_active = 0;
    }
    input_index = 0;
}


int main(void) {
    DDRA = 0b00001111;
    DDRC = 0xFF;
    DDRD |= (1 << PD5);

    init_adc();
    init_buzzer();
    init_keypad();
    init_timer0();

    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);

    sei();

    while (1) {
        if (!(PIND & (1 << PD2)) && system_state == HOME_IDLE) {
            system_state = HOME_ARMED;
            alarm_timer = ALARM_TIMEOUT_MS;
        }

        if (system_state == HOME_ARMED) {
            uint8_t key = scan_keypad();
            if (key != 0xFF) {
                user_input[input_index++] = key;
                if (input_index == 4) check_passcode();
                _delay_ms(KEYPAD_DEBOUNCE_MS);
            }
        }

        if (system_state == HOME_ALARM) {
            buzzer_on(1000);
        } else {
            buzzer_off();
        }

        // Display logic
        display_digit = (display_digit % 4) + 1;

        if (system_state == HOME_ARMED)
            display(display_digit, '*');
        else if (system_state == HOME_ALARM)
            display(display_digit, 'A');
        else
            display(display_digit, current_temp / (display_digit == 1 ? 1 :
                                                    display_digit == 2 ? 10 :
                                                    display_digit == 3 ? 100 : 1000) % 10);
    }
}
