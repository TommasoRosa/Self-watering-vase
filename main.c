
#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>

//#include <avr/iom328p.h> //included only for the sake of the IDE not being able to otherwise fetch the #defines from the correct file

#include "LCD_lib/lcd.c"
#include "RTC_lib/RTC.c"
#include "I2C.c"

#include <avr/pgmspace.h>

#define ENCODER_PORT PORTC
#define ENCODER_DDR DDRC
#define ENC_A_PIN 0
#define ENC_B_PIN 1
#define ENC_BUTTON 1

#define SECOND_MOTOR 0

#define MOTOR_PIN 0
#define MOTOR_PORT PORTB
#define MOTOR_DDR DDRB
#define MOTOR_ON PORTB |= _BV(MOTOR_PIN)
#define MOTOR_OFF PORTB &= ~_BV(MOTOR_PIN)
#define MOTOR_TOGGLE PINB |= _BV(MOTOR_PIN)
#define SECOND_MOTOR_ON if (SECOND_MOTOR) PORTB |= _BV(LIGHT_PIN)
#define SECOND_MOTOR_OFF if (SECOND_MOTOR) PORTB &= ~_BV(LIGHT_PIN)


#define LIGHT_ON if (!SECOND_MOTOR) PORTB |= _BV(LIGHT_PIN)
#define LIGHT_OFF if (!SECOND_MOTOR) PORTB &= ~_BV(LIGHT_PIN)
#define LIGHT_DDR DDRB
#define LIGHT_PIN 2
#define LIGHT_PORT PORTB
#define LIGHT_TOGGLE PINB |= _BV(LIGHT_PIN)

#define BACKLIGHT_BIT 7
#define BACKLIGHT_PORT PORTB
#define BACKLIGHT_DDR DDRB
#define BACKLIGHT_LOW PORTB &= ~(_BV(BACKLIGHT_BIT))
#define BACKLIGHT_HIGH PORTB |= _BV(BACKLIGHT_BIT)

#define LCD_REFRESH_RATE (120)

#define MILLIS_CALIBRATION 5
#define MICROS_CALIBRATION 56

#define ML_2_MS 186

#define wrap_around(var, num) var += (var>=num)? (-num) :(var<0) ? num :0

//"L" is easier to remember than PSTR
#define L(x) PSTR(x)
const char text[] PROGMEM ={"Hello World"};
const char t2[] PROGMEM={"-=-"};
const char day_name[][7] PROGMEM = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun", "Mon"};
//const char monthName[][12] PROGMEM = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

//* ------EEPROM ADDRESSES-----------*//
#define LIGHT_START_ADDRESS 10
#define LIGHT_STOP_ADDRESS 12
#define WATER_SCHEDULE_HOUR_BASE_ADDRESS 18
#define WATER_AMOUNT_ADDRESS 14
#define DAY_ADDRESS 16

volatile uint8_t enc_button_pressed=0;
volatile int8_t enc_dir=0;

volatile uint8_t tick_tock=0;
volatile uint8_t debounced_a=0;
volatile uint8_t debounced_b=0;

typedef struct{
	int8_t hour;
	uint8_t on;
	uint8_t is_being_changed;
	uint8_t done_today;
}day_schedule_t;

day_schedule_t water_schedule[7]={{0},{0},{0},{0},{0},{0},{0}};


uint16_t water_timer=0;
uint16_t water_timer_max=0;
int16_t water_ml=0;

int8_t light_start_time=0;
int8_t light_stop_time=0;
int8_t light_is_on=0;
uint8_t light_enabled=0;

typedef struct{
	int16_t millis;
	int8_t seconds;
	int8_t minutes;
	int8_t hours;
	int8_t days;
	int8_t has_changed;
	int8_t is_being_changed;
	char whole_time[9];
} time_t;

time_t time;

void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);
void restore_vars_from_EEPROM();
void EEPROM_compare(unsigned int address, unsigned char data);

void update_the_stuff(uint8_t tick_tock);

void format_2_string(time_t* time){
	if (time->seconds<10){
		itoa(time->seconds, &time->whole_time[7],10);
		time->whole_time[6]='0';
	}
	else itoa(time->seconds, &time->whole_time[6],10);

	if (time->minutes<10){
		itoa(time->minutes, &time->whole_time[4],10);
		time->whole_time[3]='0';
	}
	else itoa(time->minutes, &time->whole_time[3],10);

	if (time->hours<10){
		itoa(time->hours, &time->whole_time[1],10);
		time->whole_time[0]='0';
	}
	else ltoa(time->hours, &time->whole_time[0],10);

	time->whole_time[2]=':';
	time->whole_time[5]=':';
}

void update_clock(uint8_t elapsed_millis){
	if (time.is_being_changed) return;

	time.millis += elapsed_millis;
	if (time.millis<(1000)) return;
	time.millis -= 1000;

	int8_t old_hour = time.hours;

	rtc_get_time();
	time.seconds = (int8_t) _tm.sec;
	time.minutes = (int8_t) _tm.min;
	time.hours = (int8_t) _tm.hour;
	time.days = (int8_t) _tm.wday;
	if (time.days==7) time.days=0; //wrap over for when the rtc automatically goes from sunday (6) to monday (7), which I instead count as 0

	if (time.hours==0 && old_hour==23)
		water_schedule[time.days].done_today=0; //as the old day sets, clear the "done_today" flag, so that it can fly another day

	format_2_string(&time);
	time.has_changed=1;


}

#define NUM_MENUS 4





void update_menu(int8_t e_dir, uint8_t b_pressed, uint8_t reset_to_main_menu){

	static struct{
		int8_t current;
		uint8_t level;
		uint8_t cursor_on;
		uint8_t clear_is_due;
		uint8_t was_cleared;
	}menu = {.current=0, .level=0, .cursor_on=0, .clear_is_due=1};
	static int8_t item=0; //goes 0 to 15 for first row, 16 to 31 second row
	static int8_t cursor_pos=0; //goes 0 to 15 for first row, 16 to 31 second row


	lcd_off(); //switch off the lcd, and switch it on at the end

	//enable cursor for menu levels above 0
	if ((menu.level > 0) && !menu.cursor_on) {lcd_enable_cursor();	menu.cursor_on=1;}
	if ((menu.level == 0) && menu.cursor_on) {lcd_disable_cursor();	menu.cursor_on=0;}

	//default level 0 actions
	if (menu.level == 0) {
		//e_dir: 1=left -1=right 0=nothing
		menu.current += e_dir;
		wrap_around(menu.current, NUM_MENUS);
		//clear e_dir and the screen to avoid further interactions

		if (e_dir) menu.clear_is_due=1;
		e_dir=0;
		//if button is pressed, move to level 1 and clear cursor_pos
		if (b_pressed) {
			menu.level=1;
			menu.clear_is_due=1;
			item=0;
		}
		//clear b_pressed to avoid further button interactions this time around
		b_pressed=0;
	}

	if (menu.clear_is_due) { lcd_clear_f(); menu.clear_is_due=0; menu.was_cleared=1;}

	if (reset_to_main_menu) menu.current=0;

	//now for all the menus
	switch (menu.current){
	case 0: //just display the time...evventualy pressing the button should make it possible to change the time
		//_ddd__hh:mm:ss__
		//_W:hh__L:hh-hh__
		if (time.has_changed || time.is_being_changed || menu.was_cleared){
			char temp[3]={0};

			//lcd_set_cursor(0,0);  //debug help
			//lcd_puts(itoa(time.days, temp, 10));
			lcd_set_cursor(1,0);
			lcd_puts_P(day_name[time.days]);
			lcd_set_cursor(6,0);
			lcd_puts(time.whole_time);

			if (menu.level==0){
				lcd_set_cursor(0,1);
				lcd_puts_P(L("W:"));
				if (water_schedule[time.days].hour==24) lcd_puts_P(L("//"));
				else{
					temp[0]='0' + (water_schedule[time.days].hour / 10); //todo check if this works
					temp[1]='0' + (water_schedule[time.days].hour % 10);
					lcd_puts(temp);
				}

				lcd_set_cursor(8,1);
				if (light_stop_time==light_start_time) lcd_puts_P(L("L: off"));
				else{

					lcd_puts_P(L("L:"));
					temp[0]='0' + (light_start_time / 10);
					temp[1]='0' + (light_start_time % 10);
					lcd_puts(temp);
					lcd_puts_P(L("-"));
					temp[0]='0' + (light_stop_time / 10);
					temp[1]='0' + (light_stop_time % 10);
					lcd_puts(temp);
				}
			}
			else if (menu.level==1){

				lcd_set_cursor(0,1);
				lcd_puts_P(L("           Back  "));
			}
			time.has_changed=0;
		}

		//change the time
		if (menu.level==1){
			item+=e_dir;
			e_dir=0;
			wrap_around(item, 5);
			//choose what to show based on cursor pos
			switch(item){
			case 0: cursor_pos=1; break; //set on hour
			case 1: cursor_pos=6; break; //set on hour
			case 2: cursor_pos=9; break;//set on minutes
			case 3: cursor_pos=12; break;//set on seconds
			case 4: cursor_pos=16 + 11; break;//exit to level 0
			}
			//choose what to do for button press based on cursor pos
			if (b_pressed) switch(item){
			case 0:
			case 1:
			case 2:
			case 3: menu.level=2; break;
			case 4: menu.level=0; menu.clear_is_due=1; break;//exit to level 0
			}
			b_pressed=0;
		}

		if(menu.level==2){
			time.is_being_changed=1;
			lcd_enable_blinking();

			switch(item){
			case 0: time.days+=  e_dir; 	wrap_around(time.days, 7); 	break; //set on hour
			case 1: time.hours+=  e_dir; 	wrap_around(time.hours, 24); 	break; //set on hour
			case 2: time.minutes+=e_dir;	wrap_around(time.minutes, 60); 	break;	//set on minutes
			case 3: time.seconds+=e_dir;	wrap_around(time.seconds, 60); 	break;//set on seconds
			}
			if (b_pressed) {
				menu.level=1;
				lcd_disable_blinking();
				_tm.sec=time.seconds;
				_tm.min=time.minutes;
				_tm.hour=time.hours;
				_tm.wday=time.days;
				rtc_set_time(&_tm);
				time.is_being_changed=0;
			}
			b_pressed=0;
			e_dir=0;
			format_2_string(&time);
			//flag for an update
		}


		break;

	case 1: //watering settings
		//format:
		//_water_schedule_
		//DAY:_water_at_HH

		//print all of the neccessary info
		if (menu.was_cleared){
			lcd_set_cursor(0,0);
			lcd_puts_P(L(" Water Schedule "));

			//display level 1 or 2 settings if on that level
			if (menu.level>=1){
				lcd_set_cursor(0,1);
				//8th item: back option
				if (item==7) lcd_puts_P(L("Back            "));
				//1-7th items: day with corresponding schedule
				else {
					//display the day
					lcd_puts_P(day_name[item]);
					//if water is scheduled, write ": Water at " and the hour
					if (water_schedule[item].on){
						lcd_puts_P(L(": Water at "));
						//get hour in string format
						char temp[3]={0};
						temp[0]='0' + (water_schedule[item].hour / 10);
						temp[1]='0' + (water_schedule[item].hour % 10);
						lcd_puts(temp);
					}
					else lcd_puts_P(L(": No water"));
				}
			}
		}

		if (menu.level==1){
			item+=e_dir;
			wrap_around(item, 8);

			if (b_pressed){
				if (item==7) {menu.level=0; menu.clear_is_due=1;}
				else {
					menu.level=2;
					lcd_enable_blinking();
				}
			}

			cursor_pos=16+0;
			if (e_dir!=0) menu.clear_is_due=1;
			b_pressed=0;
			e_dir=0;

		}

		if (menu.level==2){
			water_schedule[item].is_being_changed=1;
			water_schedule[item].hour+=e_dir;
			wrap_around(water_schedule[item].hour, 25);

			//use hour==24 as an off condition
			if(water_schedule[item].hour==24) water_schedule[item].on=0;
			else water_schedule[item].on=1;

			if(b_pressed) {
				menu.level=1;
				water_schedule[item].is_being_changed=0;
				EEPROM_compare(WATER_SCHEDULE_HOUR_BASE_ADDRESS + item , water_schedule[item].hour); ///////////////////////////////////////////////////////
				lcd_disable_blinking();
				menu.clear_is_due=1;
			}

			if (e_dir!=0) menu.clear_is_due=1;
			b_pressed=0;
			e_dir=0;
			cursor_pos=16+4;
		}


		break;

	case 2: //water amount setting
		//draw
		if(menu.was_cleared){
			char temp[6];
			ltoa(water_ml, temp, 10);

			lcd_set_cursor(0,0);
			lcd_puts_P(L("  Water Amount  "));

			lcd_set_cursor(0,1);
			lcd_puts(temp);
			lcd_puts_P(L(" ml"));
		}

		if (menu.level==1){
			if(e_dir){
				//fine control...increase water amount by 50 above 100ml, and by 10 under 100ml
				if (water_ml==100) water_ml+= (e_dir==1) ? 50 : -10;
				else if((0<water_ml) && (water_ml<100)) water_ml+=e_dir*10; //todo check this
				else water_ml+=e_dir*50;
				wrap_around(water_ml, 1000);
				menu.clear_is_due=1;
			}
			if(b_pressed) {
				menu.level=0;
				menu.clear_is_due=1;
				EEPROM_compare(WATER_AMOUNT_ADDRESS , water_ml / 10); ////////////////////////////////////////////////////////////////
			}

			cursor_pos=16;
			b_pressed=0;
			e_dir=0;
		}

		break;

	case 3: //light settings, should probably be the same as above...could merge...
		//___light_menu___
		//start:dd_stop:dd
		if(menu.was_cleared){
			char temp1[3], temp2[3];
			itoa(light_start_time, temp1, 10);
			itoa(light_stop_time, temp2, 10);

			lcd_set_cursor(0,0);
			lcd_puts_P(L("   Light Menu   "));

			if(item==2){ lcd_set_cursor(0,1); lcd_puts_P(L("Back")); }
			else{
				lcd_set_cursor(0,1);
				lcd_puts_P(L("Start:"));
				lcd_puts(temp1);

				lcd_set_cursor(9,1);
				lcd_puts_P(L("Stop:"));
				lcd_puts(temp2);
			}
		}

		if(menu.level==1){
			if(e_dir){
				item+=e_dir;
				wrap_around(item, 3);
				menu.clear_is_due=1;
			}
			if(b_pressed){
				if (item==2) {menu.level=0; item=0;}
				else menu.level=2;
				menu.clear_is_due=1;

			}

			cursor_pos= (item==2) ? 16 :( (item==1) ? 14+16 : 6+16);
			e_dir=0;
			b_pressed=0;
		}

		if(menu.level==2){
			//lcd_enable_blinking();
			if(e_dir){
				if (item==0) { light_start_time+=e_dir; wrap_around(light_start_time, 24); }
				if (item==1) { light_stop_time+=e_dir;  wrap_around(light_stop_time, 24);  }
				menu.clear_is_due=1;
			}

			if(b_pressed){
				menu.level=1;
				//lcd_disable_blinking();
				if (item==0) EEPROM_compare(LIGHT_START_ADDRESS, light_start_time); ////////////////////////////////////////////////////////////////
				if (item==1) EEPROM_compare(LIGHT_STOP_ADDRESS , light_stop_time ); ////////////////////////////////////////////////////////////////
				menu.clear_is_due=1;
			}

			e_dir=0;
			b_pressed=0;
		}

		break;

	}

	if (cursor_pos<16) lcd_set_cursor(cursor_pos,0);
	else lcd_set_cursor(cursor_pos-16,1);

	lcd_on();
	menu.was_cleared=0;
}

////////////////////// M A I N ////////////////////////////////////////////////////////////////

const char TIME__[] = __TIME__;
//const char DATE__[] = __DATE__;

int main(void) {
	uint16_t millis1=0;
	uint16_t millis2=0;


	MOTOR_PORT |= _BV(MOTOR_PIN); //set port to 0, ready to pull-down if needed
	LIGHT_PORT |= _BV(LIGHT_PIN); //ditto above
	BACKLIGHT_PORT |= _BV(BACKLIGHT_BIT); //ditto above
	BACKLIGHT_HIGH;
	//LIGHT_ON;
	MOTOR_OFF;

	LIGHT_OFF;
	//motor_startup();

	//set timer 2 for timekeeping and non-blocking delays
	//set CTC mode (compare on match)
	TCCR2A = (1<<WGM20) | (1<<WGM21);
	//at 8Mhz, a prescaler of 128 with compare of 125 gives an exact 4ms interrupt (internally divides 8Mhz by 2)
	TCCR2B = (1<<CS22) | (0<<CS21) | (1<<CS20); //perscaler 128 for 4ms interrupt timer
	//TCCR2B = (0<<CS22) | (1<<CS21) | (1<<CS20); //perscaler 32 for 1ms interrupt timer
	//set the compare value
	OCR2A = 125;
	//enable and setup interrupt on match with OCR2A
	TIMSK2 = (1<<OCIE2A);


	//setup encoder pins interrupts
	//set pins as inputs
	DDRB &= ~(1<<ENC_BUTTON) ;
	DDRC &= ~(1<<ENC_B_PIN) & ~(1<<ENC_A_PIN);
	//enable internal pullups
	PORTB |= (1<<ENC_BUTTON) ;
	PORTC |=  (1<<ENC_B_PIN) | (1<<ENC_A_PIN);
	//enable INT on pins with INT[0:7] and INT[8:14]
	PCICR = (1<<PCIE0) | (1<<PCIE1);
	//set which pins are to be checked for events
	PCMSK1 = (1<<ENC_A_PIN) ;
	PCMSK0 = (1<<ENC_BUTTON);

	restore_vars_from_EEPROM();


	lcd_init();
	lcd_puts_P(L("{ Plant  Saver }"));
	_delay_ms(1000);
	lcd_clear_top();

	//enable all interrupts
	sei();

	time.has_changed = 0;
	time.millis = 0;



	/*

	 //

	*/
	rtc_init();

	time.seconds = atoi(&TIME__[6]) + 4;
	time.minutes = atoi(&TIME__[3]);
	time.hours = atoi(&TIME__[0]);
	time.days = 6;
	//if the cause of the reset was due to programming, update the date
	/*
	if  (!((MCUSR >> PORF) & 0x01)) { // todo: check if this works
		_tm.sec=time.seconds;
		_tm.min=time.minutes;
		_tm.hour=time.hours;
		_tm.wday=time.days;
		rtc_set_time(&_tm);
	}
	*/
	rtc_get_time();
	time.seconds = (int8_t) _tm.sec;
	time.minutes = (int8_t) _tm.min;
	time.hours = (int8_t) _tm.hour;
	time.days = (int8_t) _tm.wday;


	format_2_string(&time);

	while (1) {

		if (tick_tock){
			millis1 += tick_tock;
			millis2 += tick_tock;
			update_clock(tick_tock);
			update_the_stuff(tick_tock);
			tick_tock=0;
		}


		if (millis2 < LCD_REFRESH_RATE){
			millis2 -= LCD_REFRESH_RATE;

			//if (enc_button_pressed) MOTOR_TOGGLE;

			update_menu(enc_dir, enc_button_pressed, 0);

			enc_dir=0;
			enc_button_pressed=0;
		}

	}
	return 0;
}


ISR(TIMER2_COMPA_vect){
	debounced_b=0;
	debounced_a=0;
	tick_tock+=4;
}

ISR(PCINT0_vect){ //todo: simplify this stuff
	//immediately get PINB
	uint8_t pin_b = PINB;

	if (debounced_a==0){
		debounced_a++;

		static uint8_t  old_button=1;
		uint8_t  enc_button=1;

		enc_button 	= (pin_b>>ENC_BUTTON) & 0x01;

		if (enc_button!=old_button){
			old_button=enc_button;
			enc_button_pressed=!enc_button;
		}
	}
}

ISR(PCINT1_vect){
	//immediately get PINC
	uint8_t pin_c = PINC;

	if (debounced_b==0){
		debounced_b++;

		static uint8_t old_a=0;
		uint8_t enc_a=0, enc_b=0;

		enc_a 		= (pin_c>>ENC_A_PIN) & 0x01;
		enc_b 		= (pin_c>>ENC_B_PIN) & 0x01;

		if((enc_a != old_a) & enc_a){
			if ((enc_a != enc_b)) enc_dir=1;
			else enc_dir=-1;
		}
		old_a = enc_a;
	}
}

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

void update_the_stuff(uint8_t tick_tock){
	uint8_t today=time.days;
	static int32_t water_timer=-1; //-1:disabled   positive or zero: activated



	//if the time is right, and the light is not on yet, switch it on
	if (light_start_time<light_stop_time){ //case 1
		if ( (light_start_time<=time.hours) && (time.hours<light_stop_time) && !light_is_on && light_start_time!=light_stop_time){
			 LIGHT_ON;
			light_is_on=1;
		}
		//if it's outside the "on" hours, and the light is on, switch it off
		if ( ((time.hours<light_start_time) || (time.hours>=light_stop_time)) && light_is_on){
			LIGHT_OFF;
			light_is_on=0;
		}
	}
	else { //case 2 when it starts the first day and goes on till the next day
		if ( (light_start_time<=time.hours) && !light_is_on && light_start_time!=light_stop_time){
			LIGHT_ON;
			light_is_on=1;
		}
		if ( (light_stop_time<=time.hours) && (time.hours<light_start_time) && light_is_on){
			LIGHT_OFF;
			light_is_on=0;
		}
	}

	if (light_start_time==light_stop_time){
		LIGHT_OFF;
		light_is_on=0;
	}




	//if the time is right, if today it should water, and has not watered previously today...water
	if (!water_schedule[today].is_being_changed){ //only water once things have been set
		if ((water_schedule[today].hour == time.hours) && !(water_schedule[today].done_today) && water_schedule[today].on){
			MOTOR_ON;
			SECOND_MOTOR_ON;
			water_schedule[today].done_today=1;
			//enable the water timer
			water_timer=0;
		}

		if (water_timer != -1){
			water_timer+= tick_tock;

			if (water_timer >= ((long)water_ml*ML_2_MS)) {
				MOTOR_OFF;
				SECOND_MOTOR_OFF;
				water_timer=-1; //disable timer
			}
		}
	}


}


void restore_vars_from_EEPROM(){
	light_start_time=EEPROM_read(LIGHT_START_ADDRESS);
	light_stop_time=EEPROM_read(LIGHT_STOP_ADDRESS);


	water_ml = EEPROM_read(WATER_AMOUNT_ADDRESS) * 10;

	for (uint8_t i=0; i<7; i++){
		water_schedule[i].hour=EEPROM_read(WATER_SCHEDULE_HOUR_BASE_ADDRESS+i);
		//water_schedule[i].hour=24;
		//check if water should be on or off that day
		if (water_schedule[i].hour==24) water_schedule[i].on=0;
		else water_schedule[i].on=1;
	}

/*
	EEPROM_compare(WATER_AMOUNT_ADDRESS, 20);
	EEPROM_compare(LIGHT_START_ADDRESS, 7);
	EEPROM_compare(LIGHT_STOP_ADDRESS, 10);
	for (uint8_t i=0; i<7; i++) EEPROM_compare(WATER_SCHEDULE_HOUR_BASE_ADDRESS+i, 24);
	*/

}
void EEPROM_compare(unsigned int address, unsigned char data){

	uint8_t temp=EEPROM_read(address);
	if (temp!=data) EEPROM_write(address, data);
}

void EEPROM_write(unsigned int uiAddress, unsigned char ucData){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address and Data Registers */
	EEAR = uiAddress;
	EEDR = ucData;

	cli(); //the next two intrsuctions need to happen immediately one after the other, or else the write won't work
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
	sei();
}

unsigned char EEPROM_read(unsigned int uiAddress){ //should not tecnically need to be atomic
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from Data Register */
	return EEDR;
}






















