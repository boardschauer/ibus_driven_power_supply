#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <inttypes.h>


/*
 * Dieses Programm steuert ein Netzteil (geschaltet über
 * ein Transistor und Relais am PB0 so dass
 * nach einer längeren Inaktivität am IBUS das Netzteil
 * abgeschaltet wird.
 * Erfolgt wieder Kommunikation auf dem IBUS
 * wird das Netzteil augenblicklich eingeschaltet.
 * Drückt der Benutzer die vier Lenkrad-Tasten
 * in richtiger Reihenfolge hintereinander,
 * wird das Netzteil für eine kurze Zeit abgeschaltet
 * (Reset).
 */
 
  void waitfor (unsigned int currdelay)
  {
  	unsigned int counter = 0;
	while (counter != currdelay)
	{
	  //wait (30000 x 4) cycles = wait 120000 cycles
	  _delay_loop_2(5000);
	  counter++;
	}
  }

// bei neueren AVRs andere Bezeichnung fuer die Statusregister, hier ATmega16:
void uart_putc(unsigned char c)
{
    while (!(UCSRA & (1<<UDRE))); /* warten bis Senden moeglich */
    UDR = c;                      /* sende Zeichen */
}

/* puts ist unabhaengig vom Controllertyp */
void uart_puts (char *s)
{
    while (*s)
    {   /* so lange *s != '\0' also ungleich dem "Sting-Endezeichen" */
        uart_putc(*s);
        s++;
    }
}

void uart_init (void)
{
  //UART TX und RX einschalten
  // und RXCIE = Receive complete Interrupt
  UCSRB = (1<<TXEN | 1 << RXEN | 1 << RXCIE);			
  
//  UCSRC = (3<<UCSZ0);	//Asynchron 8N1  
  UCSRC = (3<<UCSZ0) | (2<<UPM0) ;	// Asynchron 8E1
  
  #define UART_BAUD_RATE 9600
  #define UART_BAUD_CALC(UART_BAUD_RATE,F_OSC) ((F_CPU)/((UART_BAUD_RATE)*16L)-1)

  UBRRH=(uint8_t)(UART_BAUD_CALC(UART_BAUD_RATE,F_CPU)>>8);
  UBRRL=(uint8_t)UART_BAUD_CALC(UART_BAUD_RATE,F_CPU);
  // UBRRH=0;
  
  // 6 = 4800
  // 13 = 2400
  // 26 = 1200 baud
  // -> Takt von ca. 537 kHz ???
  // UBRRL=13;  
  
  /* alternativ bei der avr-libc "direkt 16bit" : */
  // UBRR=UART_BAUD_CALC(UART_BAUD_RATE,F_CPU);
}

// Speicherzellen, die in der INT-Routine gesetzt werden:

// Anzahl der Empfangenen Zeichen
volatile uint8_t counter;

// 1=Zeichen empfangen, 0=kein Zeichen empfangen
volatile uint8_t was_received;

// 1=Befehl erkannt, 0=noch kein Befehl erkannt
volatile uint8_t recogn;

#define LAENGE 40
// empfangspuffer
// volatile uint8_t* rec_buf = "1234567890123456789012345678901234567890123456789";
volatile char rec_buf[LAENGE+1];


// Positionszeiger
volatile char* rec_pos;

// Interrupt-Routine, die beim Empfang eines
// Zeichens (über UART RX) aktiviert wird
SIGNAL (SIG_USART0_RX)
{
  if (counter >= LAENGE)
  {
    // buffer overflow
    counter = 0;
    // *(rec_buf) = 0x0;
	rec_pos = rec_buf;
	// *rec_buf = 0;
  }
  
  // prüfen, ob zuletzt ein Befehl erkannt wurde
  // dann Puffer löschen
  if (recogn)
  {
    counter = 0;
	rec_pos = rec_buf;
	recogn = 0;
  }
  
  counter++;
  was_received = 1;
  *rec_pos = UDR;
  rec_pos++;
  *rec_pos = 0x0;
  
  if (PORTB & (1<<1))
    PORTB &= ~(1<<1);
  else
    PORTB |= (1<<1);
}

// radio turnknob rotate left
const char command1[] = {
  0x50, 0x04, 0x68, 0x32, 0x30, 0x3E, 0x00
  };
  
// radio turnknob rotate right
const char command2[] = {
  0x50, 0x04, 0x68, 0x32, 0x31, 0x3F, 0x00 
  };
  
// Button < release
const char command3[] = {
  0x50, 0x04, 0x68, 0x3B, 0x28, 0x2F, 0x00
  };
  
// Button > release
const char command4[] = {
  0x50, 0x04, 0x68, 0x3B, 0x21, 0x26, 0x00
  };
  
const char answ[] = {
  0x68, 0x11, 0x3B, 0x23, 0x62, 0x30, 0x48,
  0x61, 0x6c, 0x6c, 0x6f, 0x20, 0x44, 0x61,
  0x6d, 0x69, 0x61, 0x6e, 0x7b, 0x00
  }; 

uint8_t codep = 0;

// calculates xor checksum for given null terminated
// message.
uint8_t calculate_xor_checksum (char *message)
{
  uint8_t result = 0x0;
  while (*message)
  {
    // byte to byte xor
	result = result ^ *message;
	message++;
  }
  return result;
}

// timeout on ibus before power down
#define IDLE_TIMEOUT 30*100000
// #define IDLE_TIMEOUT 30*100000

// reset duration
#define RESET_DURATION 5000

#define ON_DELAY 5000
  
#define FLASH_ON_TIME 50
#define FLASH_OFF_TIME 500

uint64_t flash_counter;

#define RELAIS_ON PORTB |= (1<<0)
#define RELAIS_OFF PORTB &= ~(1<<0) 
#define LED_ON PORTB |= (1<<1)
#define LED_OFF PORTB &= ~(1<<1)

int relais_is_on;

void relais_off (void)
{
  RELAIS_OFF;
  relais_is_on = 0;
}

void relais_on (void)
{
  RELAIS_ON;
  relais_is_on = 1;
}

int main (void)
{
  // set PORTB for output
  DDRB = 0xFF;
  LED_ON;
  relais_off ();

  rec_pos = rec_buf;
  recogn = 0;
  counter = 0;
  flash_counter = 0;
  
  uart_init ();
  
  waitfor (1000);
  LED_OFF;
  relais_on ();
  // PORTB &= ~(1<<0);
  //  waitfor (RESET_DURATION);
  // PORTB |= (1<<0);
		// reseting whole port B
  // PORTB = (1<<0);		
  

  // interrupts erlauben
  sei ();
  
  uint64_t ibus_idle = 0;
  while (1)
  {
	if (was_received)
	{
	  was_received = 0;
	  ibus_idle = 0;
	  relais_on ();
	  
	  /*
	  
	  if (!strcmp (rec_pos - 6, command1))
	  {
	    recogn = 1;
		PORTB |= (1<<1);
		if (codep == 0)
		  codep = 1;
		else
		{
		  codep = 1;
		  PORTB &= ~((1<<2) | (1<<3) | (1<<4));
		}
	  }
	  
	  if (!strcmp (rec_pos - 6, command2))
	  {
	    recogn = 1;
		PORTB |= (1<<2);
		if (codep == 1)
		  codep = 2;
		else
		{
		  codep = 0;
		  PORTB &= ~((1<<1) | (1<<2) | (1<<3) | (1<<4));
		}
	  }
	  
	  if (!strcmp (rec_pos - 6, command3))
	  {
	    recogn = 1;
		PORTB |= (1<<3);
		if (codep == 2)
		  codep = 3;
		else
		{
		  codep = 0;
		  PORTB &= ~((1<<1) | (1<<2) | (1<<3) | (1<<4));
		}
	  }
	  if (!strcmp (rec_pos - 6, command4))
	  {
	    recogn = 1;
		PORTB |= (1<<4);
		if (codep == 3)
		  codep = 4;
		else
		{
		  codep = 0;
		  PORTB &= ~((1<<1) | (1<<2) | (1<<3) | (1<<4));
		}
	  }
	  
	  // werden die 4 commands hintereinander erkannt,
	  // und zwar in der richtigen Reihenfolge,
	  // wird der Pin 0 vom Port B aus- und wieder eingeschaltet
	  // (reset)
	  if (codep == 4)
	  {
	    PORTB &= ~(1<<0);
		waitfor (RESET_DURATION);
		// PORTB |= (1<<0);
		// reseting whole port B
		PORTB |= (1<<0);		
      }
	  // uart_puts (rec_pos-1);
	  */
	}
	else
	{
	  ibus_idle++;
	  if (ibus_idle > IDLE_TIMEOUT)
	  {
	    ibus_idle = IDLE_TIMEOUT;
		relais_off ();
	  }
	}
	
	flash_counter++;
	if ((flash_counter % 32768) == 0)
	{
	  if (relais_is_on)
	  {
		LED_OFF;
	  }
	  else
	  {
	    LED_ON;
	  }
	}
	if (((flash_counter-1024) % 32768) == 0)
	{
	  if (relais_is_on)
	  {
		LED_ON;
	  }
	  else
	  {
	    LED_OFF;
	  }
	}
	
	
  }
  
  
  return 1;
}

