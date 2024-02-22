#include <LSA08.h>

#define LSA08_USE_UART
 
#define _XTAL_FREQ 20000000                           //definition of oscillator crystal frequency
#define BAUDRATE 57600                                    //UART Baudrate
#define LSA08_UE PORTCbits.RC5                       //any digital output pin for UE pin of LSA08
#define LSA08_UE_TRIS TRISCbits.TRISC5           //corresponding direction tristate .
 
//define the LSA08 address------------------------------------------------------
unsigned char LSA08_ADD= 0x01;
 
//define the UART variable-----------------------------------------------------------
unsigned char ERR_FLAG=0;
 
//Choose the UART Mode used-----------------------------------------------------
//#define LSA08_UARTMODE 0
//#define LSA08_UARTMODE 1
#define LSA08_UARTMODE 2
#include <pic.h>
#include <LSA08.h>

void UART_INIT(void);
void UART_SEND(char data);
unsigned char UART_REC(void);
void UART_DUMP(void);


void main (void) 
{

//user main program here....-----------------------------------------------------

    while(1)
   {

    }//while
}//main 

void UART_INIT(void)
{ 
    //set port direction
  TRISCbits.TRISC7=1;   //RX
  TRISCbits.TRISC6=0;   //TX
 
#ifdef LSA08_UE_TRIS
  LSA08_UE_TRIS=0;
#endif 
  // Initialize UART.
  TXSTAbits.BRGH = 1;               // Select high speed baud rate.
  BAUDCTLbits.BRG16=1;           // Baud 16bits
 
  SPBRG =0x56; SPBRGH=0x00; //57600
  RCSTAbits.SPEN = 1;                 // Enable serial port.
  TXSTAbits.TXEN = 1;
  RCSTAbits.CREN = 1; 
}
void UART_SEND(char data)
{
 while(!TRMT) ; //wait for previous transmit completion
 TXREG=data;
}
unsigned char UART_REC(void)                                            
{
  unsigned long waitcount=0;
  unsigned char rec_data;
 
  if(RCSTAbits.OERR){
      RCSTAbits.CREN=0;
      RCSTAbits.CREN=1;
      ERR_FLAG=1;
      return(255);
  }
  // Read the received data.
  while (RCIF == 0) //wait for data
  {
      waitcount++;
      if (waitcount > 15000){ //    break if wait too long, no incoming data
          ERR_FLAG=1;
          return (255); //no line
      }
  }
 
  rec_data = RCREG;
 
  if (FERR == 1) {
      while(RCIF) rec_data=RCREG;
      ERR_FLAG=1;
      return (255);
  }
  else{
      ERR_FLAG=0;
      return rec_data;                                                                    //return the data received
  }
}
 
void UART_DUMP(void)
{
  unsigned char dump; 

  while (RCIF == 1) //wait for data
  {
        dump=UART_REC();
  }
}
