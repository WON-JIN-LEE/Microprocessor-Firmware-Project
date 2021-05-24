#include <avr/io.h> 
#include <avr/interrupt.h> 
#include <util/delay.h> 
#include "lcd.h"


void init_serial(void) ;  //  Serial �����Ʈ �ʱ�ȭ
void SerialPutChar(char ch);
void SerialPutString(char str[]);

static volatile  char  rdata = 0,  recv_cnt = 0, new_recv_flag = 0  ;                
static volatile  char  recv_data[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0};  

static volatile char Cmd_Message_1[] = { "stop" } ;     //  Blutooth Command
static volatile char Cmd_Message_2[] = { "start" } ;  
static volatile char Cmd_Message_3[] = { "pwm_week" } ;  
static volatile char Cmd_Message_4[] = { "pwm_mid" } ;  
static volatile char Cmd_Message_5[] = { "pwm_strong" } ;  
static volatile char Cmd_Message_6[] = { "d=" } ;  
static volatile char Cmd_Message_7[] = { "servomotor_a" } ; 
static volatile char Cmd_Message_8[] = { "servomotor_ma" } ;
static volatile char Cmd_Message_9[] = { "servomotor_rco"} ;
static volatile char Cmd_Message_10[] = { "servomotor_left"} ;

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
char AscToNum(char Num);
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 


void msec_delay(int n)  ;   // msec ���� �ð�����
void usec_delay(int n)  ;   // usec ���� �ð�����
unsigned char Time_Delay_Polling( unsigned short d_time ) ;   // �ð����� üũ�Լ�(�������)
unsigned short step=0;
static volatile short     Servi_time = 150 ;      


static volatile unsigned short    distance_1 = 50, distance_2 = 50,  distance_3 = 50, sensor_count = 0, active_sensor_flag = 0  ;
static volatile unsigned short    distance_1_old = 0, distance_2_old = 0,  distance_3_old = 0 ;
unsigned short main_dist1 = 100 ,main_dist2 = 100 ,main_dist3 = 100  ;
static volatile short     Limited_d = 30 ;      // ���ѰŸ�

void DC_Motor_Run_Fwd( short duty );    // DC ���� ��ȸ��(PWM����) �Լ�
void DC_Motor_Stop( void );             // DC ���� ���� �Լ�  
static volatile short  Vmax = 0 ; 

void Servo_Move( short sv_pos_cmd ) ;
static volatile short     Servo_Pos_CMD = 0 ;    // ���� ��ġ ��� ( ���� : 0 - 180,  ����:  �� )
static volatile short     Pos_max = 125;        // ���� �ִ� ��ġ ��� ( 180 �� )
static volatile short     Pos_min = 45 ;          // ���� �ּ� ��ġ ��� ( 0   �� )
static volatile short    Pos_center=90;      //  ���� 

int main() 
{   
	 unsigned short duty = 0, servo_control=0;
 	 char    eq_count1=0, eq_count2=0, eq_count3=0, eq_count4=0, eq_count5=0, eq_count6 = 0,eq_count7 = 0,eq_count8 = 0,eq_count9 = 0,eq_count10 = 0,cmd_data = 0xFF  ; 
	 unsigned char   i=0 ; //for��
	short  servo_duty=0; 
	 
////LED ��Ʈ
DDRA |=0x70;
PORTA |=0x70; //LED off

/////////////////////////////��������

	DDRB |= 0x80;    //  PWM ��Ʈ: OC2( PB7 ) ��¼��� 
 
 	// PWM ��ȣ  pin: OC2(PB7), Timer2, PWM signal (period= 16.384msec )

	TCCR2 |= 0x68;   //  Trigger signal (OC2)   �߻� :  WGM20(bit6)=1,  WGM21(bit3)=1,  COM21(bit5)=1, COM20(bit4)=0 ,  
	TCCR2 |= 0x05;   //  1024����,  ����Ŭ���ֱ� = 64usec  : CS22(bit2) = 1, CS21(bit1) = 0,  CS20(bit0) = 1 
   
    Servo_Pos_CMD = Pos_center ;
 	Servo_Move( Servo_Pos_CMD );               // �������� ��� ��ġ Pos_center = 90 ���� ���� ���� ȸ��  

	servo_duty=90;	
/////////////������� �ʱ�ȭ
 	init_serial() ;    // Serial Port (USART1) �ʱ�ȭ
 	UCSR0B |=  0x80  ;      // UART1 �۽�(RX) �Ϸ� ���ͷ�Ʈ ���
	
/////////////////////////////DC����
	DDRB |= 0x20;   // ���ͱ�����ȣ + ����:  PWM ��Ʈ( pin: OC1A(PB5) )   --> ��� ���� 
	DDRA |= 0x01;   // ���ͱ�����ȣ - ���� : ���� ��/�����Ʈ(pin : PA0 ) --> ��� ���� 
	PORTA &=~0x01;
// ���ͱ�����ȣ ( pin: OC1A(PB5) ),   Timer1, PWM signal (period= 200 usec )

	TCCR1A = 0x82;    // OC1A(PB5)) :  PWM ��Ʈ ����,   Fast PWM ( mode 14 )
	TCCR1B = 0x1b;    // 64 ���� Ÿ�̸� 1 ���� (����Ŭ�� �ֱ� =  64/(16*10^6) = 4 usec ),  Fast PWM ( mode 14 ) 
	ICR1 = 50;        // PWM �ֱ� = 50 * 4 usec = 200 usec (  PWM ���ļ� = 1/200usec = 5 kHz )

    Vmax = ICR1; 
	OCR1A = duty;      //  OC1A(PB5) PWM duty = 0 ���� : ���� ����
	duty=20;

////  3 ���� �����ļ���( Ultrasonic Sensor) ////////////
// �����Ʈ ���� 
	DDRB |= 0x07;     // 3 �����ļ��� Trigger signals( PB0, PB1, PB2 : �����Ʈ ����  )
	PORTB &= ~0x07;   // PB0, PB1, PB2  : Low  ( 3 Trigger signals OFF )  
   
 ////////////  Timer 0 ����  ( 10 msec �ֱ� Ÿ�̸� 0 ���ͷ�Ʈ )  ///////////////
      
    TCCR0 = 0x00;            // Ÿ�̸� 0 ����(���ֺ� = 1024 ) , Normal mode(Ÿ�̸Ӹ��)
    TCNT0 = 256 - 156;       //  ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                             //  �����÷����ͷ�Ʈ �ֱ� = 10msec
                             //  156 = 10msec/ 64usec
    TIMSK &= ~0x01;            // Ÿ�̸�0 �����÷����ͷ�Ʈ ���
  
// 3 Echo Signal�� Pulse Width measurment,  Timer3 

	TCCR3A = 0x00; 
	TCCR3B = 0x02;     // Ÿ�̸� 3 ����(���ֺ� 8) ,  0.5usec ������ ���� 


 
// 3 �����ļ��� Echo Signals : external interrupt 4( pin: INT4 (PE4)),  external interrupt 5( pin: INT5 (PE5)) 
//                           : external interrupt 6( pin: INT4 (PE6)) 
	EICRB |= 0x15;    // Both falling edge and rising edge interrupt
	EICRB &= ~0x2A;   // Both falling edge and rising edge interrupt

	EIMSK &= ~0x70;    // INT4 Enable, INT5 Enable, INT6 Enable
	sei(); 
///////////////////////////////////////
    TCCR0 |= 0x07;    // Ÿ�̸� 0 ����(���ֺ� = 1024 ) 
		
	
		
	 
	while (1) 

	{ 
	
	if( new_recv_flag == 1 )      // 1 ���� ���ſϷ� �� 
		 { 
               for( i=0; i < recv_cnt ; i++) 
			   {
			      if( recv_data[i] == Cmd_Message_1[i] ) eq_count1++ ;
			      if( recv_data[i] == Cmd_Message_2[i] ) eq_count2++ ; 
			      if( recv_data[i] == Cmd_Message_3[i] ) eq_count3++ ;
			      if( recv_data[i] == Cmd_Message_4[i] ) eq_count4++ ; 
			      if( recv_data[i] == Cmd_Message_5[i] ) eq_count5++ ;
			      if( recv_data[i] == Cmd_Message_6[i] ) eq_count6++ ;
				  if( recv_data[i] == Cmd_Message_7[i] ) eq_count7++ ;
				  if( recv_data[i] == Cmd_Message_8[i] ) eq_count8++ ;
				  if( recv_data[i] == Cmd_Message_9[i] ) eq_count9++ ;
			   	 if( recv_data[i] == Cmd_Message_10[i] ) eq_count10++ ;
            
               }
               if     ( eq_count1 == 4 )  cmd_data = 0 ;    
               else if( eq_count2 == 5 )  cmd_data = 1 ;    
			   else if( eq_count3 == 8 )  cmd_data = 2 ;      
			   else if( eq_count4 == 7 )  cmd_data = 3 ;     
			   else if( eq_count5 == 10 )  cmd_data = 4 ;     
			   else if( eq_count6 == 2 )  cmd_data = 5 ;
			   else if( eq_count7 == 12 )  cmd_data = 6 ;     
			   else if( eq_count8 == 13 )  cmd_data = 7 ;     
			   else if( eq_count9 == 14)  cmd_data = 8 ;   
	 			else if( eq_count10 == 15)  cmd_data = 9 ;
			   else                       cmd_data = 0xFE ;  

               eq_count1 = 0;  eq_count2 = 0;  eq_count3 = 0;  eq_count4 = 0;eq_count5 = 0;  eq_count6 = 0;eq_count7 = 0;eq_count8 = 0;eq_count9 = 0; eq_count10 = 0;
               new_recv_flag = 0 ; 
		 }

		  //////////////  ��ɾ� ó��   //////////////

			if( cmd_data == 0)          // ���� 0 �� ���ŵǸ� 
			{
			    TIMSK &= ~0x01;      // Ÿ�̸�0 �����÷����ͷ�Ʈ ����
				EIMSK &= ~0x70;    // INT4 Enable, INT5 Enable, INT6 Enable

				PORTA |=0x70; //LED off
				servo_control=0;

				SerialPutString("Fan Stop"); 
				SerialPutChar('\n');

				distance_1 = 100;
				distance_2 = 100;
				distance_3 = 100;

					DC_Motor_Stop();             // DC ���� ���� �Լ�
					Servo_Pos_CMD = Pos_center ;
 		     		Servo_Move( Servo_Pos_CMD );               // �������� ��� ��ġ Pos_center = 90 ���� ���� ���� ȸ��  	
			}
			
			else if( cmd_data == 1 )     // ���� 1 �� ���ŵǸ�
			{
			DC_Motor_Stop();             // DC ���� ���� �Լ�
            TIMSK |= 0x01;           // Ÿ�̸�0 �����÷����ͷ�Ʈ ���
			EIMSK |= 0x70;    // INT4 Enable, INT5 Enable, INT6 Enable
			
			servo_control=0;

			SerialPutString("Fan Start"); 
			SerialPutChar('\n'); 
			}


			else if( cmd_data == 2)      
			{
			PORTA &=~0x10;
			PORTA |=0x20;
			PORTA |=0x40;
			duty=20; //���� PWM ��Ƽ(�޽���) ����	//PWM ��
			HexToDec( duty, 10); //10������ ��ȯ
			SerialPutString("duty= "); 	
            SerialPutChar(NumToAsc(cnumber[1]));
			SerialPutChar(NumToAsc(cnumber[0])); 
			SerialPutChar('\n');                     
		                       
			} 
		
			else if( cmd_data == 3)      
			{

			
			PORTA &=~0x10;
			PORTA &=~0x20;
			PORTA |=0x40;

			duty=30;			//PWM �߰�
			HexToDec( duty, 10); //10������ ��ȯ
			
			SerialPutString("duty= "); 	
            SerialPutChar(NumToAsc(cnumber[1]));
			SerialPutChar(NumToAsc(cnumber[0])); 
			SerialPutChar('\n'); 
		        
			} 
		
			else if(cmd_data == 4)      
			{
			
			PORTA &=~0x10;
			PORTA &=~0x20;
			PORTA &=~0x40;

		    duty=50;//PWM ��
			HexToDec( duty, 10); //10������ ��ȯ
			
			SerialPutString("duty= "); 	
            SerialPutChar(NumToAsc(cnumber[1]));
			SerialPutChar(NumToAsc(cnumber[0])); 
			SerialPutChar('\n');  
 
		  	} 
			
			else if(cmd_data == 5)      
			{
				 duty=(AscToNum(recv_data[2])*10+AscToNum(recv_data[3]));
				if(duty >50) duty=AscToNum(recv_data[2]);	
				
				if(duty<=20){
					PORTA &=~0x10;
					PORTA |=0x20;
					PORTA |=0x40;
				}
				else if (duty<=35){
					PORTA &=~0x10;
					PORTA &=~0x20;
					PORTA |=0x40;
				}
				else if (duty<=50){
					PORTA &=~0x10;
					PORTA &=~0x20;
					PORTA &=~0x40;
				}
				
				SerialPutString("duty= "); 	
                SerialPutChar(recv_data[2]);
			    SerialPutChar(recv_data[3]);
 				SerialPutChar('\n'); 
 
		  	} 
			else if(cmd_data == 6)      
			{
			
				servo_control=0;
				SerialPutString("Svo auto"); 	
         
 				SerialPutChar('\n'); 
 
		  	} 
			else if(cmd_data == 7)      
			{
				
				servo_control=1;

				SerialPutString("Svo manual"); 	
 				SerialPutChar('\n'); 
			
 
		  	} 
			else if(cmd_data == 8)      
			{

				servo_duty -=10;	
				if(servo_duty<0)servo_duty=0; 

				Servo_Pos_CMD = servo_duty ;
 		   		Servo_Move( Servo_Pos_CMD );
				
				HexToDec(servo_duty,10); //10������ ��ȯ
				SerialPutString("servo= "); 	
                SerialPutChar(NumToAsc(cnumber[2]));
			    SerialPutChar(NumToAsc(cnumber[1]));
				SerialPutChar(NumToAsc(cnumber[0]));
 				SerialPutChar('\n'); 
 
		  	} 
			else if(cmd_data == 9)      
			{
				
				
				servo_duty +=10;	
				if(servo_duty>180)servo_duty=180; 

				HexToDec(servo_duty,10); //10������ ��ȯ
				Servo_Pos_CMD = servo_duty  ;
 		    	Servo_Move( Servo_Pos_CMD );
				
				SerialPutString("servo= ");
                SerialPutChar(NumToAsc(cnumber[2]));
			    SerialPutChar(NumToAsc(cnumber[1]));
				SerialPutChar(NumToAsc(cnumber[0]));
 				SerialPutChar('\n'); 
 
		  	} 
			 else if( cmd_data == 0xFE )      //  ��� ���� �̸� 
			{

                SerialPutString( "Command Error!\n" ); //  ��� ���� �޽��� ����
			}

		 
        cmd_data = 0xFF;                             //  ����� �ʱⰪ���� ����
  

	
	cli();
 	    main_dist1 = distance_1 ;
		main_dist2 = distance_2 ;
		main_dist3 = distance_3 ;
 	sei(); 
	



	if(main_dist1<=Limited_d || main_dist2<=Limited_d || main_dist3<=Limited_d )//20cm���� ������
		{

		DC_Motor_Run_Fwd(duty);    // DC ���� ��ȸ��(PWM����) �Լ�
	
		}
		else{
			  
		 DC_Motor_Stop();             // DC ���� ���� �Լ�
		//���� 
		}
	
if(servo_control==0){
	
	 if( main_dist1<=Limited_d && main_dist2<=Limited_d && main_dist3<=Limited_d)
		{
			if(step==0)
			{ 	
			Servo_Pos_CMD = Pos_max ;
 		    Servo_Move( Servo_Pos_CMD );
		
				if( Time_Delay_Polling( Servi_time ) == 1 ) {step++;}  
			}  
			else if(step==1)
			{
				Servo_Pos_CMD = Pos_center ;
 		        Servo_Move( Servo_Pos_CMD );               // �������� �ּ� ��ġ Pos_min = 0 ���� ���� ���� ȸ�� 
		
		 		if( Time_Delay_Polling( Servi_time) == 1 ) {step++;} 
			} 
		
			 if(step==2)
			{
			
				Servo_Pos_CMD = Pos_min ;
 		        Servo_Move( Servo_Pos_CMD );               // �������� �ּ� ��ġ Pos_min = 0 ���� ���� ���� ȸ�� 
		
		 		if( Time_Delay_Polling( Servi_time) == 1 ) {step++;}  
			} 
			
			 if(step==3)
			{
			
				Servo_Pos_CMD =Pos_center ;
 		        Servo_Move( Servo_Pos_CMD );               // �������� �ּ� ��ġ Pos_min = 0 ���� ���� ���� ȸ�� 
		
		 		if( Time_Delay_Polling( Servi_time ) == 1 ) {step=0;}  
			} 
			  msec_delay(50);
			           
		}
		else if( main_dist2<=Limited_d && main_dist3<=Limited_d)
		{
			if(step==0)
			{ 	
			
			Servo_Pos_CMD = Pos_min ;
 		    Servo_Move( Servo_Pos_CMD );
		
				if( Time_Delay_Polling( Servi_time ) == 1 ) step++; 
			}  
			        
		
			 if(step!=0)
			{
			
				Servo_Pos_CMD = Pos_center ;
 		        Servo_Move( Servo_Pos_CMD );               // �������� �ּ� ��ġ Pos_min = 0 ���� ���� ���� ȸ�� 
		
		 		if( Time_Delay_Polling( Servi_time) == 1 ) step=0;  
			} 
		  msec_delay(50);	
		}
		
		else if(main_dist1<=Limited_d &&  main_dist2<=Limited_d)
		{
			
			if(step==0)
			{
			
				Servo_Pos_CMD = Pos_max ;
 		        Servo_Move( Servo_Pos_CMD );               // �������� �ּ� ��ġ Pos_min = 0 ���� ���� ���� ȸ�� 
		
		 		if( Time_Delay_Polling(Servi_time) == 1 ) step++;  
			} 
			
		   if(step!=0)
			{
			
				Servo_Pos_CMD =Pos_center ;
 		        Servo_Move( Servo_Pos_CMD );               // �������� �ּ� ��ġ Pos_min = 0 ���� ���� ���� ȸ�� 
		
		 		if( Time_Delay_Polling( Servi_time ) == 1 ) step=0;  
			} 
		  msec_delay(50);   
		}
		else if(main_dist1<=Limited_d &&  main_dist3<=Limited_d)
		{
			
			if(step==0)
			{
			
				Servo_Pos_CMD = Pos_max ;
 		        Servo_Move( Servo_Pos_CMD );               // �������� �ּ� ��ġ Pos_min = 0 ���� ���� ���� ȸ�� 
		
		 		if( Time_Delay_Polling(Servi_time) == 1 ) step++;  
			} 
			
		   if(step!=0)
			{
			
				Servo_Pos_CMD =Pos_min ;
 		        Servo_Move( Servo_Pos_CMD );               // �������� �ּ� ��ġ Pos_min = 0 ���� ���� ���� ȸ�� 
		
		 		if( Time_Delay_Polling( Servi_time ) == 1 ) step=0;  
			} 
		  msec_delay(50);   
		}	
		
		else if (main_dist1<=Limited_d ){
		
       
               Servo_Pos_CMD = Pos_max ;
 		        Servo_Move( Servo_Pos_CMD );             
		 
		}
		else if (main_dist2<=Limited_d ){
		
       
               Servo_Pos_CMD = Pos_center ;
 		        Servo_Move( Servo_Pos_CMD );             
		 
		}
		else if (main_dist3<=Limited_d){
		                        
       
                Servo_Pos_CMD = Pos_min ;
 		        Servo_Move( Servo_Pos_CMD );                


		}

		}//servo_contro	

     }//while��


} //main �� 


ISR(  USART0_RX_vect )
{

    static unsigned char r_cnt = 0 ;

    rdata = UDR0; 

    if( rdata != '.' )                      // ���ŵ� �����Ͱ� ������ ���ڸ� ��Ÿ���� ������(��ħǥ)�� �ƴϸ�
    {
        //SerialPutChar( rdata);               // Echo  ���ŵ� �����͸� �ٷ� �۽��Ͽ� ���ŵ� �����Ͱ� ��Ȯ���� Ȯ�� 
   	    recv_data[r_cnt] = rdata;        //  ���ŵ� ���� ���� 
	    r_cnt++;                         //  ���� ���� ���� ���� 

		new_recv_flag = 0;

    }
    else if(  rdata == '.' )                // ���ŵȵ����Ͱ� ������ ���ڸ� ��Ÿ���� ������(��ħǥ) �̸�
    {
       // SerialPutChar('\n');                // �޴������� ������ ���۽� Line Feed('\n')�� �׻� ���� �����ؾ��� 
        recv_cnt = r_cnt ;                  // ���ŵ� ������ ����Ʈ�� ����
        r_cnt = 0;  
        
		new_recv_flag = 1;

    }


}



ISR( TIMER0_OVF_vect )    //  10 msec �ֱ� Ÿ�̸�1 �����÷� ���ͷ�Ʈ �������α׷�
{

    static unsigned short  time_index = 0 ; 


    TCNT0 = 256 - 156;       //  ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                             //  �����÷����ͷ�Ʈ �ֱ� = 10msec
                             //  156 = 10msec/ 64usec

    time_index++ ; 

    if( time_index == 10 )   // 50 msec �ֱ� 
    {

       time_index = 0; 

       sensor_count++;          // ������ ���� ī���� �� ���� 
	       
	 
	   if( sensor_count == 4 )  sensor_count = 1; 


       if ( sensor_count == 1 )        //  �����ļ��� 1 Ʈ���� ��ȣ �߻�(������ 1 �߻�) 
	   {
	      PORTB |= 0x01;    // PB0 : High
		  usec_delay(20) ;  // 20usec ���� High ���� 
	      PORTB &= 0xFE;    // PB0 : Low 
          
		  active_sensor_flag = 1;

	   }
       else if ( sensor_count == 2 )   //  �����ļ��� 2 Ʈ���� ��ȣ �߻�(������ 2 �߻�)
	   {
	      PORTB |= 0x02;    // PB1 : High
	 	  usec_delay(20) ;  // 20usec ���� High ���� 
	      PORTB &= 0xFD;    // PB1 : Low 

		  active_sensor_flag = 2;

	   }
       else if ( sensor_count == 3 )   //  �����ļ��� 3 Ʈ���� ��ȣ �߻�(������ 3 �߻�)
	   {
	      PORTB |= 0x04;    // PB2 : High
		  usec_delay(20) ;  // 20usec ���� High ���� 
	      PORTB &= 0xFB;    // PB2 : Low 

		  active_sensor_flag = 3;

	   }	
   }
}



ISR(INT4_vect)
{

    static unsigned short count1 = 0, count2 = 0, del_T = 0, flag = 0 ;

    if ( active_sensor_flag == 1 )
 	{

	   if(flag == 0) 
	   {
		  count1 = TCNT3; 
		  flag = 1;
	  } 
	  else 
	  { 
		  count2 = TCNT3; 
		  del_T = count2 - count1;

    	  distance_1 = del_T/(2*58); 

          if( distance_1 > 380 )  // �ݻ�Ǵ� �����İ� ������� ������ 
		  {
		      distance_1 = distance_1_old ;   // ���� ������ ��� 
		  } 

          distance_1_old = distance_1 ;    // ���� ������ ���� ���� ������Ʈ  

		  flag = 0; 

	 	  active_sensor_flag = 0;
	  } 
    }
} 


ISR(INT5_vect)
{

    static unsigned short count1 = 0, count2 = 0, del_T = 0, flag = 0 ;


    if ( active_sensor_flag == 2 )
	{

	   if(flag == 0) 
	   {
		  count1 = TCNT3; 
		  flag = 1;
	  } 
	  else 
	  { 
		  count2 = TCNT3; 
		  del_T = count2 - count1;
    	  distance_2 = del_T/(2*58); 

          if( distance_2 > 380 )  // �ݻ�Ǵ� �����İ� ������� ������ 
		  {
		      distance_2 = distance_2_old ;   // ���� ������ ��� 
		  } 

          distance_2_old = distance_2 ;    // ���� ������ ���� ���� ������Ʈ  

		  flag = 0; 

	 	  active_sensor_flag = 0;
	  } 
    }
} 



ISR(INT6_vect)
{

    static unsigned short count1 = 0, count2 = 0, del_T = 0, flag = 0 ;

    if ( active_sensor_flag == 3 )
	{

	   if(flag == 0) 
	   {
		  count1 = TCNT3; 
		  flag = 1;
	  } 
	  else 
	  { 
		  count2 = TCNT3; 
		  del_T = count2 - count1;
    	  distance_3 = del_T/(2*58); 

          if( distance_3 > 380 )  // �ݻ�Ǵ� �����İ� ������� ������ 
		  {
		      distance_3 = distance_3_old ;   // ���� ������ ��� 
		  } 

          distance_3_old = distance_3 ;    // ���� ������ ���� ���� ������Ʈ  

		  flag = 0; 

	 	  active_sensor_flag = 0;
	  } 
    }
} 



//================================================================================================================

void Servo_Move( short sv_pos_cmd )
{
      OCR2 = ( 135 * sv_pos_cmd )/900  + 10  ;  

      //  �޽��� = 0.64msec = 64usec * 10,   ���� ��(0 ��)  (�޽��� = 0.66msec )
      //  �޽��� = 1.47msec = 64usec * 23 ,  ���(90 ��) (�޽��� = 1.5msec )
      //  �޽��� = 2.37msec = 64usec * 37 ,  ������ ��(180 ��) (�޽��� = 2.45msec ) 
}

/////////////////////////////////////////////////////////////

// UART1 ��� �ʱ�ȭ ���α׷� 

void init_serial(void)
{
    UCSR0A = 0x00;                    //�ʱ�ȭ
    UCSR0B = 0x18  ;                  //�ۼ������,  �ۼ��� ���ͷ�Ʈ ����
    UCSR0C = 0x06;                    //������ ���ۺ�Ʈ �� 8��Ʈ�� ����.
    
    UBRR0H = 0x00;
    UBRR0L = 103;                     //Baud Rate 9600 
}
////////////////////////////////////////////////////////////////


//=================================================================================
void DC_Motor_Run_Fwd( short duty )   // DC ���� ��ȸ�� �Լ� 
{

    if( duty > Vmax )     duty = Vmax ;

    PORTA &= ~0x01;     //  ���ͱ�����ȣ - ���� : 0 V �ΰ�( PA0 = 0 );  
	OCR1A = duty;       //  ���ͱ�����ȣ + ���� : OC1A(PB5) PWM duty ���� 


}
void DC_Motor_Stop( void )   // DC ���� ���� �Լ� 
{

    PORTA &= ~0x01;     //  ���ͱ�����ȣ - ���� : 0 V �ΰ�( PA0 = 0 );  
	OCR1A = 0;          //  ���ͱ�����ȣ + ���� : OC1A(PB5) PWM duty = 0 ���� 
}


//======================================
// �� ���ڸ� �۽��Ѵ�.
//======================================

void SerialPutChar(char ch)
{
	while(!(UCSR0A & (1<<UDRE)));			// ���۰� �� ���� ��ٸ�
  	UDR0 = ch;								// ���ۿ� ���ڸ� ����
}


//=============================================
// ���ڿ��� �۽��Ѵ�.
// �Է�   : str - �۽��� ���ڿ��� ������ ������ �ּ�
//=============================================

 void SerialPutString(char *str)
 {
    while(*str != '\0')         // ���ŵ� ���ڰ� Null ����( 0x00 )�� �ƴϸ� 
    {

        SerialPutChar(*str++);
    }
}

///////////////////////////////////////////////////////////////


void HexToDec( unsigned short num, unsigned short radix) 
{
	int j ;

	for(j=0; j<5 ; j++) cnumber[j] = 0 ;

	j=0;
	do
	{
		cnumber[j++] = num % radix ; 
		num /= radix; 

	} while(num);

} 

char NumToAsc( unsigned char Num )
{
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 

	return Num ;
}

char AscToNum(char Num)
{
      Num -= 0x30;
   	return Num;
}

void msec_delay(int n)
{	
	for(; n>0; n--)		// 1msec �ð� ������ nȸ �ݺ�
		_delay_ms(1);		// 1msec �ð� ����
}



void usec_delay(int n)
{	
	for(; n>0; n--)		// 1usec �ð� ������ nȸ �ݺ�
		_delay_us(1);		// 1usec �ð� ����
}

//////////////////////////////////////////////////////////

unsigned char Time_Delay_Polling( unsigned short d_time )
{

    static unsigned short  curr_delay = 0; 
	unsigned char  ret_val = 0;


    curr_delay++ ;  

    if( curr_delay >= d_time )   // 50msec * d_time ��� �� 
	{
       ret_val = 1; 
       curr_delay = 0 ;
	} 


    return  ret_val ;


}




