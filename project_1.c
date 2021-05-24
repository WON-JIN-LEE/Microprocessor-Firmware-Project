#include <avr/io.h> 
#include <avr/interrupt.h> 
#include <util/delay.h> 
#include "lcd.h"


void init_serial(void) ;  //  Serial 토신포트 초기화
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


void msec_delay(int n)  ;   // msec 단위 시간지연
void usec_delay(int n)  ;   // usec 단위 시간지연
unsigned char Time_Delay_Polling( unsigned short d_time ) ;   // 시간지연 체크함수(폴링방식)
unsigned short step=0;
static volatile short     Servi_time = 150 ;      


static volatile unsigned short    distance_1 = 50, distance_2 = 50,  distance_3 = 50, sensor_count = 0, active_sensor_flag = 0  ;
static volatile unsigned short    distance_1_old = 0, distance_2_old = 0,  distance_3_old = 0 ;
unsigned short main_dist1 = 100 ,main_dist2 = 100 ,main_dist3 = 100  ;
static volatile short     Limited_d = 30 ;      // 제한거리

void DC_Motor_Run_Fwd( short duty );    // DC 모터 정회전(PWM구동) 함수
void DC_Motor_Stop( void );             // DC 모터 정지 함수  
static volatile short  Vmax = 0 ; 

void Servo_Move( short sv_pos_cmd ) ;
static volatile short     Servo_Pos_CMD = 0 ;    // 서보 위치 명령 ( 범위 : 0 - 180,  단위:  도 )
static volatile short     Pos_max = 125;        // 서보 최대 위치 명령 ( 180 도 )
static volatile short     Pos_min = 45 ;          // 서보 최소 위치 명령 ( 0   도 )
static volatile short    Pos_center=90;      //  기준 

int main() 
{   
	 unsigned short duty = 0, servo_control=0;
 	 char    eq_count1=0, eq_count2=0, eq_count3=0, eq_count4=0, eq_count5=0, eq_count6 = 0,eq_count7 = 0,eq_count8 = 0,eq_count9 = 0,eq_count10 = 0,cmd_data = 0xFF  ; 
	 unsigned char   i=0 ; //for문
	short  servo_duty=0; 
	 
////LED 포트
DDRA |=0x70;
PORTA |=0x70; //LED off

/////////////////////////////서보모터

	DDRB |= 0x80;    //  PWM 포트: OC2( PB7 ) 출력설정 
 
 	// PWM 신호  pin: OC2(PB7), Timer2, PWM signal (period= 16.384msec )

	TCCR2 |= 0x68;   //  Trigger signal (OC2)   발생 :  WGM20(bit6)=1,  WGM21(bit3)=1,  COM21(bit5)=1, COM20(bit4)=0 ,  
	TCCR2 |= 0x05;   //  1024분주,  내부클럭주기 = 64usec  : CS22(bit2) = 1, CS21(bit1) = 0,  CS20(bit0) = 1 
   
    Servo_Pos_CMD = Pos_center ;
 	Servo_Move( Servo_Pos_CMD );               // 서보모터 가운데 위치 Pos_center = 90 도로 서보 모터 회전  

	servo_duty=90;	
/////////////블루투스 초기화
 	init_serial() ;    // Serial Port (USART1) 초기화
 	UCSR0B |=  0x80  ;      // UART1 송신(RX) 완료 인터럽트 허용
	
/////////////////////////////DC모터
	DDRB |= 0x20;   // 모터구동신호 + 단자:  PWM 포트( pin: OC1A(PB5) )   --> 출력 설정 
	DDRA |= 0x01;   // 모터구동신호 - 단자 : 범용 입/출력포트(pin : PA0 ) --> 출력 설정 
	PORTA &=~0x01;
// 모터구동신호 ( pin: OC1A(PB5) ),   Timer1, PWM signal (period= 200 usec )

	TCCR1A = 0x82;    // OC1A(PB5)) :  PWM 포트 설정,   Fast PWM ( mode 14 )
	TCCR1B = 0x1b;    // 64 분주 타이머 1 시작 (내부클럭 주기 =  64/(16*10^6) = 4 usec ),  Fast PWM ( mode 14 ) 
	ICR1 = 50;        // PWM 주기 = 50 * 4 usec = 200 usec (  PWM 주파수 = 1/200usec = 5 kHz )

    Vmax = ICR1; 
	OCR1A = duty;      //  OC1A(PB5) PWM duty = 0 설정 : 모터 정지
	duty=20;

////  3 개의 초음파센서( Ultrasonic Sensor) ////////////
// 출력포트 설정 
	DDRB |= 0x07;     // 3 초음파센서 Trigger signals( PB0, PB1, PB2 : 출력포트 설정  )
	PORTB &= ~0x07;   // PB0, PB1, PB2  : Low  ( 3 Trigger signals OFF )  
   
 ////////////  Timer 0 설정  ( 10 msec 주기 타이머 0 인터럽트 )  ///////////////
      
    TCCR0 = 0x00;            // 타이머 0 정지(분주비 = 1024 ) , Normal mode(타이머모드)
    TCNT0 = 256 - 156;       //  내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                             //  오버플로인터럽트 주기 = 10msec
                             //  156 = 10msec/ 64usec
    TIMSK &= ~0x01;            // 타이머0 오버플로인터럽트 허용
  
// 3 Echo Signalㄴ Pulse Width measurment,  Timer3 

	TCCR3A = 0x00; 
	TCCR3B = 0x02;     // 타이머 3 시작(분주비 8) ,  0.5usec 단위로 측정 


 
// 3 초음파센서 Echo Signals : external interrupt 4( pin: INT4 (PE4)),  external interrupt 5( pin: INT5 (PE5)) 
//                           : external interrupt 6( pin: INT4 (PE6)) 
	EICRB |= 0x15;    // Both falling edge and rising edge interrupt
	EICRB &= ~0x2A;   // Both falling edge and rising edge interrupt

	EIMSK &= ~0x70;    // INT4 Enable, INT5 Enable, INT6 Enable
	sei(); 
///////////////////////////////////////
    TCCR0 |= 0x07;    // 타이머 0 시작(분주비 = 1024 ) 
		
	
		
	 
	while (1) 

	{ 
	
	if( new_recv_flag == 1 )      // 1 문자 수신완료 시 
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

		  //////////////  명령어 처리   //////////////

			if( cmd_data == 0)          // 문자 0 이 수신되면 
			{
			    TIMSK &= ~0x01;      // 타이머0 오버플로인터럽트 금지
				EIMSK &= ~0x70;    // INT4 Enable, INT5 Enable, INT6 Enable

				PORTA |=0x70; //LED off
				servo_control=0;

				SerialPutString("Fan Stop"); 
				SerialPutChar('\n');

				distance_1 = 100;
				distance_2 = 100;
				distance_3 = 100;

					DC_Motor_Stop();             // DC 모터 정지 함수
					Servo_Pos_CMD = Pos_center ;
 		     		Servo_Move( Servo_Pos_CMD );               // 서보모터 가운데 위치 Pos_center = 90 도로 서보 모터 회전  	
			}
			
			else if( cmd_data == 1 )     // 문자 1 이 수신되면
			{
			DC_Motor_Stop();             // DC 모터 정지 함수
            TIMSK |= 0x01;           // 타이머0 오버플로인터럽트 허용
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
			duty=20; //실제 PWM 듀티(펄스폭) 변경	//PWM 약
			HexToDec( duty, 10); //10진수로 변환
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

			duty=30;			//PWM 중간
			HexToDec( duty, 10); //10진수로 변환
			
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

		    duty=50;//PWM 강
			HexToDec( duty, 10); //10진수로 변환
			
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
				
				HexToDec(servo_duty,10); //10진수로 변환
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

				HexToDec(servo_duty,10); //10진수로 변환
				Servo_Pos_CMD = servo_duty  ;
 		    	Servo_Move( Servo_Pos_CMD );
				
				SerialPutString("servo= ");
                SerialPutChar(NumToAsc(cnumber[2]));
			    SerialPutChar(NumToAsc(cnumber[1]));
				SerialPutChar(NumToAsc(cnumber[0]));
 				SerialPutChar('\n'); 
 
		  	} 
			 else if( cmd_data == 0xFE )      //  명령 오류 이면 
			{

                SerialPutString( "Command Error!\n" ); //  명령 오류 메시지 전송
			}

		 
        cmd_data = 0xFF;                             //  명령을 초기값으로 리셋
  

	
	cli();
 	    main_dist1 = distance_1 ;
		main_dist2 = distance_2 ;
		main_dist3 = distance_3 ;
 	sei(); 
	



	if(main_dist1<=Limited_d || main_dist2<=Limited_d || main_dist3<=Limited_d )//20cm보다 작으면
		{

		DC_Motor_Run_Fwd(duty);    // DC 모터 정회전(PWM구동) 함수
	
		}
		else{
			  
		 DC_Motor_Stop();             // DC 모터 정지 함수
		//멈춤 
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
 		        Servo_Move( Servo_Pos_CMD );               // 서보모터 최소 위치 Pos_min = 0 도로 서보 모터 회전 
		
		 		if( Time_Delay_Polling( Servi_time) == 1 ) {step++;} 
			} 
		
			 if(step==2)
			{
			
				Servo_Pos_CMD = Pos_min ;
 		        Servo_Move( Servo_Pos_CMD );               // 서보모터 최소 위치 Pos_min = 0 도로 서보 모터 회전 
		
		 		if( Time_Delay_Polling( Servi_time) == 1 ) {step++;}  
			} 
			
			 if(step==3)
			{
			
				Servo_Pos_CMD =Pos_center ;
 		        Servo_Move( Servo_Pos_CMD );               // 서보모터 최소 위치 Pos_min = 0 도로 서보 모터 회전 
		
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
 		        Servo_Move( Servo_Pos_CMD );               // 서보모터 최소 위치 Pos_min = 0 도로 서보 모터 회전 
		
		 		if( Time_Delay_Polling( Servi_time) == 1 ) step=0;  
			} 
		  msec_delay(50);	
		}
		
		else if(main_dist1<=Limited_d &&  main_dist2<=Limited_d)
		{
			
			if(step==0)
			{
			
				Servo_Pos_CMD = Pos_max ;
 		        Servo_Move( Servo_Pos_CMD );               // 서보모터 최소 위치 Pos_min = 0 도로 서보 모터 회전 
		
		 		if( Time_Delay_Polling(Servi_time) == 1 ) step++;  
			} 
			
		   if(step!=0)
			{
			
				Servo_Pos_CMD =Pos_center ;
 		        Servo_Move( Servo_Pos_CMD );               // 서보모터 최소 위치 Pos_min = 0 도로 서보 모터 회전 
		
		 		if( Time_Delay_Polling( Servi_time ) == 1 ) step=0;  
			} 
		  msec_delay(50);   
		}
		else if(main_dist1<=Limited_d &&  main_dist3<=Limited_d)
		{
			
			if(step==0)
			{
			
				Servo_Pos_CMD = Pos_max ;
 		        Servo_Move( Servo_Pos_CMD );               // 서보모터 최소 위치 Pos_min = 0 도로 서보 모터 회전 
		
		 		if( Time_Delay_Polling(Servi_time) == 1 ) step++;  
			} 
			
		   if(step!=0)
			{
			
				Servo_Pos_CMD =Pos_min ;
 		        Servo_Move( Servo_Pos_CMD );               // 서보모터 최소 위치 Pos_min = 0 도로 서보 모터 회전 
		
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

     }//while문


} //main 문 


ISR(  USART0_RX_vect )
{

    static unsigned char r_cnt = 0 ;

    rdata = UDR0; 

    if( rdata != '.' )                      // 수신된 데이터가 마지막 문자를 나타내는 데이터(마침표)가 아니면
    {
        //SerialPutChar( rdata);               // Echo  수신된 데이터를 바로 송신하여 수신된 데이터가 정확한지 확인 
   	    recv_data[r_cnt] = rdata;        //  수신된 문자 저장 
	    r_cnt++;                         //  수신 문자 갯수 증가 

		new_recv_flag = 0;

    }
    else if(  rdata == '.' )                // 수신된데이터가 마지막 문자를 나타내는 데이터(마침표) 이면
    {
       // SerialPutChar('\n');                // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함 
        recv_cnt = r_cnt ;                  // 수신된 데이터 바이트수 저장
        r_cnt = 0;  
        
		new_recv_flag = 1;

    }


}



ISR( TIMER0_OVF_vect )    //  10 msec 주기 타이머1 오버플로 인터럽트 서비스프로그램
{

    static unsigned short  time_index = 0 ; 


    TCNT0 = 256 - 156;       //  내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                             //  오버플로인터럽트 주기 = 10msec
                             //  156 = 10msec/ 64usec

    time_index++ ; 

    if( time_index == 10 )   // 50 msec 주기 
    {

       time_index = 0; 

       sensor_count++;          // 초음파 센서 카운터 값 증가 
	       
	 
	   if( sensor_count == 4 )  sensor_count = 1; 


       if ( sensor_count == 1 )        //  초음파센서 1 트리거 신호 발생(초음파 1 발사) 
	   {
	      PORTB |= 0x01;    // PB0 : High
		  usec_delay(20) ;  // 20usec 동안 High 유지 
	      PORTB &= 0xFE;    // PB0 : Low 
          
		  active_sensor_flag = 1;

	   }
       else if ( sensor_count == 2 )   //  초음파센서 2 트리거 신호 발생(초음파 2 발사)
	   {
	      PORTB |= 0x02;    // PB1 : High
	 	  usec_delay(20) ;  // 20usec 동안 High 유지 
	      PORTB &= 0xFD;    // PB1 : Low 

		  active_sensor_flag = 2;

	   }
       else if ( sensor_count == 3 )   //  초음파센서 3 트리거 신호 발생(초음파 3 발사)
	   {
	      PORTB |= 0x04;    // PB2 : High
		  usec_delay(20) ;  // 20usec 동안 High 유지 
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

          if( distance_1 > 380 )  // 반사되는 초음파가 검출되지 않을때 
		  {
		      distance_1 = distance_1_old ;   // 직전 측정값 사용 
		  } 

          distance_1_old = distance_1 ;    // 직전 측정값 저장 변수 업데이트  

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

          if( distance_2 > 380 )  // 반사되는 초음파가 검출되지 않을때 
		  {
		      distance_2 = distance_2_old ;   // 직전 측정값 사용 
		  } 

          distance_2_old = distance_2 ;    // 직전 측정값 저장 변수 업데이트  

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

          if( distance_3 > 380 )  // 반사되는 초음파가 검출되지 않을때 
		  {
		      distance_3 = distance_3_old ;   // 직전 측정값 사용 
		  } 

          distance_3_old = distance_3 ;    // 직전 측정값 저장 변수 업데이트  

		  flag = 0; 

	 	  active_sensor_flag = 0;
	  } 
    }
} 



//================================================================================================================

void Servo_Move( short sv_pos_cmd )
{
      OCR2 = ( 135 * sv_pos_cmd )/900  + 10  ;  

      //  펄스폭 = 0.64msec = 64usec * 10,   왼쪽 끝(0 도)  (펄스폭 = 0.66msec )
      //  펄스폭 = 1.47msec = 64usec * 23 ,  가운데(90 도) (펄스폭 = 1.5msec )
      //  펄스폭 = 2.37msec = 64usec * 37 ,  오른쪽 끝(180 도) (펄스폭 = 2.45msec ) 
}

/////////////////////////////////////////////////////////////

// UART1 통신 초기화 프로그램 

void init_serial(void)
{
    UCSR0A = 0x00;                    //초기화
    UCSR0B = 0x18  ;                  //송수신허용,  송수신 인터럽트 금지
    UCSR0C = 0x06;                    //데이터 전송비트 수 8비트로 설정.
    
    UBRR0H = 0x00;
    UBRR0L = 103;                     //Baud Rate 9600 
}
////////////////////////////////////////////////////////////////


//=================================================================================
void DC_Motor_Run_Fwd( short duty )   // DC 모터 정회전 함수 
{

    if( duty > Vmax )     duty = Vmax ;

    PORTA &= ~0x01;     //  모터구동신호 - 단자 : 0 V 인가( PA0 = 0 );  
	OCR1A = duty;       //  모터구동신호 + 단자 : OC1A(PB5) PWM duty 설정 


}
void DC_Motor_Stop( void )   // DC 모터 정지 함수 
{

    PORTA &= ~0x01;     //  모터구동신호 - 단자 : 0 V 인가( PA0 = 0 );  
	OCR1A = 0;          //  모터구동신호 + 단자 : OC1A(PB5) PWM duty = 0 설정 
}


//======================================
// 한 문자를 송신한다.
//======================================

void SerialPutChar(char ch)
{
	while(!(UCSR0A & (1<<UDRE)));			// 버퍼가 빌 때를 기다림
  	UDR0 = ch;								// 버퍼에 문자를 쓴다
}


//=============================================
// 문자열을 송신한다.
// 입력   : str - 송신한 문자열을 저장할 버퍼의 주소
//=============================================

 void SerialPutString(char *str)
 {
    while(*str != '\0')         // 수신된 문자가 Null 문자( 0x00 )가 아니면 
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
	for(; n>0; n--)		// 1msec 시간 지연을 n회 반복
		_delay_ms(1);		// 1msec 시간 지연
}



void usec_delay(int n)
{	
	for(; n>0; n--)		// 1usec 시간 지연을 n회 반복
		_delay_us(1);		// 1usec 시간 지연
}

//////////////////////////////////////////////////////////

unsigned char Time_Delay_Polling( unsigned short d_time )
{

    static unsigned short  curr_delay = 0; 
	unsigned char  ret_val = 0;


    curr_delay++ ;  

    if( curr_delay >= d_time )   // 50msec * d_time 경과 후 
	{
       ret_val = 1; 
       curr_delay = 0 ;
	} 


    return  ret_val ;


}




