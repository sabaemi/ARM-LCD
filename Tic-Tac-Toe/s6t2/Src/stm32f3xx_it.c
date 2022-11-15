/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
#include "LiquidCrystal.h"
//int i=0;
char a;
int b=0;
int position = 0;
extern int r1;
extern int r2;
extern char s[20];
extern char co[3][3];
extern int end;
extern int turn;
extern char tn;
int checkvertic(char a[3][3]){
if(a[0][0]==a[1][0] && a[2][0]==a[1][0] ){
	if(a[0][0]=='x') return(1);
	else if(a[0][0]=='o') return(2);
}
else if(a[0][1]==a[1][1] && a[2][1]==a[1][1] ){
	if(a[0][1]=='x') return(1);
	else if(a[0][1]=='o') return(2);
}
else if(a[0][2]==a[1][2] && a[2][2]==a[1][2] ){
	if(a[0][2]=='x') return(1);
	else if(a[0][2]=='o') return(2);
}
else return(0);
}

int checkhoriz(char a[3][3]){
if(a[0][0]==a[0][1] && a[0][2]==a[0][1] ){
	if(a[0][0]=='x') return(1);
	else if(a[0][0]=='o') return(2);
}
else if(a[1][1]==a[1][0] && a[1][2]==a[1][1] ){
	if(a[1][1]=='x') return(1);
	else if(a[1][1]=='o') return(2);
}
else if(a[2][1]==a[2][0] && a[2][2]==a[2][1] ){
	if(a[2][2]=='x') return(1);
	else if(a[2][2]=='o') return(2);
}
else return(0);
}

int checkcross(char a[3][3]){
if(a[0][0]==a[1][1] && a[2][2]==a[1][1] ){
	if(a[0][0]=='x') return(1);
	else if(a[0][0]=='o') return(2);
}
else if(a[1][1]==a[2][0] && a[0][2]==a[1][1] ){
	if(a[1][1]=='x') return(1);
	else if(a[1][1]=='x') return(2);
}
else return(0);
}

int checktie(char a[3][3]){
	int flag=0;
for(int i=0;i<3;i++){
	for(int j=0;j<3;j++){
	if(a[i][j]=='\0') flag=1;}}
	
	if(flag==0 && checkcross(a)==0 && checkhoriz(a)==0 && checkvertic(a)==0)
		return(1);
	else if(flag==1) return(0);
}

typedef unsigned char byte;
byte first[8] = {
  0x00,
  0x00,
  0x00,
  0x01,
  0x0F,
  0x08,
  0x1A,
  0x00
};
byte second[8] = {
  0x00,
  0x00,
  0x02,
  0x19,
  0x09,
  0x1F,
  0x00,
  0x00
};

byte third[8] = {
  0x00,
  0x00,
  0x00,
  0x07,
  0x05,
  0x07,
  0x00,
  0x00
};
byte fourth[8] = {
  0x00,
  0x04,
  0x0E,
  0x00,
  0x15,
  0x1F,
  0x00,
  0x00
};
byte fifth[8] = {
  0x00,
  0x00,
  0x00,
  0x06,
  0x02,
  0x07,
  0x00,
  0x00
};
byte fmos[8] = {
	0x00,
  0x00,
  0x00,
  0x03,
  0x1F,
  0x00,
  0x00,
  0x00
};

byte smos[8] = {
  0x00,
  0x00,
  0x00,
  0x15,
  0x1F,
  0x00,
  0x00,
  0x00
};
byte tmos[8] = {
  0x00,
  0x01,
  0x19,
  0x19,
  0x09,
  0x18,
  0x00,
  0x00
};
byte fomos[8] = {
  0x00,
  0x00,
  0x07,
  0x04,
  0x12,
  0x12,
  0x0C,
  0x00
};
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	extern unsigned char data[1];
  extern unsigned char buffer[100];
	r1=0;
	r2=0;
	s[0]='\0';
			if(data[0] != '\r'){
					buffer[position] = data[0];
					buffer[position+1] = '\0';
				  position++;
			}
			else if(data[0] == '\r'){
			  if(end==1){
				clear();
				 for(int i=0;i<3;i++){
	       for(int j=0;j<3;j++){ co[i][j]='\0';}}
				 end=0;
				 tn='\0';
				 turn=0;
				 setCursor(3,0);
				 print("|");
				 setCursor(3,1);
				 print("|");
				 setCursor(3,2);
				 print("|");
				 
				 setCursor(0,3);
				 print("-");
				 setCursor(1,3);
				 print("-");
				 setCursor(2,3);
				 print("-");
				}
					
				b=buffer[0]-'0';
				r2=b-1;
				b=buffer[2]-'0';
				r1=b-1;
				if(buffer[4]=='x'||buffer[4]=='o') s[0]=buffer[4];
				
				if(turn==0) tn=s[0];
				if(r1>-1 && r1<3 && r2>-1 && r2<3){
				if(co[r2][r1]=='\0') {
				if(s[0]=='x'||s[0]=='o'){
				if(turn%2==0 && s[0]==tn){
				co[r2][r1]=s[0];
				setCursor(r1,r2);
				print(s);
					turn++;
			  }
				else if(turn%2==1 && s[0]!=tn){
				co[r2][r1]=s[0];
				setCursor(r1,r2);
				print(s);
					turn++;
			  }
				}
				}
			}
				
				if(checkcross(co)==1 || checkhoriz(co)==1 || checkvertic(co)==1){
					createChar(0,first);
					createChar(1,second);
					createChar(2,third);
					createChar(3,fourth);
					createChar(4,fifth);
					home();
					setCursor(10,1);
					write(4);
					write(3);
					write(2);
					write(1);
					write(0);
					print(" x ");
					end=1;
				}
				else if(checkcross(co)==2 || checkhoriz(co)==2 || checkvertic(co)==2){
					createChar(0,first);
					createChar(1,second);
					createChar(2,third);
					createChar(3,fourth);
					createChar(4,fifth);
					home();
					setCursor(10,1);
					write(4);
					write(3);
					write(2);
					write(1);
					write(0);
					print(" o ");
					end=1;
				}
				else if(checktie(co)==1){
					createChar(5,fmos);
					createChar(6,smos);
					createChar(7,tmos);
					createChar(8,fomos);
					home();
					setCursor(10,1);
					write(8);
					write(7);
					write(6);
					write(5);
					end=1;
				}
				
				position=0;		
			}	
			HAL_UART_Receive_IT(&huart2,data,sizeof(data));
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
