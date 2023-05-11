#include"DigitalIn.h"
#include"PwmOut.h"
#include"mbed.h"
Serial pc(USBTX, USBRX);// tx, rx
PwmOut led(LED1);
//PwmOut led1(PB_8);
//PwmOut led2(PB_9);
DigitalIn bt(A0);
//AnalogOut dac(PA_4);
PwmOut servo(D13);
float brightness = 0.0;
int main() 
{
    servo.period_ms(20);
    while(1) 
    {
            char c = pc.getc();
            servo.pulsewidth_us(500);
                if(c =='a') //기본표정
                {
                    led = 0;
                    //led1 = 1;
                    //led2 = 0;
                   // dac.write(3.3);//아날로그 직접 출력
                    printf("1");
                    servo.pulsewidth_us(500);
                    wait(2);
                    servo.pulsewidth_us(500);
                }
                if(c=='b') // 기쁜표정
                {
                    led = 1;
                    //led1 =0;
                    //led2 =1;
                    printf("2");
                    servo.pulsewidth_us(2500);
                    wait(2);
                    servo.pulsewidth_us(2500);
                }
                if(c=='c') // 슬픈표정
                {
                    led = 1;
                    //led1 =0;
                    //led2 =1;
                    printf("3");
                    servo.pulsewidth_us(2500);
                    wait(2);
                    servo.pulsewidth_us(2500);
                }
                if(c=='d') // 화남표정
                {
                    led = 1;
                    //led1 =0;
                    //led2 =1;
                    printf("4");
                    servo.pulsewidth_us(2500);
                    wait(2);
                    servo.pulsewidth_us(2500);
                }
    }
}
