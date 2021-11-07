// 車体傾斜と走行系制御 dsPIC 版
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "hard_i2c.h"
#include "lcd_i2c.h"
#include "adxl355.h"


// PWM 送信パッケージ
typedef union tagPWM4 {
    uint16_t pwm[4];
    uint8_t buf[8];
} PWM4;


uint16_t table_pwm[] = {
	20,
	21,
	22,
	22,
	23,
	24,
	25,
	26,
	27,
	28,
	29,
	30,
	32,
	33,
	34,
	35,
	37,
	38,
	40,
	41,
	43,
	45,
	46,
	48,
	50,
	52,
	54,
	56,
	58,
	61,
	63,
	65,
	68,
	71,
	73,
	76,
	79,
	82,
	85,
	89,
	92,
	96,
	100,
	103,
	107,
	112,
	116,
	120,
	125,
	130,
	135,
	140,
	146,
	151,
	157,
	163,
	170,
	176,
	183,
	190,
	198,
	206,
	214,
	222,
	231,
	240,
	249,
	259,
	269,
	279,
	290,
	301,
	313,
	325,
	338,
	351,
	365,
	379,
	394,
	409,
	425,
	441,
	459,
	476,
	495,
	514,
	534,
	555,
	577,
	599,
	623,
	647,
	672,
	698,
	725,
	754,
	783,
	813,
	845,
	878,
	912,
	948,
	985,
	1023,
	1063,
	1104,
	1147,
	1192,
	1238,
	1286,
	1337,
	1389,
	1443,
	1499,
	1557,
	1618,
	1681,
	1746,
	1814,
	1885,
	1958,
	2035,
	2114,
	2196,
	2282,
	2370,
	2463,
	2559,
	2559
};            


#define CENT1 13000 /* data2.pwm[3] +で突っ張る */
#define CENT2 12000 /* data2.pwm[2] -で突っ張る */
#define CENT3 12000 /* data1.pwm[0] +で突っ張る */
#define CENT4 11500 /* data1.pwm[1] -で突っ張る */
#define CENT5 12000 /* data1.pwm[3] +で突っ張る */
#define CENT6 13000 /* data1.pwm[2] -で突っ張る */
#define CENT7 11000 /* data2.pwm[1] +で突っ張る */
#define CENT8 12000 /* data2.pwm[0] -で突っ張る */
// 車体重量が増えるとこの補正値を大きくする
// 上記調整値より256倍大きいスケールで値を定義する
#define CENT_WEIGHT (0L*256)
#define CENT1L (256L * CENT1)
#define CENT2L (256L * CENT2)
#define CENT3L (256L * CENT3)
#define CENT4L (256L * CENT4)
#define CENT5L (256L * CENT5)
#define CENT6L (256L * CENT6)
#define CENT7L (256L * CENT7)
#define CENT8L (256L * CENT8)


#define SPI_BYTES2 8 /* SPI送信するデーターのバイト数 */
PWM4 data1, data2; // SPI送信するデーター
uint8_t motor; // SPI送信するデーター
uint8_t step_val[2]; // ステッピングモーター値
signed short hosei = 0; // 補正量
signed long cannon = 0; // 仰角

#define SPI_BYTES 8 /* SPI受信するデーターのバイト数 */
uint8_t data[SPI_BYTES]; // SPI受信格納先

ADXL355 a;

char buf[32];


// SPI受信
void int_strb(void) {
    if (SPI2_STRB_GetValue() == 0) return;

    uint16_t t;
    uint8_t idx, b, d, *dp = data;
    for (idx=0; idx<SPI_BYTES; idx++) {
        d = 0;
        for (b=0; b<8; b++) {
            t = 0;
            while (SPI2_CLOCK_GetValue() == 0) {
                if ((t++) > 1000) return;
            } // CLOCK立ち上がりをソフトループで待つ
            d <<= 1;
            t = 0;
            while (SPI2_CLOCK_GetValue() == 1) {
                if ((t++) > 1000) return;
            } // CLOCK立ち下がりをソフトループで待つ
            d |= SPI2_DATA_GetValue();
        }
        (*(dp++)) = d;
    }
}


// SPI送信
void spi_send(void) {
    uint16_t portb; // = step_val;
//    portb <<= 8;
    uint8_t idx, b, m, d1, *dp1 = data1.buf;
    uint8_t d2, *dp2 = data2.buf;
    // パケット先頭 STRB=1 で相手に伝える
    // クロックを1にするまで15μ秒以上空けるのを仕様とする
    SPI_STRB_SetHigh();
    __delay_us(14);

    SPI_STRB_SetLow();
    for (idx=0; idx<SPI_BYTES2; idx++) {
        d1 = (*(dp1++));
        d2 = (*(dp2++));
        m = 0x80;
        for (b=0; b<8; b++) {
            SPI_CLOCK_SetHigh();
            if (d1 & m) {
                portb |= 1; // SPI_DATA1_SetHigh();
            }
            else {
                portb &= 0xfffe; // SPI_DATA1_SetLow();
            }
            if (d2 & m) {
                portb |= 2; // SPI_DATA2_SetHigh();
            }
            else {
                portb &= 0xfffd; // SPI_DATA2_SetLow();
            }
            if (motor & m) {
                portb |= 0x10; // SPI_DATA3_SetHigh();
            }
            else {
                portb &= 0xffef; // SPI_DATA3_SetLow();
            }
            if (step_val[idx & 1] & m) {
                portb |= 0x80; // SPI_DATA4_SetHigh();
            }
            else {
                portb &= 0xff7f; // SPI_DATA4_SetLow();
            }
            LATB = portb;
            __delay_us(2);
            m >>= 1;
            SPI_CLOCK_SetLow();
            __delay_us(5);
        }        
    }
    SPI_DATA1_SetLow();
    SPI_DATA2_SetLow();
    SPI_DATA3_SetLow();
    SPI_DATA4_SetLow();
}


// サーボを待機位置にする（転輪を最大に上げる）
void waiting_position(void) {
    data1.pwm[0] = 4000;
    data1.pwm[1] = 20000;
    data1.pwm[2] = 20000;
    data1.pwm[3] = 4000;
    data2.pwm[0] = 20000;
    data2.pwm[1] = 4000;
    data2.pwm[2] = 20000;
    data2.pwm[3] = 4000;
    spi_send();
}


// サーボに値をセット
// c: 水平を0とし、仰角は＋ 俯角は－
// h1: 補正量
void set_servo(signed long c, signed short h1) {
    signed long c3 = (c / 3);
    
    // 最前転輪
    signed long s1 = c;
    signed long s8 = c;
    // 2番目転輪
    signed long s2 = c3;
    signed long s7 = c3;
    // 3番目転輪
    signed long s3 = (-c3);
    signed long s6 = s3;
    // 最後転輪
    signed long s4 = (-c);
    signed long s5 = s4;

    s1 = (CENT1L + CENT_WEIGHT) + s1;
    s2 = (CENT2L - CENT_WEIGHT) - s2;
    s3 = (CENT3L + CENT_WEIGHT) + s3;
    s4 = (CENT4L - CENT_WEIGHT) - s4;
    s5 = (CENT5L + CENT_WEIGHT) + s5;
    s6 = (CENT6L - CENT_WEIGHT) - s6;
    s7 = (CENT7L + CENT_WEIGHT) + s7;
    s8 = (CENT8L - CENT_WEIGHT) - s8;
    
    if (s1 < 0) { s1 = 0; }
    if (s2 < 0) { s2 = 0; }
    if (s3 < 0) { s3 = 0; }
    if (s4 < 0) { s4 = 0; }
    if (s5 < 0) { s5 = 0; }
    if (s6 < 0) { s6 = 0; }
    if (s7 < 0) { s7 = 0; }
    if (s8 < 0) { s8 = 0; }

    uint16_t u1 = (uint16_t)(s1 >> 8);
    uint16_t u2 = (uint16_t)(s2 >> 8);
    uint16_t u3 = (uint16_t)(s3 >> 8);
    uint16_t u4 = (uint16_t)(s4 >> 8);
    uint16_t u5 = (uint16_t)(s5 >> 8);
    uint16_t u6 = (uint16_t)(s6 >> 8);
    uint16_t u7 = (uint16_t)(s7 >> 8);
    uint16_t u8 = (uint16_t)(s8 >> 8);

    uint16_t mh, w;
    if (h1 >= 0) { // 車体が右下がり
        // 最大補正可能量を計算
        mh = (u1 - 4000);
        w = (20000 - u2);
        if (w < mh) mh = w;
        w = (u3 - 4000);
        if (w < mh) mh = w;
        w = (20000 - u4);
        if (w < mh) mh = w;
        w = (20000 - u5);
        if (w < mh) mh = w;
        w = (u6 - 4000);
        if (w < mh) mh = w;
        w = (20000 - u7);
        if (w < mh) mh = w;
        w = (u8 - 4000);
        if (w < mh) mh = w;
        if (h1 > mh) h1 = mh;
        
        // 補正実行
        u1 -= h1;
        u2 += h1;
        u3 -= h1;
        u4 += h1;
        u5 += h1;
        u6 -= h1;
        u7 += h1;
        u8 -= h1;
    }
    else { // 車体が右上がり
        h1 = (-h1);
        // 最大補正可能量を計算
        mh = (20000 - u1);
        w = (u2 - 4000);
        if (w < mh) mh = w;
        w = (20000 - u3);
        if (w < mh) mh = w;
        w = (u4 - 4000);
        if (w < mh) mh = w;
        w = (u5 - 4000);
        if (w < mh) mh = w;
        w = (20000 - u6);
        if (w < mh) mh = w;
        w = (u7 - 4000);
        if (w < mh) mh = w;
        w = (20000 - u8);
        if (w < mh) mh = w;
        if (h1 > mh) h1 = mh;

        // 補正実行
        u1 += h1;
        u2 -= h1;
        u3 += h1;
        u4 -= h1;
        u5 -= h1;
        u6 += h1;
        u7 -= h1;
        u8 += h1;
    }

    data1.pwm[0] = u3;
    data1.pwm[1] = u4;
    data1.pwm[2] = u6;
    data1.pwm[3] = u5;
    data2.pwm[0] = u8;
    data2.pwm[1] = u7;
    data2.pwm[2] = u2;
    data2.pwm[3] = u1;
    spi_send();
}


// サーボを中立位置にする
void neutral_position(void) {
    set_servo(0, 0);
}


int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    CN_SetInterruptHandler(int_strb);
    i2c1_driver_driver_open();
    i2c1_driver_initSlaveHardware();

    neutral_position();
    
    WATCHDOG_TimerClear();
    __delay_ms(100);    
    WATCHDOG_TimerClear();
    LCD_i2c_init(8);
    
//    while (ADXL355_init(6)) {
//        i2c1_driver_close();            
//        i2c1_driver_driver_open();
//        i2c1_driver_initSlaveHardware();
//    }
    
    uint8_t can;
    data[0] = data[1] = data[2] = data[3] = 0;
    data[4] = data[5] = data[6] = data[7] = 0x80; // 停止
    motor = step_val[0] = step_val[1] = 128;
    
    while (1)
    {
        WATCHDOG_TimerClear();

        motor = step_val[0] = data[4];
        step_val[1] = data[6];

        can = data[7];
        if (can > 128) { // ↑に
            cannon += table_pwm[can - 128];
            if (cannon > 1536000) {
                cannon = 1536000;
            }
        }
        if (can < 128) { // ↓に
            cannon -= table_pwm[128 - can];
            if (cannon < (-1536000)) {
                cannon = (-1536000);
            }
        }

        if ((data[0] & 12) && (data[1] & 12)) { // 左右の下トリムを同時押し
            waiting_position(); // サスアーム収納ポジション
        }
        else if (data[1] & 3) {
            neutral_position();
        }
        else {
//            signed long x = ADXL355_readAcc(ADXL355_ADR_X);
//            if (x == 9999999) {
//                i2c1_driver_close();            
//                i2c1_driver_driver_open();
//                i2c1_driver_initSlaveHardware();
//            }
//            else {
//                x >>= 10; // 事実上は符号だけ必要 >>12 標準だがそれは少し精度落ち
//                hosei += (signed short)x;
//                set_servo(cannon, hosei);

                LCD_i2C_cmd(0x80);
                LCD_i2C_data("ABC");
//          }
        }
        __delay_ms(1);
    }
    return 1; 
}


/**
 End of File
*/

