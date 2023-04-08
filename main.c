// 車体傾斜と走行系制御 dsPIC 版
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "hard_i2c.h"
#include "lcd_i2c.h"
#include "wt901.h"


typedef union tagHL16 {
    signed short SHL;
    uint16_t HL;
    struct {
        uint8_t L;
        uint8_t H;
    };
    struct {
        unsigned :8;
        unsigned :7;
        unsigned T:1;
    };
} HL16;


typedef union tagHL32 {
    uint32_t HL;
    struct {
        signed short L;
        signed short H;
    };
} HL32;


// PWM 送信パッケージ
typedef union tagPWM4 {
    uint16_t pwm[4];
    uint8_t buf[8];
} PWM4;


// Allocate and reserve a page of flash for this test to use.  The compiler/linker will reserve this for data and not place any code here.
static __prog__  uint8_t flashTestPage[FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS] __attribute__((space(prog),aligned(FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS)));

// We have detected a flash hardware error of some type.
static void FlashError()
{
    while (1) 
    { }
}

static void MiscompareError()
{
    while (1) 
    { }
}


uint16_t table_pwm[] = {
	10,
	11,
	12,
	13,
	15,
	16,
	18,
	19,
	21,
	24,
	26,
	29,
	31,
	35,
	38,
	42,
	46,
	51,
	56,
	61,
	67,
	74,
	81,
	90,
	98,
	108,
	119,
	131,
	144,
	159,
	174,
	192,
	211,
	267,
	323,
	379,
	435,
	491,
	546,
	602,
	658,
	714,
	770,
	826,
	882,
	938,
	994,
	1050,
	1106,
	1161,
	1217,
	1273,
	1329,
	1385,
	1441,
	1497,
	1553,
	1609,
	1665,
	1720,
	1776,
	1832,
	1888,
	1944,
	2000,
	2312,
	2625,
	2938,
	3250,
	3562,
	3875,
	4188,
	4500,
	4812,
	5125,
	5438,
	5750,
	6062,
	6375,
	6688,
	7000,
	7312,
	7625,
	7938,
	8250,
	8562,
	8875,
	9188,
	9500,
	9812,
	10125,
	10438,
	10750,
	11062,
	11375,
	11688,
	12000,
	12312,
	12625,
	12938,
	13250,
	13562,
	13875,
	14188,
	14500,
	14812,
	15125,
	15438,
	15750,
	16062,
	16375,
	16688,
	17000,
	17312,
	17625,
	17938,
	18250,
	18562,
	18875,
	19188,
	19500,
	19812,
	20125,
	20438,
	20750,
	21062,
	21375,
	21688,
	21688
};            


#define CENT1 13000 /* data2.pwm[3] +で突っ張る */
#define CENT2 13000 /* data2.pwm[2] -で突っ張る */
#define CENT3 13000 /* data1.pwm[0] +で突っ張る */
#define CENT4 12000 /* data1.pwm[1] -で突っ張る */
#define CENT5 11500 /* data1.pwm[2] +で突っ張る */
#define CENT6 10000 /* data1.pwm[3] -で突っ張る */
#define CENT7 11000 /* data2.pwm[1] +で突っ張る */
#define CENT8 13000 /* data2.pwm[0] -で突っ張る */
// 車体重量が増えるとこの補正値を大きくする
// 上記調整値より256倍大きいスケールで値を定義する
#define CENT_WEIGHT (2000L*256)
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
signed short height = 0; // 車高補正量
signed short hosei = 0; // 左右傾斜補正量
signed long hosei2 = 0; // 俯仰補正量
signed long cannon = 0; // 仰角

#define SPI_BYTES 8 /* SPI受信するデーターのバイト数 */
uint8_t data[SPI_BYTES]; // SPI受信格納先

#define TIMEOUT 300 /* UART受信タイムアウト */

char buf[32];

// キャリブレーション値
signed short roll0 = (-382);
signed short pitch0 = (-552);

// Angle
signed short roll = 0;
signed short pitch = 0;
// Angular Velocity
signed short wx = 0;
signed short wy = 0;



/////////////////////////////////////////
// キャリブレーション保存
// 失敗したら0以外を返す
/////////////////////////////////////////
char save_calibration() {
    uint32_t flash_storage_address;
    bool result;
    uint32_t write_data[2];
    uint32_t read_data[2];

    HL32 d;
    
    // Get flash page aligned address of flash reserved above for this test.
    flash_storage_address = FLASH_GetErasePageAddress((uint32_t)&flashTestPage[0]);

    FLASH_Unlock(FLASH_UNLOCK_KEY);

    result = FLASH_ErasePage(flash_storage_address);
    if (result == false) {
        FlashError();
        return 1;
    }
    
    // Fill first 4 flash words with data
    // For this product we must write two adjacent words at a one time.
    d.H = roll0;
    d.L = pitch0;
    write_data[0] = d.HL;
    write_data[1] = d.HL;

    // For this product we must write two adjacent words at a one time.
    result  = FLASH_WriteDoubleWord24(flash_storage_address,   write_data[0], write_data[1]);
    if (result == false) {
        FlashError();
        return 1;
    }

    // Clear Key for NVM Commands so accidental call to flash routines will not corrupt flash
    FLASH_Lock();
    
    // read the flash words to verify the data
    read_data[0] = FLASH_ReadWord24(flash_storage_address);
    read_data[1] = FLASH_ReadWord24(flash_storage_address + 2);

    // Stop if the read data does not match the write data;
    if ( (write_data[0] != read_data[0]) ||
         (write_data[1] != read_data[1]) )
    {
        MiscompareError();    
        return 1;
    }
    
    return 0;
}


/////////////////////////////////////////
// キャリブレーション取得
// キャリブレーションが必要なら0以外を返す
/////////////////////////////////////////
char get_calibration() {
    uint32_t flash_storage_address;
    HL32 d;
    
    // Get flash page aligned address of flash reserved above for this test.
    flash_storage_address = FLASH_GetErasePageAddress((uint32_t)&flashTestPage[0]);
    
    d.HL = FLASH_ReadWord24(flash_storage_address);
    roll0 = d.H;
    pitch0 = d.L;
    return 0;
}


// 受信データー確認
// 受信あれば1 なければ0 を返す
char check_rsv(void) {
    uint8_t n = get_rsv_size1();
    if (n < 4) {
        return 0; // 受信データーが少な過ぎる
    }
    n = get_rsv_size2();
    if (n < 4) {
        return 0; // 受信データーが少な過ぎる
    }
    return 1;
}


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
    data1.pwm[2] = 4000;
    data1.pwm[3] = 20000;
    data2.pwm[0] = 20000;
    data2.pwm[1] = 4000;
    data2.pwm[2] = 20000;
    data2.pwm[3] = 4000;
    spi_send();
}


// サーボに値をセット
// c: 水平を0とし、仰角は＋ 俯角は－
// h1: 左右傾斜補正量
// h2: 車高補正量
// h3: 俯仰補正量
// return h3 の修正後
signed long set_servo(signed long c, signed short h1, signed short h2, signed long h3) {
    signed long ret1 = h3;
    c += h3;
    if (c > 1536000) {
        c = 1536000;
        //ret1 = 1536000 - c;
    }
    if (c < (-1536000)) {
        c = (-1536000);
        //ret1 = (-1536000) - c;
    }
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

    signed short u1 = (signed short)(s1 >> 8);
    signed short u2 = (signed short)(s2 >> 8);
    signed short u3 = (signed short)(s3 >> 8);
    signed short u4 = (signed short)(s4 >> 8);
    signed short u5 = (signed short)(s5 >> 8);
    signed short u6 = (signed short)(s6 >> 8);
    signed short u7 = (signed short)(s7 >> 8);
    signed short u8 = (signed short)(s8 >> 8);

    signed short mh, w;
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

    // 車高調整可能範囲を計算
    signed short min_h2 = (4000 - u1);
    signed short max_h2 = (22000 - u1);
    signed short min1 = (4000 - u3);
    signed short max1 = (22000 - u3);
    if (min1 > min_h2) { min_h2 = min1; }
    if (max1 < max_h2) { max_h2 = max1; }
    min1 = (4000 - u5);
    max1 = (22000 - u5);
    if (min1 > min_h2) { min_h2 = min1; }
    if (max1 < max_h2) { max_h2 = max1; }
    min1 = (4000 - u7);
    max1 = (22000 - u7);
    if (min1 > min_h2) { min_h2 = min1; }
    if (max1 < max_h2) { max_h2 = max1; }
    min1 = (u2 - 20000);
    max1 = (u2 - 2000);
    if (min1 > min_h2) { min_h2 = min1; }
    if (max1 < max_h2) { max_h2 = max1; }
    min1 = (u4 - 20000);
    max1 = (u4 - 2000);
    if (min1 > min_h2) { min_h2 = min1; }
    if (max1 < max_h2) { max_h2 = max1; }
    min1 = (u6 - 20000);
    max1 = (u6 - 2000);
    if (min1 > min_h2) { min_h2 = min1; }
    if (max1 < max_h2) { max_h2 = max1; }
    min1 = (u8 - 20000);
    max1 = (u8 - 2000);
    if (min1 > min_h2) { min_h2 = min1; }
    if (max1 < max_h2) { max_h2 = max1; }
    if (h2 < min_h2) { h2 = min_h2; }
    if (h2 > max_h2) { h2 = max_h2; }

    // 車高調整
    u1 += h2; // 4000 to 22000
    u2 -= h2; // 2000 to 20000
    u3 += h2; // 4000 to 22000
    u4 -= h2; // 2000 to 20000
    u5 += h2; // 4000 to 22000
    u6 -= h2; // 2000 to 20000
    u7 += h2; // 4000 to 22000
    u8 -= h2; // 2000 to 20000

    data1.pwm[0] = (uint16_t)u3;  // min 4000
    data1.pwm[1] = (uint16_t)u4; // max 20000
    data1.pwm[2] = (uint16_t)u5;  // min 4000
    data1.pwm[3] = (uint16_t)u6; // max 20000
    data2.pwm[0] = (uint16_t)u8; // max 20000
    data2.pwm[1] = (uint16_t)u7;  // min 4000
    data2.pwm[2] = (uint16_t)u2; // max 20000
    data2.pwm[3] = (uint16_t)u1;  // min 4000
    spi_send();

    return ret1;
}


// 俯仰を中立位置にする
void neutral_position(signed long x512) {
    hosei2 = set_servo(cannon, hosei, height, 0);
}


// 車高を中立位置にする
void neutral_height(void) {
    height = 0;
    hosei2 = set_servo(cannon, hosei, height, hosei2);
}


int main(void)
{
    uint8_t btn1s = 0; // ボタンが連続で押された回数
    uint8_t btn0s = 0; // ボタンが連続で押されていない回数
    
    // initialize the device
    SYSTEM_Initialize();
    set_rsv_buf((uint8_t *)&wx, 4, (uint8_t *)&roll, 4);
    UART1_SetRxInterruptHandler(WT901_rsv_int);

    CN_SetInterruptHandler(int_strb);
    i2c1_driver_driver_open();
    i2c1_driver_initSlaveHardware();

    neutral_position(0);
    
    WATCHDOG_TimerClear();
    __delay_ms(100);    
    WATCHDOG_TimerClear();
    LCD_i2c_init(8);
    //get_calibration();    

    uint8_t nocon = 0; // ノーコン連続回数
    uint8_t can;
    data[0] = data[1] = data[2] = data[3] = 0;
    data[4] = data[5] = data[6] = data[7] = 0x80; // 停止
    motor = step_val[0] = step_val[1] = 128;

    while (1)
    {
        WATCHDOG_TimerClear();

        uint16_t t;
        for (t=0; t<TIMEOUT; t++) {
            if (check_rsv()) {
                break;
            }
            __delay_ms(1);
        }
        clear_rsv_size();

        signed long x2 = (signed long)(roll - roll0 + wx);
        signed long x512 = (x2 * 512);
        signed long hosei2d = (cannon - x512) / 8;
        if (hosei2d > 100000) {
            hosei2d = 100000;
        }
        if (hosei2d < (-100000)) {
            hosei2d = (-100000);
        }
        
        if ((data[0] & 1) == 0) { // 通信エラーもしくはノーコン
            motor = step_val[0] = 128;
            step_val[1] = 128;
        }
        else {
            motor = step_val[0] = data[4];
            step_val[1] = data[6];
        }
        
        can = data[7];
        if (can > 128) { // ↑に
            cannon += table_pwm[can - 128];
            if (cannon > (x512 + 100000)) {
                cannon = (x512 + 100000);
            }
        }
        if (can < 128) { // ↓に
            cannon -= table_pwm[128 - can];
            if (cannon < (x512 - 100000)) {
                cannon = (x512 - 100000);
            }
        }

        if ((data[0] & 1) == 0) { // 通信エラーもしくはノーコン
            if (nocon < 255) nocon ++;
        }
        else {
            nocon = 0;
        }        
        
        if (nocon > 100) { // 通信エラーもしくはノーコンが続いている
            waiting_position(); // サスアーム収納ポジション
        }
        else if (data[1] & 3) { // 左UDボタンを押す
            neutral_position(x512); // 俯仰水平に
        }
        else if (data[0] & 12) { // 右LRボタンを押す
            neutral_height(); // 車高中立に
        }
        else {
            //HL16 x16;
            //x16.L = data[2];
            //x16.H = data[3];
            //signed short x = x16.SHL;
            signed short x = (pitch - pitch0 + wy);
            x >>= 1;
            signed short y = data[5];
            y -= 128;
            height += y;
            if (y > 112) {
                height = 0;
            }
            if (height < -8000) {
                height = -8000;
            }
            if (height > 2000) {
                height = 2000;
            }
            hosei += x;

            hosei2 += hosei2d;
            hosei2 = set_servo(cannon, hosei, height, hosei2);
        }
        if (hosei > 10000) {
            hosei = 10000;
        }
        if (hosei < (-10000)) {
            hosei = (-10000);
        }
        if (hosei2 > 1536000) {
            hosei2 = 1536000;
        }
        if (hosei2 < (-1536000)) {
            hosei2 = (-1536000);
        }

        __delay_ms(10);

        if (BUTTON1_GetValue() == 0) { // ボタンが押されている
            btn0s = 0;
            if (btn1s < 255) {
                btn1s ++;
            }
        }
        else {
            btn1s = 0;
            if (btn0s < 255) {
                btn0s ++;
            }
        }

        //if (btn1s >= 10) {
        if (0) {
            // キャリブレーション実行
            signed long rolls = 0;
            signed long pitchs = 0;
            for (t=0; t<100; t++) {
                LCD_i2C_cmd(0x80);
                sprintf(buf, "START CAL.%3d", t);
                LCD_i2C_data(buf);

                rolls += (signed long)roll;
                pitchs += (signed long)pitch;
                __delay_ms(15);
            }
            roll0 = (signed short)(rolls / 100);
            pitch0 = (signed short)(pitchs / 100);
            //save_calibration();

            LCD_i2C_cmd(0x80);
            sprintf(buf, "END CAL.      ");
            LCD_i2C_data(buf);
            LCD_i2C_cmd(0xC0);
            sprintf(buf, "%8d%8d", roll0, pitch0);
            LCD_i2C_data(buf);
            while (1) ;
        }
    }
    return 1; 
}


/**
 End of File
*/

