#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FCY 69784687UL
#include <libpic30.h>
#include "hard_i2c.h"
#include "adxl355.h"
#include "mcc_generated_files/mcc.h"


// レジスター regset に、regdata を書き込む
// 1:タイムアウト 0:正常
int ADXL355_write(uint8_t regset, uint8_t regdata) {
	if (I2C_start()) return 1;
	if (I2C_send(ADXL355_dev_addr)) return 1;
	if (I2C_ackchk() == 2) return 1;
	if (I2C_send(regset)) return 1;
	if (I2C_ackchk() == 2) return 1;
    if (I2C_send(regdata)) return 1;
	if (I2C_ackchk() == 2) return 1;
	return I2C_stop();
}        


// 読み出し開始アドレスをセット
// 1:タイムアウト 0:正常
int ADXL355_setadr(uint8_t addr) {
	if (I2C_start()) return 1;
	if (I2C_send(ADXL355_dev_addr)) return 1;
	if (I2C_ackchk() == 2) return 1;
	if (I2C_send(addr)) return 1;
	if (I2C_ackchk() == 2) return 1;
    return 0;
}


// 1:タイムアウト 0:正常
int ADXL355_init(uint8_t sample) {
    if (ADXL355_write(0x28, sample)) return 1; // ローパス&サンプリング
    return ADXL355_write(0x2D, 0);
}


// 温度のナマ値を摂氏に変換
signed char ADXL355_calt(uint16_t t) {
    uint16_t a = t * 20; 
    if (a <= 41565) {
        a = 41655 - a;
        a /= 181;
        return (signed char)a;
    }
    a -= 41475;
    a /= 181;
    signed char r = (signed char)a;
    return (-r);
}


// 1:タイムアウト 0:正常
int ADXL355_read(ADXL355 *v) {
    if (ADXL355_setadr(8)) return 1;
    
	if (I2C_restart()) return 1;
	if (I2C_send(ADXL355_dev_addr | 1)) return 1;
	if (I2C_ackchk() == 2) return 1;

    // 温度取得
//    v->tH = I2C_rcv();
//    I2C_acksnd();
//    v->tL = I2C_rcv();
//    I2C_acksnd();
    
    // X加速度取得
    uint16_t r;
    r = I2C_rcv();
    if (r == 256) return 1;
    v->xH = (uint8_t)r;
    if (I2C_acksnd()) return 1;
    r = I2C_rcv();
    if (r == 256) return 1;
    v->xM = (uint8_t)r;
    if (I2C_acksnd()) return 1;
    r = I2C_rcv();
    if (r == 256) return 1;
    v->xL = (uint8_t)r;
    if (I2C_acksnd()) return 1;
    v->x >>= 12;
    
    // Y加速度取得
    r = I2C_rcv();
    if (r == 256) return 1;
    v->yH = (uint8_t)r;
    if (I2C_acksnd()) return 1;
    r = I2C_rcv();
    if (r == 256) return 1;
    v->yM = (uint8_t)r;
    if (I2C_acksnd()) return 1;
    r = I2C_rcv();
    if (r == 256) return 1;
    v->yL = (uint8_t)r;
    if (I2C_acksnd()) return 1;
    v->y >>= 12;

    // Z加速度取得
    r = I2C_rcv();
    if (r == 256) return 1;
    v->zH = (uint8_t)r;
    if (I2C_acksnd()) return 1;
    r = I2C_rcv();
    if (r == 256) return 1;
    v->zM = (uint8_t)r;
    if (I2C_acksnd()) return 1;
    r = I2C_rcv();
    if (r == 256) return 1;
    v->zL = (uint8_t)r;
    if (I2C_nacksnd()) return 1;
    if (I2C_stop()) return 1;
    v->z >>= 12;
    return 0;
}


// 9,999,999:タイムアウト
signed long ADXL355_readAcc(uint8_t addr) {
    union {
        signed long x;  // X加速
        struct {
            uint8_t xS;
            uint8_t xL;
            uint8_t xM;
            uint8_t xH;
        };
    } v;
    
    uint8_t retry = 0;
    for (retry=0; retry<5; retry++) {
        if (ADXL355_setadr(addr)) return 9999999;

    	if (I2C_restart()) return 9999999;
        if (I2C_send(ADXL355_dev_addr | 1)) return 9999999;
        if (I2C_ackchk() == 2)  return 9999999;

        // 加速度取得
        uint16_t r;
        r = I2C_rcv();
        if (r == 256) return 9999999;
        v.xH = (uint8_t)r;
        if (I2C_acksnd()) return 9999999;
        r = I2C_rcv();
        if (r == 256) return 9999999;
        v.xM = (uint8_t)r;
        if (I2C_acksnd()) return 9999999;
        r = I2C_rcv();
        if (r == 256) return 9999999;
        v.xL = (uint8_t)r;
        if (I2C_nacksnd()) return 9999999;
        if (I2C_stop()) return 9999999;

        if (I2C1STATbits.BCL == 0) { // バス衝突無し
            v.x >>= 12;
            return v.x;
        }
        i2c1_driver_clearBusCollision();
        __delay_ms(5);
    }
    return 9999999;
}

