/**
 * @file DHT11_driver.c
 * @brief DHT11 Driver for Microchip PIC18F4550 MCU.
 *
 * Date: 24-06-2012
 * License: BSD 3-Clause
 * 
 * Copyright (c) 2012-2014, Pedro A. Hortas (pah@ucodev.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  - Neither the name of the ucodev.org nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 * Project details:
 *
 *  Home Page:   http://www.ucodev.org
 *  Version:     0.1
 *  Portability: Microchip PIC* MCUs
 *  Description: DHT11 Driver
 *  Notes:       Delay functions must be implemented based on the instruction set documentation and must match the
 *               frequency of the chipset being used.
 *               PORT definitions must be changed on DHT11_driver.h file.
 *
 */

#include "DHT11_driver.h"


void DHT11_init(void) {
    /* Data pin shall be initialized to Vcc in order to set DHT11 to
     * power save mode after boot.
     */
    dht11_DATA_tris = 0;
    dht11_DATA_out = 1;

    // Wait 1sec for DHT11 boot up.
    delay_10ms(100);
}

static BOOL DHT11_start_signal(void) {
    int response = -1;

    /* Send start signal and wait 18ms - stage 1
     * This will force DHT11 to leave power save mode.
     */
    dht11_DATA_out = 0;
    delay_1ms(18);

    /* Send start signal and wait 40ms - stage 2 */
    dht11_DATA_out = 1;
    delay_10us(4);

    /* Try to read DHT response and wait 80ms - stage 1 */
    dht11_DATA_tris = 1;
    response = dht11_DATA_in;

    /* DHT11 shall answer with voltage low (GND) for 80us */
    if (response != 0)
        return 0;   // Failure

    while (dht11_DATA_in == 0)
        continue;
    //delay_10us(8);

    /* Try to read DHT response and wiat 80ms - stage 2 */
    response = dht11_DATA_in;

    /* DHT11 shall answer, again, with voltage high (Vcc) for another 80us */
    if (response != 1)
        return 0;   // Failure

    /* Wait until voltage drops to 0 (GND) - This shall take about 80us as
     * stated before.
     */
    while (dht11_DATA_in != 0)
        continue;

    /* All good */
    return 1;
}

static BOOL DHT11_read_bit(void) {
    /* Sanity check:
     *
     * Since the signal may still be 1 (Vcc) from the previous bit read,
     * We should wait for DHT to drop voltage, signaling the incoming
     * of another data bit.
     *
     */
    while (dht11_DATA_in != 0)
        continue;

    /* 50us until voltage rise */
    while (dht11_DATA_in == 0)
        continue;

    /* If the signal is 1 (Vcc) for 26-28us and then 
     * drops to 0 (GND) the data bit is 0.
     *
     * If the signal is 1 (Vcc) for up to 70us and then
     * drops to 0 (GND) the data bit is 1.
     *
     * We'll wait about 40us and then read the signal.
     */

    /* Wait 40us and read voltage */
    delay_10us(4);

    /*
     * If after 40us the data pin is 1 (Vcc) then bit is 1.
     * 
     * If after 40us the data pin is 0 (GND) then bit is 0.
     */

    return (dht11_DATA_in == 1); /* Return data bit */
}

static void DHT11_read_data(BYTE *ans) {
    int i, j;

    for (i = 0; i < 5; i++) { /* Read 5 octects */
        /* Reset octect */
        ans[i] = 0;

        for (j = 7; j >= 0; j--) { /* Read 8 bits - MSB comes first */
            ans[i] |= (DHT11_read_bit() << j);
        }
    }

    /* After communication is complete, wait 50us */
    delay_10us(5);

    /* Set DHT11 to power save mode */
    dht11_DATA_tris = 0;
    dht11_DATA_out = 1;

}

static BYTE DHT11_verify_checksum(BYTE *ans) {
    /* Checksum is the 8 LSBs from the sum of the
     * first 4 octects of ans
     */

    /* Compare the last 8bits of the sum to ans[4] (checksum) */
    return (((ans[0] + ans[1] + ans[2] + ans[3]) & 0xFF) == ans[4]);
}

BOOL DHT11_retrieve(BYTE *ans) {
    /* Send start signal to init communication */
    if (DHT11_start_signal() != 1) {
        ans[0] = ans[1] = ans[2] = ans[3] = ans[4] = 0;
        return 0;
    }

    /* Read data from DHT11 */
    DHT11_read_data(ans);

    /* Verify checksum and return 1 (success) or 0 (failed) */
    return DHT11_verify_checksum(ans);
}
