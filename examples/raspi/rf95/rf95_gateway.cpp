// rf95_gateway.cpp
//
// LoRa gateway for LoRa sensor networks using RH_RF95 on Raspberry Pi
// Uses the bcm2835 library to access the GPIO pins to drive the RFM95 module
// Requires bcm2835 library to be already installed
// http://www.airspayce.com/mikem/bcm2835/
// Use the Makefile in this directory:
// cd example/raspi/rf95
// make
// sudo ./rf95_gateway
//
// based on rf95_client and rf95_server on Raspberry Pi by Charles-Henri Hallard based on sample RH_NRF24 by Mike Poublon
// written by Dominic Solpico

#include <bcm2835.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/time.h> //Getting the current time of the Pi
#include <time.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <inttypes.h>

// define hardware used change to fit your need
// Uncomment the board you have, if not listed
// uncommment custom board and set wiring tin custom section

// LoRasPi board
// see https://github.com/hallard/LoRasPI
//#define BOARD_LORASPI

// iC880A and LinkLab Lora Gateway Shield (if RF module plugged into)
// see https://github.com/ch2i/iC880A-Raspberry-PI
//#define BOARD_IC880A_PLATE

// Raspberri PI Lora Gateway for multiple modules
// see https://github.com/hallard/RPI-Lora-Gateway
//#define BOARD_PI_LORA_GATEWAY

// Dragino Raspberry PI hat
// see https://github.com/dragino/Lora
#define BOARD_DRAGINO_PIHAT

// Now we include RasPi_Boards.h so this will expose defined
// constants with CS/IRQ/RESET/on board LED pins definition
#include "../RasPiBoards.h"

// Our RFM95 Configuration
#define RF_FREQUENCY  433.00
#define RF_NODE_ID    1

// Create an instance of a driver
RH_RF95 rf95d(RF_CS_PIN, RF_IRQ_PIN);
//This class manages message delivery and reception
RHReliableDatagram rf95m(rf95d, RF_NODE_ID);
//RH_RF95 rf95(RF_CS_PIN);

//Flag for Ctrl-C
volatile sig_atomic_t force_exit = false;

void sig_handler(int sig)
{
  printf("\n%s Break received, exiting!\n", __BASEFILE__);
  force_exit=true;
}

struct dataStruct{      //stores the sensor values in a struct for easier sending and receiving via LoRa
  uint8_t  _temp0[4], _temp1[4], _diox0[4], _diox1[4], _cdty0[4], _cdty1[4];
}rcvData;

void openRF95Stream() {

    printf( "RF95 CS=GPIO%d", RF_CS_PIN);

#ifdef RF_LED_PIN
    pinMode(RF_LED_PIN, OUTPUT);
    digitalWrite(RF_LED_PIN, HIGH );
#endif

#ifdef RF_IRQ_PIN
    printf( ", IRQ=GPIO%d", RF_IRQ_PIN );
    // IRQ Pin input/pull down
    pinMode(RF_IRQ_PIN, INPUT);
    bcm2835_gpio_set_pud(RF_IRQ_PIN, BCM2835_GPIO_PUD_DOWN);
#endif

#ifdef RF_RST_PIN
    printf( ", RST=GPIO%d", RF_RST_PIN );
    // Pulse a reset on module
    pinMode(RF_RST_PIN, OUTPUT);
    digitalWrite(RF_RST_PIN, LOW );
    bcm2835_delay(150);
    digitalWrite(RF_RST_PIN, HIGH );
    bcm2835_delay(100);
#endif

#ifdef RF_LED_PIN
    printf( ", LED=GPIO%d", RF_LED_PIN );
    digitalWrite(RF_LED_PIN, LOW );
#endif

}

void initializeRF95Module() {

#ifdef RF_IRQ_PIN
    // Since we may check IRQ line with bcm_2835 Rising edge detection
    // In case radio already have a packet, IRQ is high and will never
    // go to low so never fire again
    // Except if we clear IRQ flags and discard one if any by checking
    rf95m.available();

    // Now we can enable Rising edge detection
    bcm2835_gpio_ren(RF_IRQ_PIN);
#endif

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
    // you can set transmitter powers from 5 to 23 dBm:
    rf95d.setTxPower(23, false);

    // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
    // transmitter RFO pins and not the PA_BOOST pins
    // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
    // Failure to do that will result in extremely low transmit powers.
    //rf95.setTxPower(14, true);

    // You can optionally require this module to wait until Channel Activity
    // Detection shows no activity on the channel before transmitting by setting
    // the CAD timeout to non-zero:
    //rf95.setCADTimeout(10000);

    // Adjust Frequency
    rf95d.setFrequency(RF_FREQUENCY);

    printf("RF95 node #%d init OK @ %3.2fMHz\n", RF_NODE_ID, RF_FREQUENCY );
}

/*
 * Send a trigger to sensor node over Radiohead.
 */
void sendRF95Trigger() {

    uint8_t tx_trg[] = "S";
    uint8_t tlen = sizeof(tx_trg);

    printf("Sending broadcast ping...\n");
    rf95m.sendto(tx_trg, tlen, RH_BROADCAST_ADDRESS);
    rf95m.waitPacketSent();
}

void recvRF95Data(unsigned long **led_blink) {
    float temp0, temp1, diox0, diox1, cdty0, cdty1;
#ifdef RF_IRQ_PIN
  // We have a IRQ pin ,pool it instead reading
  // Modules IRQ registers from SPI in each loop

  // Rising edge fired ?
    if (bcm2835_gpio_eds(RF_IRQ_PIN)) {
        // Now clear the eds flag by setting it to 1
        bcm2835_gpio_set_eds(RF_IRQ_PIN);
        //printf("Packet Received, Rising event detect for pin GPIO%d\n", RF_IRQ_PIN);
#endif

    if (rf95m.available()) {
        //printf("Message available\n");
#ifdef RF_LED_PIN
        **led_blink = millis();
        digitalWrite(RF_LED_PIN, HIGH);
#endif
        // Should be a message for us now
        uint8_t rx_buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t rlen  = sizeof(rx_buf);
        uint8_t from = rf95m.headerFrom();
        uint8_t to   = rf95m.headerTo();
        uint8_t id   = rf95m.headerId();
        uint8_t flags= rf95m.headerFlags();;
        int8_t rssi  = rf95d.lastRssi();

        if (rf95m.recvfromAck(rx_buf, &rlen, &from)) {
            //printf("Data copied to rx_buf\n");
            memcpy(&rcvData, rx_buf, sizeof(rcvData)); // copying data
            temp0 =  *((float*)(rcvData._temp0));
            temp1 =  *((float*)(rcvData._temp1));
            diox0 =  *((float*)(rcvData._diox0));
            diox1 =  *((float*)(rcvData._diox1));
            cdty0 =  *((float*)(rcvData._cdty0));
            cdty1 =  *((float*)(rcvData._cdty1));

            printf("Packet[%02d] #%d => #%d %ddB:\n", rlen, from, to, rssi);
            printf("temp0: %4.2f\n", temp0);
            printf("diox0: %4.2f\n", diox0);
            printf("cdty0: %4.2f\n", cdty0);
            printf("temp1: %4.2f\n", temp1);
            printf("diox1: %4.2f\n", diox1);
            printf("cdty1: %4.2f\n", cdty1);
        }
        else {
            printf("receive failed\n");
        }
    }
#ifdef RF_IRQ_PIN
  }
#endif

#ifdef RF_LED_PIN
    // Led blink timer expiration ?
    if (**led_blink && millis()-**led_blink>200) {
        **led_blink = 0;
        digitalWrite(RF_LED_PIN, LOW);
    }
#endif
}

void loopRF95(unsigned long *last_millis, unsigned long *led_blink) {
    // Send trigger signal every 5 seconds
    if ( millis() - *last_millis > 5000 ) {
        *last_millis = millis();
        sendRF95Trigger();
    }
    recvRF95Data(&led_blink);
}

void closeRF95Stream() {

#ifdef RF_LED_PIN
    digitalWrite(RF_LED_PIN, LOW );
#endif
    printf( "\n%s Ending\n", __BASEFILE__ );
    bcm2835_close();
}

int executeServerStream() {
    static unsigned long last_millis;
    static unsigned long led_blink = 0;

    signal(SIGINT, sig_handler);
    printf( "%s\n", __BASEFILE__);

    if (!bcm2835_init()) {
        fprintf( stderr, "%s bcm2835_init() Failed\n\n", __BASEFILE__ );
        return 1;
    }

    openRF95Stream();

    if (!rf95m.init()) {
        fprintf( stderr, "\nRF95 module init failed, Please verify wiring/module\n" );
    } else {
        initializeRF95Module();
        while (!force_exit) {
            loopRF95(&last_millis, &led_blink);
            // Let OS doing other tasks
            // For timed critical appliation you can reduce or delete
            // this delay, but this will charge CPU usage, take care and monitor
            bcm2835_delay(5);
        }
    }
    closeRF95Stream();
    return 0;
}

//Main Function
int main (int argc, const char* argv[] )
{
    return executeServerStream();
}
