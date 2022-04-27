#include <DWM3000.h>
#include <iostream>
#include <bitset>

const int PIN_CSN = 5;
const int PIN_RST = 25;

/* Frames used in the ranging process. See NOTE 2 below. */
byte tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};
byte rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0};
byte tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint32_t status;
byte frameSeq = 0;
byte rxBuffer[20];

uint64_t finalTXTimestamp;
uint64_t pollTXTimestamp;
uint64_t respRXTimestamp;

DWM3000 dw(PIN_CSN);
DWM3000Config config = {
    5,             /* Channel number. */
    PLEN_128,      /* Preamble length. Used in TX only. */
    PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,             /* TX preamble code. Used in TX only. */
    9,             /* RX preamble code. Used in RX only. */
    1,             /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    BR_6M8,        /* Data rate. */
    PHRMODE_STD,   /* PHY header mode. */
    PHRRATE_STD,   /* PHY header rate. */
    (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    STS_LEN_64     /* STS length see allowed values in Enum dwt_sts_lengths_e */
};

DW3000RFTXConfig rftxConfig = {
    0x34,       /* PGdly value */
    0xFDFDFDFD, /* Power value */
    0x3F        /* PG count */
};

void setup() {
    Serial.begin(115200);
    SPI.begin();

    dw.initialize(PIN_RST);
    dw.configure(&config);
    dw.configureRFTX(&rftxConfig);

    dw.write16bitReg(CIA_2, CIA_CONF, 0x4001);
    dw.write16bitReg(GEN_CFG_AES_1, TX_ANTD, 0x4001);

    dw.modify32bitReg(GEN_CFG_AES_1, ACK_RESP_T, 0xFFF00000, 0x2BC);
    dw.write32bitReg(GEN_CFG_AES_0, RX_FWTO, 0x12C);
    dw.or16bitReg(GEN_CFG_AES_0, SYS_CFG, 0x200);
    dw.write16bitReg(DRX, DTUNE1, 0x05);

    dw.or32bitReg(GPIO_CTRL, GPIO_MODE, 0x48000);

    // Print DEV ID
    char devString[64];
    dw.getPrintableDevID(devString);
    Serial.println(devString);
}

void loop() {
    tx_poll_msg[2] = frameSeq;
    dw.setTransmitData(sizeof(tx_poll_msg), tx_poll_msg);
    dw.startTransmit(false, true);

    while (!((status = dw.read32bitReg(GEN_CFG_AES_0, SYS_STATUS)) & 0x2427D000)) {};

    frameSeq++;
    if (status & 0x4000) {
        dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x4080);

        uint32_t frameLength = dw.read32bitReg(GEN_CFG_AES_0, RX_FINFO) & 0x3FF;
        if (frameLength <= 20) {
            dw.getReceiveData(frameLength, rxBuffer);
        }

        rxBuffer[2] = 0;
        if (memcmp(rxBuffer, rx_resp_msg, 10) == 0) {
            uint32_t finalTXTime;

            pollTXTimestamp = dw.getTimestamp();
            respRXTimestamp = dw.getReceiveTimestamp();
            finalTXTime = (respRXTimestamp + 0x02AA8118) >> 8;
            dw.setDelayedTime(finalTXTime);

            /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
            finalTXTimestamp = (((uint64_t)(finalTXTime & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            for (byte i = 0; i < 4; i++) {
                (&tx_final_msg[10])[i] = (uint8_t)pollTXTimestamp;
                (&tx_final_msg[14])[i] = (uint8_t)respRXTimestamp;
                (&tx_final_msg[18])[i] = (uint8_t)finalTXTimestamp;

                pollTXTimestamp >>= 8;
                respRXTimestamp >>= 8;
                finalTXTimestamp >>= 8;
            }

            tx_final_msg[2] = frameSeq;
            dw.setTransmitData(sizeof(tx_final_msg), tx_final_msg);
            dw.startTransmit(true, false);

            while (!dw.isTransmitDone()) {};
            dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x80);
            frameSeq++;
        }
    } else {
        dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x24279080);
    }

    delay(1000);
}
