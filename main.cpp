#include "mbed.h"
#include "main.h"
#include "sx1276-hal.h"
#include "debug.h"

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE			0

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA			1
#define USE_MODEM_FSK			!USE_MODEM_LORA

#define RF_FREQUENCY			920000000 // Hz
#define TX_OUTPUT_POWER			14        // 14 dBm

#if USE_MODEM_LORA == 1

#define LORA_BANDWIDTH			2         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR		7         // [SF7..SF12]
#define LORA_CODINGRATE			1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH		8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT		5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON	false
#define LORA_FHSS_ENABLED		false  
#define LORA_NB_SYMB_HOP		4     
#define LORA_IQ_INVERSION_ON		false
#define LORA_CRC_ENABLED		true
    
#elif USE_MODEM_FSK == 1

#define FSK_FDEV			25000     // Hz
#define FSK_DATARATE			19200     // bps
#define FSK_BANDWIDTH			50000     // Hz
#define FSK_AFC_BANDWIDTH		83333     // Hz
#define FSK_PREAMBLE_LENGTH		5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON	false
#define FSK_CRC_ENABLED			true
    
#else
#error "Please define a modem in the compiler options."
#endif

#define RX_TRP_TIMEOUT			50000 // in us
#define RX_TIS_TIMEOUT			50000 // in us
#define BUFFER_SIZE			32        // Define the payload size here

DigitalOut led(LED1);
InterruptIn button(USER_BUTTON);

/*
 *  Global variables declarations
 */
typedef enum
{
	IDLE,
	RX,
	RX_TIMEOUT,
	RX_ERROR,
	TX,
	TX_TIMEOUT,
} AppStates_t;

typedef enum
{
	NO_ACT,
	TRP_REQ,
	TRP_RESP,
	TIS_REQ,
	TIS_RESP,
} AppMode_t;

volatile AppStates_t State = IDLE;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1276MB1xAS Radio( NULL );

uint16_t BufferSize = BUFFER_SIZE;
char Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;
int32_t AccRssi;
int32_t AccSnr;

// Process State
AppMode_t Mode = NO_ACT;
int SID = '0';
uint16_t CurCount;
uint16_t EndCount;

uint32_t RxTimeout;
int SkipRx = 0;

static uint16_t GetDigit(char *p)
{
	uint8_t i;
	uint16_t v = 0;

	for (i = 0; i < 3; i++)
		v = v * 10 + p[i] - '0';
	return v;
}

static void SendTRPReq()
{
	debug("TRP_REQ %d %03d\r\n", SID, EndCount);
	Buffer[0] = 'a';
	Buffer[1] = SID;
	sprintf(&Buffer[2], "%03d", EndCount);
	Radio.Send((uint8_t*)Buffer, 2 + 3);
}
static void SendTRPResp()
{
	debug_if(DEBUG_MESSAGE || CurCount+1 == EndCount, "TRP_RESP %d %03d (%03d)\r\n",
		 SID, CurCount, EndCount);
	Buffer[0] = 'b';
	Buffer[1] = SID;
	sprintf(&Buffer[2], "%03d", CurCount++);
	sprintf(&Buffer[5], "%03d", AccRssi * -1);
	Radio.Send((uint8_t*)Buffer, 2 + 6);
}

static void SendTISReq()
{
	debug_if(DEBUG_MESSAGE || CurCount == 0, "TIS_REQ %d %03d %03d\r\n",
		 SID, CurCount+1, EndCount);
	Buffer[0] = 'c';
	Buffer[1] = SID;
	sprintf(&Buffer[2], "%03d%03d", CurCount++, EndCount);
	Radio.Send((uint8_t*)Buffer, 2 + 6);
}

static void SendTISResp()
{
	debug("TIS_RESP %d %03d (RSSI: %d, SNR:%d)\r\n", SID, CurCount,
	      AccRssi / CurCount, AccSnr / CurCount);
	Buffer[0] = 'd';
	Buffer[1] = SID;
	sprintf(&Buffer[2], "%03d", CurCount);
	sprintf(&Buffer[5], "%03d", AccRssi / CurCount * -1);
	Radio.Send((uint8_t*)Buffer, 2 + 6);
}

static void GenSID()
{
	if (++SID > '9')
		SID = '1';
}
  
static void StartTRP(int count)
{
	GenSID();
	CurCount = 0;
	EndCount = count;
	Mode = TRP_REQ;
	AccRssi = 0;
	AccSnr = 0;
	SendTRPReq();
}

static void StartTIS(int count)
{
	GenSID();
	CurCount = 0;
	EndCount = count;
	Mode = TIS_REQ;
	AccRssi = 0;
	AccSnr = 0;
	SendTISReq();
}

static void ReportTRP()
{
	debug("TRP Result: %d/%d (RSSI: %d, SNR: %d)\r\n", CurCount, EndCount,
	      AccRssi / CurCount, AccSnr / CurCount);
}

static void fire()
{
	printf("Firing\r\n");
//	StartTRP(100);
	StartTIS(500);
}

static void RxProc()
{
	uint16_t t;
	if (BufferSize <= 0 || SkipRx)  {
		SkipRx = 0;
		Radio.Rx(RxTimeout);
		return;
	}
	
	switch (Buffer[0]) {
	case 'a':		// from master
		Mode = TRP_RESP;
		SID = Buffer[1];
		CurCount = 0;
		EndCount = GetDigit(&Buffer[2]);
		AccRssi = RssiValue;
		AccSnr = SnrValue;
		debug("TRP Req: %c %d (RSSI:%d, SNR:%d)\r\n",
		      SID, EndCount, RssiValue, SnrValue);
		SendTRPResp();
		break;
	case 'b':		// from slave
		if (Mode == TRP_REQ) {
			CurCount++;
			if (memcmp(&Buffer[2],
				   &Buffer[2 + 3], 3) == 0) {
				// finished
				ReportTRP();
				Mode = NO_ACT;
				RxTimeout = 0;
				break;
			}
				
		} else {
			RxTimeout = RX_TRP_TIMEOUT;
		}
		Radio.Rx(RxTimeout);
		break;
	case 'c':		// from master
		if (Mode != TIS_RESP || SID != Buffer[1]) {
			SID = Buffer[1];
			CurCount = 0;
			Mode = TIS_RESP;
			AccRssi = 0;
			AccSnr = 0;
		}
		CurCount++;
		AccRssi += RssiValue;
		AccSnr += SnrValue;
		t = GetDigit(&Buffer[2]);
		EndCount = GetDigit(&Buffer[2] + 3);
		//debug("TIS Req: %d/%d (RSSI:%d, SNR:%d)\r\n", t, EndCount, RssiValue, SnrValue);
		if (t + 1 == EndCount) {
			SendTISResp();
		} else {
			RxTimeout = RX_TRP_TIMEOUT;
			Radio.Rx(RxTimeout);
		}
		break;
	case 'd':		// from slave
		if (Mode == TIS_REQ) {
			CurCount = GetDigit(&Buffer[2]);
			debug("TIS Result: %d, %d (RSSI:%d, SNR:%d)\r\n", CurCount, EndCount, RssiValue, SnrValue);
			Mode = NO_ACT;
			StartTIS(100);
			break;
		}
		RxTimeout = 0;
		Radio.Rx(RxTimeout);
		break;
	default:
		Radio.Rx(RxTimeout);
		break;
	}
}

static void TxProc()
{
	switch (Mode) {
	case TRP_REQ:
		RxTimeout = RX_TRP_TIMEOUT;
		Radio.Rx(RxTimeout);
		break;
	case TRP_RESP:
		if (CurCount >= EndCount) {
			// finished
			Mode = NO_ACT;
			RxTimeout = 0;
			Radio.Rx(RxTimeout);
		} else {
			SendTRPResp();
		}
		break;
	case TIS_REQ:
		if (CurCount >= EndCount) {
			// finished
			RxTimeout = RX_TIS_TIMEOUT;
			Radio.Rx(RxTimeout);
		} else {
			SendTISReq();
		}
		break;
	case TIS_RESP:
		Mode = NO_ACT;
		RxTimeout = 0;
		Radio.Rx(RxTimeout);
		break;
	default:
		Radio.Rx(RxTimeout);
		break;
	}
}

int main() 
{
	debug("\r\nSX1276 PER Test Application\r\n");
	button.rise(&fire);

	// Initialize Radio driver
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.RxError = OnRxError;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	Radio.Init(&RadioEvents);
    
	// verify the connection with the board
	while (Radio.Read( REG_VERSION ) == 0x00) {
		debug("Radio could not be detected!\r\n", NULL);
		wait(1);
	}
            
	debug_if((DEBUG_MESSAGE & (Radio.DetectBoardType() == SX1276MB1LAS)) , 
		 "\r\n> Board Type: SX1276MB1LAS <\r\n" );
	debug_if((DEBUG_MESSAGE & (Radio.DetectBoardType() == SX1276MB1MAS)) , 
		 "\r\n> Board Type: SX1276MB1MAS <\r\n" );
	
	Radio.SetChannel(RF_FREQUENCY);

#if USE_MODEM_LORA == 1
	debug_if(LORA_FHSS_ENABLED, "\r\n> LORA FHSS Mode <\r\n");
	debug_if(!LORA_FHSS_ENABLED, "\r\n> LORA Mode <\r\n");
	
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
			  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
			  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
			  LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP, 
			  LORA_IQ_INVERSION_ON, 2000000);
	
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
			  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
			  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
			  LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP, 
			  LORA_IQ_INVERSION_ON, true);
#elif USE_MODEM_FSK == 1
	debug("\r\n> FSK Mode <\r\n");
	Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
			  FSK_DATARATE, 0,
			  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
			  FSK_CRC_ENABLED, 0, 0, 0, 2000000);
	
	Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
			  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
			  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, FSK_CRC_ENABLED,
			  0, 0, false, true);
#else
#error "Please define a modem in the compiler options."
#endif
     
	debug_if(DEBUG_MESSAGE, "Starting PER loop\r\n"); 
        
	led = 0;
	Radio.Rx(0);

	while (1) {
		__disable_irq();
		AppStates_t s = State;
		State = IDLE;
		__enable_irq();
		switch (s) {
		default:
		case IDLE:
			break;
		case TX:
		case TX_TIMEOUT:
			TxProc();
			break;
		case RX:
			RxProc();
			break;
		case RX_TIMEOUT: // only TIS_RESP
			if (Mode == TRP_REQ) {
				ReportTRP();
				Mode = NO_ACT;
				RxTimeout = 0;
			} else if (Mode == TIS_RESP) {
				SendTISResp();
				break;
			}
			Radio.Rx(RxTimeout);
			break;
		case RX_ERROR:
			Radio.Rx(RxTimeout);
			break;
		}
	}
}

void OnTxDone(void)
{
	Radio.Sleep();
	State = TX;
	debug_if(DEBUG_MESSAGE, "> OnTxDone\r\n");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	Radio.Sleep();
	BufferSize = size;
	memcpy(Buffer, payload, BufferSize);
	RssiValue = rssi;
	SnrValue = snr;
	State = RX;
	debug_if(DEBUG_MESSAGE, "> OnRxDone\r\n");
}

void OnTxTimeout(void)
{
	Radio.Sleep();
	State = TX_TIMEOUT;
	debug_if(DEBUG_MESSAGE, "> OnTxTimeout\r\n");
}

void OnRxTimeout(void)
{
	Radio.Sleep();
	Buffer[BufferSize] = 0;
	State = RX_TIMEOUT;
	debug_if(DEBUG_MESSAGE, "> OnRxTimeout\r\n");
}

void OnRxError(void)
{
	Radio.Sleep();
	SkipRx = 1;
	State = RX_ERROR;
	debug_if(DEBUG_MESSAGE, "> OnRxError\r\n");
}
