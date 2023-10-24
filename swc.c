/* D13 = PORTB5 -> SCK
 * D12 = PORTB4 -> MISO
 * D11 = PORTB3 -> MOSI
 * D10 = PORTB2 -> CS mcp2515
 * D9  = PORTB1 -> CS MCP41050-E/P
 *
 * A0 = digipot enable
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stddef.h>
#include <string.h>

/* SPI port, pins, ddr and dd */
#define PORT_SPI PORTB
#define DDR_SPI  DDRB

#define SCK_PIN     PORTB5
#define MISO_PIN    PORTB4
#define MOSI_PIN    PORTB3
#define CS_MCP_PIN  PORTB2
#define CS_DPOT_PIN PORTB1

#define SCK_DD     DDB5
#define MISO_DD    DDB4
#define MOSI_DD    DDB3
#define CS_MCP_DD  DDB2
#define CS_DPOT_DD DDB1

/* spi instructions for the mcp2515 */
#define RESET            0b11000000
#define READ             0b00000011
#define WRITE            0b00000010
#define BIT_MODIFY       0b00000101
#define READ_STATUS      0b10100000
#define READ_RX_RXB0SIDH 0b10010000
#define READ_RX_RXB0DATA 0b10010010
#define READ_RX_RXB1SIDH 0b10010100
#define READ_RX_RXB1DATA 0b10010110

/* mcp2515 addresses */
#define CNF1     0x2A
#define CNF2     0x29
#define CNF3     0x28
#define CANCTRL  0xF
#define CANSTAT  0xE
#define CANINTF  0x2C
#define RXB0CTRL 0x60
#define RXB1CTRL 0x70
#define EFGL     0x2D
#define CANINTE  0x2B
#define RXF0SIDH 0x00
#define RXF0SIDL 0x01
#define RXF0EID8 0x02
#define RXF0EID0 0x03
#define RXF2SIDH 0x08
#define RXF2SIDL 0x09
#define RXF2EID8 0x0A
#define RXF2EID0 0x0B
#define RXM0SIDH 0x20
#define RXM0SIDL 0x21
#define RXM0EID8 0x22
#define RXM0EID0 0x23
#define RXM1SIDH 0x24
#define RXM1SIDL 0x25
#define RXM1EID8 0x26
#define RXM1EID0 0x27

/* mcp2515 pins/bits */
#define CANCTRL_REQOP     0b11100000
#define CANSTAT_OPMOD     0b11100000
#define CANINTF_RX1IF     0b00000010
#define CANINTF_RX0IF     0b00000001
#define READ_STATUS_RX0IF 0b00000001
#define READ_STATUS_RX1IF 0b00000010
#define RXBnCTRL_RXRTR    0b00001000
#define EFLG_RX1OVR       0b10000000
#define EFLG_RX0OVR       0b01000000
#define CANINTE_RX0IE     0b00000001
#define CANINTE_RX1IE     0b00000010
#define RXBnCTRL_RXM0     0b00100000
#define RXBnCTRL_RXM1     0b01000000
#define RXBnCTRL_RXM_F    0b00000000
#define RXBnCTRL_RXM_ALL  0b01100000
#define SIDL_SID          0b11100000
#define SIDL_EID          0b00000011
#define RXBnSIDL_EXIDE    0b00001000
#define RXFnSIDL_EXIDE    0b00001000
#define RXFMnSIDL_EXIDE   0b00001000

/* mcp2515 modes */
#define NORMAL_MODE    (0b000 << 5)
#define CONFIGURE_MODE (0b100 << 5)

/* 8mhz clock at 50kbps */
#define CFG1 0x3
#define CFG2 0xB4
#define CFG3 0x86

/* mcp2515 commands, but not functions */
#define CLEAR_CANINTF(MASK) \
	do { \
		mcp_bit_modify(CANINTF, (MASK), 0b0); \
	} while (0); \

/* indexes of value for mcp_read_message_in_rxbn */
#define SIDH_INDEX 0
#define SIDL_INDEX SIDH_INDEX + 1
#define EID8_INDEX SIDL_INDEX + 1
#define EID0_INDEX EID8_INDEX + 1
#define DLC_INDEX  EID0_INDEX + 1

/* use the remaining three bits of the uint32_t
 * variable for different flags - not necessary
 */
#define ID_EXTENDED_FLAG 0x80000000UL
#define ID_REMOTE_FLAG   0x40000000UL
#define ID_ERROR_FLAG    0x20000000UL
#define ID_STANDARD_MASK 0x000007FFUL
#define ID_EXTENDED_MASK 0x1FFFFFFFUL
#define ID_ERROR_MASK    0x1FFFFFFFUL

/* spi instructions for the MCP41050-E/P */
#define DIGIPOT_WRITE    0b00010011
#define DIGIPOT_SHUTDOWN 0b00100011

/* port and pin of optocoupler that controls the digipot */
#define DIGIPOT_OPTOCOUPLER_PORT   PORTD
#define DIGIPOT_OPTOCOUPLER_DD     DDRD
#define DIGIPOT_OPTOCOUPLER_PIN    PORTD3
#define DIGIPOT_OPTOCOUPLER_DD_BIT DDD3

/* values for the write command for the digital potentiometer
 * 250, 238, 212, 195, 167, 100
 */
#define SOURCE_VAL         0xFA
#define MUTE_VAL           0xEE
#define NEXT_TRACK_VAL     0xD4
#define PREVIOUS_TRACK_VAL 0xC3
#define VOLUME_UP_VAL      0xA7
#define VOLUME_DOWN_VAL    0x86

/* steering wheel control can id (contains the extended frame format flag) and size */
#define SWC_CAN_ID 0x86354000UL
#define SWC_DATA_LENGTH 2
#define SWC_DATA_SIZE (SWC_DATA_LENGTH * sizeof(uint8_t))

/* set and unset instructions for PIN on PORT */
#define SET(PORT, PIN) ((PORT) = (PORT) | (1 << (PIN)))
#define UNSET(PORT, PIN) ((PORT) = (PORT) & ~(1 << (PIN)))

/* message structure */
struct message {
	uint32_t id;
	uint8_t data_size;
	uint8_t data[8];
};

struct RXB_READ_MESSAGE {
	uint8_t READ_RX_RXBSIDH, CANINTF_RXIF, RXBCTRL, READ_RX_RXBDATA;
};

/* function declarations */
static void spi_master_init(void);
static uint8_t spi_transmit(const uint8_t to_send);
static void mcp_set_register(const uint8_t reg, const uint8_t val);
static void mcp_bit_modify(const uint8_t reg, const uint8_t mask, const uint8_t val);
static uint8_t mcp_read_register(const uint8_t reg);
static uint8_t mcp_read_status(void);
static void mcp_set_mode(const uint8_t mode);
static struct message *mcp_read_message(struct message *msg);
static struct message *mcp_read_message_in_rxbn(const uint8_t n, struct message *msg);
static void digipot_write(const uint8_t val);
static void digipot_shutdown(void);
static void digipot_optocoupler_write(const uint8_t val);
static void digipot_optocoupler_shutdown(void);
static void message_handler(const struct message *msg);
static void show(const uint8_t to_show);

/* global variables */
static const struct RXB_READ_MESSAGE rxb_read[2] = {
	{.READ_RX_RXBSIDH = READ_RX_RXB0SIDH, .CANINTF_RXIF = CANINTF_RX0IF,
	 .RXBCTRL = RXB0CTRL, .READ_RX_RXBDATA = READ_RX_RXB0DATA},
	{.READ_RX_RXBSIDH = READ_RX_RXB1SIDH, .CANINTF_RXIF = CANINTF_RX1IF,
	 .RXBCTRL = RXB1CTRL, .READ_RX_RXBDATA = READ_RX_RXB1DATA}
};
static const struct message source         = {.id = SWC_CAN_ID,
                                              .data_size = SWC_DATA_SIZE,
                                              .data = {0x4,  0x0}
};
static const struct message mute           = {.id = SWC_CAN_ID,
                                              .data_size = SWC_DATA_SIZE,
                                              .data = {0x20, 0x0}
};
static const struct message next_track     = {.id = SWC_CAN_ID,
                                              .data_size = SWC_DATA_SIZE,
                                              .data = {0x10, 0x0}};
static const struct message previous_track = {.id = SWC_CAN_ID,
                                              .data_size = SWC_DATA_SIZE,
                                              .data = {0x8,  0x0}
};
static const struct message volume_up      = {.id = SWC_CAN_ID,
                                              .data_size = SWC_DATA_SIZE,
                                              .data = {0x80, 0x0}};
static const struct message volume_down    = {.id = SWC_CAN_ID,
                                              .data_size = SWC_DATA_SIZE,
                                              .data = {0x40, 0x0}
};
static const struct message button_release = {.id = SWC_CAN_ID,
                                              .data_size = SWC_DATA_SIZE,
                                              .data = {0x0,  0x0}
};
volatile uint8_t interrupted = 0;

/* function definitions */
static void
spi_master_init(void)
{
	DDR_SPI |= (1 << SCK_DD) | (1 << MOSI_DD) | (1 << CS_MCP_DD) | (1 << CS_DPOT_DD);
	SET(PORT_SPI, CS_MCP_PIN);
	SET(PORT_SPI, CS_DPOT_PIN);
	SPCR = (1 << SPE) | (1 << MSTR); // | (1 << SPR0);
	SET(SPSR, SPI2X);
}

static uint8_t
spi_transmit(const uint8_t to_send)
{
	SPDR = to_send;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}

static void
mcp_set_register(const uint8_t reg, const uint8_t val)
{
	UNSET(PORT_SPI, CS_MCP_PIN);
	spi_transmit(WRITE);
	spi_transmit(reg);
	spi_transmit(val);
	SET(PORT_SPI, CS_MCP_PIN);
}

static void
mcp_bit_modify(const uint8_t reg, const uint8_t mask, const uint8_t val)
{
	UNSET(PORT_SPI, CS_MCP_PIN);
	spi_transmit(BIT_MODIFY);
	spi_transmit(reg);
	spi_transmit(mask);
	spi_transmit(val);
	SET(PORT_SPI, CS_MCP_PIN);
}

static uint8_t
mcp_read_register(const uint8_t reg)
{
	uint8_t ret = 0x0;
	UNSET(PORT_SPI, CS_MCP_PIN);
	spi_transmit(READ);
	spi_transmit(reg);
	ret = spi_transmit(0x0);
	SET(PORT_SPI, CS_MCP_PIN);
	return ret;
}

static uint8_t
mcp_read_status(void)
{
	uint8_t ret = 0x0;
	UNSET(PORT_SPI, CS_MCP_PIN);
	spi_transmit(READ_STATUS);
	ret = spi_transmit(0x0);
	SET(PORT_SPI, CS_MCP_PIN);
	return ret;
}

static void
mcp_set_mode(const uint8_t mode)
{
	mcp_bit_modify(CANCTRL, CANCTRL_REQOP, mode);

	while (mcp_read_register(CANSTAT) & CANSTAT_OPMOD != mode);
}

static struct message *
mcp_read_message(struct message *msg)
{
	uint8_t status = mcp_read_status(), efgl = mcp_read_register(EFGL);

	/* i don't know if this is necessary. it clears
	 * overflows and interrupts just in case
	 */
	if (efgl & (EFLG_RX0OVR | EFLG_RX1OVR))
		mcp_bit_modify(EFGL, EFLG_RX0OVR | EFLG_RX1OVR, 0b0);

	if (status & READ_STATUS_RX0IF)
		return mcp_read_message_in_rxbn(0, msg);
	else if (status & READ_STATUS_RX1IF)
		return mcp_read_message_in_rxbn(1, msg);
	else
		return NULL;
}

static struct message *
mcp_read_message_in_rxbn(const uint8_t n, struct message *msg)
{
	uint8_t value[5] = {0}, i;
	uint32_t id = 0;
	uint8_t data_size = 0, data[8] = {0}, ctrl;

	UNSET(PORT_SPI, CS_MCP_PIN);
	spi_transmit(rxb_read[n].READ_RX_RXBSIDH);
	for (i = 0; i < 5; i++)
		value[i] = spi_transmit(0x0);
	SET(PORT_SPI, CS_MCP_PIN);

	/* SIDH and SIDL */
	id = ((uint32_t)value[SIDH_INDEX] << 3) + ((uint32_t)value[SIDL_INDEX] >> 5);

	if (value[SIDL_INDEX] & RXBnSIDL_EXIDE == RXBnSIDL_EXIDE) {
		/* EID17 and EID16 */
		id = (id << 2) + ((uint32_t)value[SIDL_INDEX] & 0b00000011);
		/* EID15 to EID8 */
		id = (id << 8) + (uint32_t)value[EID8_INDEX];
		/* EID7 to EID0 */
		id = (id << 8) + (uint32_t)value[EID0_INDEX];
		id = id | ID_EXTENDED_FLAG;
	}

	data_size = value[DLC_INDEX] & 0b00001111;
	
	if (data_size > 8) {
		CLEAR_CANINTF(rxb_read[n].CANINTF_RXIF);
		return NULL;
	}
	
	ctrl = mcp_read_register(rxb_read[n].RXBCTRL);
	if (ctrl & RXBnCTRL_RXRTR)
		id = id | ID_REMOTE_FLAG;
	
	msg->id = (uint32_t)id;
	msg->data_size = data_size;

	/* RXB0DATA = RXB0D0 */
	UNSET(PORT_SPI, CS_MCP_PIN);
	spi_transmit(rxb_read[n].READ_RX_RXBDATA);
	for (i = 0; i < data_size; i++)
		msg->data[i] = spi_transmit(0x0);
	SET(PORT_SPI, CS_MCP_PIN);

	CLEAR_CANINTF(rxb_read[n].CANINTF_RXIF);

	return msg;
}

static void
digipot_write(const uint8_t val)
{
	UNSET(PORT_SPI, CS_DPOT_PIN);
	spi_transmit(DIGIPOT_WRITE);
	spi_transmit(val);
	SET(PORT_SPI, CS_DPOT_PIN);
}

static void
digipot_shutdown(void)
{
	UNSET(PORT_SPI, CS_DPOT_PIN);
	spi_transmit(DIGIPOT_SHUTDOWN);
	spi_transmit(0x0);
	SET(PORT_SPI, CS_DPOT_PIN);
}

static void
digipot_optocoupler_write(const uint8_t val)
{
	digipot_write(val);
	SET(DIGIPOT_OPTOCOUPLER_PORT, DIGIPOT_OPTOCOUPLER_PIN);
}

static void
digipot_optocoupler_shutdown(void)
{
	digipot_shutdown();
	UNSET(DIGIPOT_OPTOCOUPLER_PORT, DIGIPOT_OPTOCOUPLER_PIN);
}

static void
message_handler(const struct message *msg)
{
	if (msg->id != SWC_CAN_ID)
		return;
	if (msg->data_size != SWC_DATA_SIZE)
		return;

	if (memcmp(msg->data, source.data, SWC_DATA_SIZE) == 0)
		digipot_optocoupler_write(SOURCE_VAL);
	else if (memcmp(msg->data, mute.data, SWC_DATA_SIZE) == 0)
		digipot_optocoupler_write(MUTE_VAL);
	else if (memcmp(msg->data, next_track.data, SWC_DATA_SIZE) == 0)
		digipot_optocoupler_write(NEXT_TRACK_VAL);
	else if (memcmp(msg->data, previous_track.data, SWC_DATA_SIZE) == 0)
		digipot_optocoupler_write(PREVIOUS_TRACK_VAL);
	else if (memcmp(msg->data, volume_up.data, SWC_DATA_SIZE) == 0)
		digipot_optocoupler_write(VOLUME_UP_VAL);
	else if (memcmp(msg->data, volume_down.data, SWC_DATA_SIZE) == 0)
		digipot_optocoupler_write(VOLUME_DOWN_VAL);
	else if (memcmp(msg->data, button_release.data, SWC_DATA_SIZE) == 0)
		digipot_optocoupler_shutdown();
}

ISR(INT0_vect)
{
	interrupted = 1;
}

/* main */
int
main(void)
{
	struct message msg = {0};
	/* initialise spi */
	spi_master_init();

	/* initialise mcp2515 into configuration mode */
	UNSET(PORT_SPI, CS_MCP_PIN);
	spi_transmit(RESET);
	SET(PORT_SPI, CS_MCP_PIN);

	/* despite the mcp2515 being just reset and the resetting
	 * putting you into configure mode, i want to make sure we are
	 * in configure mode
	 */
	mcp_set_mode(CONFIGURE_MODE);

	/* set kbps of can */
	mcp_set_register(CNF1, CFG1);
	mcp_set_register(CNF2, CFG2);
	mcp_set_register(CNF3, CFG3);

	/* use no filter criteria */
	mcp_bit_modify(RXB0CTRL, RXBnCTRL_RXM0 | RXBnCTRL_RXM1, RXBnCTRL_RXM_ALL);
	mcp_bit_modify(RXB1CTRL, RXBnCTRL_RXM0 | RXBnCTRL_RXM1, RXBnCTRL_RXM_ALL);

	/* set mode of mcp2515 to normal mode */
	mcp_set_mode(NORMAL_MODE);

	/* initialise interrupt on atmega328p */
	SET(EIMSK, INT0);

	/* interrupt on the rising edge of INT0 - PORTD2 */
	SET(EICRA, ISC01);
	SET(EICRA, ISC00);
	sei();

	/* initialise interrupt on mcp2515 */
	mcp_bit_modify(CANINTE, CANINTE_RX0IE | CANINTE_RX1IE, 0b11111111);

	/* initialise digipot */
	SET(DIGIPOT_OPTOCOUPLER_DD, DIGIPOT_OPTOCOUPLER_DD_BIT);
	digipot_optocoupler_shutdown();
	//digipot_optocoupler_write(VOLUME_DOWN_VAL);
	//digipot_optocoupler_write(SOURCE_VAL);
	//digipot_optocoupler_shutdown();

	/* this is a fix for a weird bug.
	 * whenever i turn the key, so the project gets power and
	 * turned on, on the first powerup of the mcu (controller),
	 * it doesn't communicate with the mcp2515 module, so it
	 * won't work. it requires a restart, achieved by either
	 * letting the key on the first position before turning it
	 * further to start the engine, which will make it work,
	 * a restart from the the restart button, or a restart from
	 * the watchdog.
	 *
	 * here i'm resetting the watchdog timer and setting it
	 * to restart the mcu after 1 second unless the timer watchdog
	 * is disabled. the watchdog is disabled upon the first
	 * succesful communication with the mcp2515 module
	 */
	wdt_reset();
	wdt_enable(WDTO_1S);

	/* wait for a interrupt */
	while (1) {
		if (interrupted) {
			int status = mcp_read_status(), efgl = mcp_read_register(EFGL);

			wdt_disable();
			interrupted = 0;

			if (efgl & (EFLG_RX0OVR | EFLG_RX1OVR))
				mcp_bit_modify(EFGL, EFLG_RX0OVR | EFLG_RX1OVR, 0b0);

			if (status & READ_STATUS_RX0IF && mcp_read_message_in_rxbn(0, &msg))
				message_handler(&msg);

			if (status & READ_STATUS_RX1IF && mcp_read_message_in_rxbn(1, &msg))
				message_handler(&msg);
		}
	}
		

	return 0;
}
