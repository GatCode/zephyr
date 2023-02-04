#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <nrf.h>

uint8_t aa[4] = { 0x4f, 0x10, 0x16, 0x8b }; // 0x8b16104f

int main(void)
{
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
  	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

	irq_disable(RADIO_IRQn);

	NRF_RADIO->PCNF1 = 0x00000000UL;

	NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk;

	// NRF_RADIO->MODECNF0 = ((RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos) & RADIO_MODECNF0_DTX_Msk) |
	// 		      		((RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos) & RADIO_MODECNF0_RU_Msk);

	NRF_RADIO->PCNF0 =
		(((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) |
		(((8U) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk) |
		(((0UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) |
		((RADIO_PCNF0_PLEN_16bit << RADIO_PCNF0_PLEN_Pos) & RADIO_PCNF0_PLEN_Msk);
		// ((RADIO_PCNF0_S1INCL_Include << RADIO_PCNF0_S1INCL_Pos) & RADIO_PCNF0_S1INCL_Msk);

	NRF_RADIO->PCNF1 = 0x00000000UL;
	NRF_RADIO->PCNF1 &= ~(RADIO_PCNF1_MAXLEN_Msk | RADIO_PCNF1_STATLEN_Msk |
			      RADIO_PCNF1_BALEN_Msk | RADIO_PCNF1_ENDIAN_Msk);
	NRF_RADIO->PCNF1 |=
		(((255UL) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |
		(((0UL) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |
		(((3UL) << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |
		(((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk);

	volatile uint8_t packet[255];
	NRF_RADIO->PACKETPTR = (uint32_t)&packet[0];

	// Configure address Prefix0 + Base0 (own MAC)
    // NRF_RADIO->BASE0   = 0x0000BABE;
    // NRF_RADIO->PREFIX0 = 0x41 << RADIO_PREFIX0_AP0_Pos;
	NRF_RADIO->PREFIX0 = aa[3];
	NRF_RADIO->BASE0 = (aa[2] << 24) | (aa[1] << 16) | (aa[0] << 8);
	NRF_RADIO->RXADDRESSES = ((RADIO_RXADDRESSES_ADDR0_Enabled) << RADIO_RXADDRESSES_ADDR0_Pos);
	// AA of the SYNC info messages

	#define PDU_CRC_POLYNOMIAL     ((0x5bUL) | ((0x06UL) << 8) | ((0x00UL) << 16))
	NRF_RADIO->CRCCNF = (((RADIO_CRCCNF_SKIPADDR_Skip) << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk) |
	    				(((RADIO_CRCCNF_LEN_Disabled) << RADIO_CRCCNF_LEN_Pos) & RADIO_CRCCNF_LEN_Msk);
    NRF_RADIO->CRCPOLY = PDU_CRC_POLYNOMIAL; //0x0000AAAA;
    NRF_RADIO->CRCINIT = 0xdeadbeef;
	
	uint32_t chan = 5;
	switch (chan) {
	case 37:
		NRF_RADIO->FREQUENCY = 2;
		break;

	case 38:
		NRF_RADIO->FREQUENCY = 26;
		break;

	case 39:
		NRF_RADIO->FREQUENCY = 80;
		break;

	default:
		if (chan < 11) {
			NRF_RADIO->FREQUENCY = 4 + (chan * 2U);
		} else if (chan < 40) {
			NRF_RADIO->FREQUENCY = 28 + ((chan - 11) * 2U);
		} else {
			LL_ASSERT(0);
		}
		break;
	}

	NRF_RADIO->DATAWHITEIV = chan;
	NRF_RADIO->PCNF1 &= ~RADIO_PCNF1_WHITEEN_Msk;
	NRF_RADIO->PCNF1 |= ((1UL) << RADIO_PCNF1_WHITEEN_Pos) &
			    RADIO_PCNF1_WHITEEN_Msk;

	NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                      (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);


	// Device address match only if DACNF is also set && bit number 6 in S0 (TxAdd bit) is set

	uint8_t bdaddr[6];
	bdaddr[5] = 0x2a; // 2a:2f:c1:f2:a4:e3
	bdaddr[4] = 0x2f;
	bdaddr[3] = 0xc1;
	bdaddr[2] = 0xf2;
	bdaddr[1] = 0xa4;
	bdaddr[0] = 0xe3;

	uint8_t index = 0;
	NRF_RADIO->DAB[index] = ((uint32_t)bdaddr[3] << 24) |
		((uint32_t)bdaddr[2] << 16) |
		((uint32_t)bdaddr[1] << 8) |
		bdaddr[0];
	NRF_RADIO->DAP[index] = ((uint32_t)bdaddr[5] << 8) | bdaddr[4];
	NRF_RADIO->DACNF = 1UL << RADIO_DACNF_ENA0_Pos;
	NRF_RADIO->DACNF = 1UL << RADIO_DACNF_TXADD0_Pos;








//   // Packet receive buffer
//   volatile uint8_t packet[16];
  
//   // Start HFCLK from crystal oscillator. The radio needs crystal to function correctly.
//   NRF_CLOCK->TASKS_HFCLKSTART = 1;
//   while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
  
//   // Configure radio with 2Mbit Nordic proprietary mode
//   NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk;
  
//   // Configure packet with no S0,S1 or Length fields and 8-bit preamble.
// //   NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_LFLEN_Pos) |
// //                      (0 << RADIO_PCNF0_S0LEN_Pos) |
// //                      (0 << RADIO_PCNF0_S1LEN_Pos) | 
// //                      (RADIO_PCNF0_S1INCL_Automatic << RADIO_PCNF0_S1INCL_Pos) |
// //                      (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);

// 	NRF_RADIO->PCNF0 = ((1UL << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) |
// 					   ((8UL << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk) |
// 					   ((0 << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) | 
// 					   ((RADIO_PCNF0_PLEN_16bit << RADIO_PCNF0_PLEN_Pos) & RADIO_PCNF0_PLEN_Msk) |
// 					   ((RADIO_PCNF0_S1INCL_Include << RADIO_PCNF0_S1INCL_Pos) & RADIO_PCNF0_S1INCL_Msk);
					     
//   // Configure static payload length of 16 bytes. 3 bytes address, little endian with whitening enabled.
// //   NRF_RADIO->PCNF1 =  (255UL << RADIO_PCNF1_MAXLEN_Pos) |
// //                       (0UL << RADIO_PCNF1_STATLEN_Pos) |
// //                       (2  << RADIO_PCNF1_BALEN_Pos) | 
// //                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
// //                       (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

// 	NRF_RADIO->PCNF1 &= ~(RADIO_PCNF1_MAXLEN_Msk | RADIO_PCNF1_STATLEN_Msk |
// 			      RADIO_PCNF1_BALEN_Msk | RADIO_PCNF1_ENDIAN_Msk);
// 	NRF_RADIO->PCNF1 |=
// 		((255UL << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |
// 		((0UL << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |
// 		((3UL << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |
// 		((RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk);
  
//   // initialize whitening value
//   NRF_RADIO->DATAWHITEIV = 0x55;
  
//   // Configure address Prefix0 + Base0
//   NRF_RADIO->BASE0   = 0x0000BABE;
//   NRF_RADIO->PREFIX0 = 0x41 << RADIO_PREFIX0_AP0_Pos;
  
//   // Use logical address 0 (BASE0 + PREFIX0 byte 0)
//   NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos;
  
//   // Initialize CRC (two bytes)
//   NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |
//                       (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
//   NRF_RADIO->CRCPOLY = 0x0000AAAA;
//   NRF_RADIO->CRCINIT = 0x12345678;
  
//   // Enable fast rampup, new in nRF52
//   NRF_RADIO->MODECNF0 = ((RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos) & RADIO_MODECNF0_DTX_Msk) |
// 			      		((RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos) & RADIO_MODECNF0_RU_Msk);

// //   NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_DTX_B0 << RADIO_MODECNF0_DTX_Pos) |
// //                         (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
  
//   // receiving packets at 2400MHz
//   NRF_RADIO->FREQUENCY = 0 << RADIO_FREQUENCY_FREQUENCY_Pos;
  
//   // Configure address of the packet and logic address to use
//   NRF_RADIO->PACKETPTR = (uint32_t)&packet[0];
  
//   // Configure shortcuts to start as soon as READY event is received, and disable radio as soon as packet is received.
//   NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
//                       (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);
  
  // Continually receive packet
  while (1)
  {
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;

	// printk("Packet:");
	// for (uint8_t i = 0; i < 255; i++) {
	// 	printk("%02x ", packet[i]);
	// }
	// printk("\n");
    
	// MAC address match/miss
	uint8_t devmatch = NRF_RADIO->EVENTS_DEVMATCH;
	uint8_t devmiss = NRF_RADIO->EVENTS_DEVMISS;

	// Received address (A device address match occurred on the last received packet)
	uint8_t rxmatch = NRF_RADIO->RXMATCH;

	// Disable device addess match on addr 0
	// NRF_RADIO->DACNF = (1UL << RADIO_DACNF_ENA0_Pos) & RADIO_DACNF_ENA0_Msk;
	// NRF_RADIO->DACNF = (1UL << RADIO_DACNF_TXADD0_Pos) & RADIO_DACNF_TXADD0_Msk; //  TODO: DISABLE!!!


    printk("DEVMATCH: %u | DEVMISS: %u | RXMATCH: %x | CRCOK: %u | Packet:", devmatch, devmiss, rxmatch, NRF_RADIO->EVENTS_CRCOK);
	for (uint8_t i = 0; i < 255; i++) {
		printk("%02x ", packet[i]);
	}
	printk("\n");
	printk("--------------------------------------\n");
	// __NOP();
  }
}