// Package sx127x provides a driver for SX127x LoRa transceivers.
//
// Datasheet:
// https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V6.pdf
//
// LoRa Configuration Parameters:
//
// Frequency: is the frequency the tranceiver uses. Valid frequencies depend on
//  the type of LoRa module, typically around 433MHz or 866MHz. It has
//  a granularity of about 23Hz, how close it can be to others depends on the
//  Bandwidth being used.
//
// Bandwidth: is the bandwidth used for tranmissions, ranging from 7k8 to 512k
//  A higher bandwidth gives faster transmissions, lower gives greater range
//
// SpreadingFactor: is how a transmission is spread over the spectrum. It ranges
//  from 6 to 12, a higher value gives greater range but slower transmissions.
//
// CodingRate: is the cyclic error coding used to improve the robustness of the
//  transmission. It ranges from 5 to 8, a higher value gives greater
//  reliability but slower transmissions.
//
// TxPower: is the power used for the transmission, ranging from 1 to 20.
//  A higher power gives greater range. Regulations in your country likely
//  limit the maximum power permited.
//
// Presently this driver is only synchronous and so does not use any DIOx pins
//
package sx127x

import (
	"errors"
	"machine"
	"time"
)

// Device wraps an SPI connection to a SX127x device.
type Device struct {
	spi                machine.SPI
	csPin              machine.Pin
	rstPin             machine.Pin
	packetIndex        uint8
	implicitHeaderMode bool
	rxPktBuf           []byte
	rxPktChan          chan []byte
	cnf                Config
}

// Config holds the LoRa configuration parameters
type Config struct {
	Frequency            uint32
	SpreadingFactor      uint8
	Bandwidth            int32
	CodingRate           uint8
	TxPower              int8
	SyncWord             uint8
	PaBoost              bool
	ImplicitHeaderModeOn bool
	CrcOn                bool
	AgcAutoOn            bool
	LowDataRateOptimOn   bool
	InvertIQRx           bool
	InvertIQTx           bool
}

const (
	MAX_PACKET_SIZE = 255
)

// New creates a new SX127x connection. The SPI bus must already be configured.
func New(spi machine.SPI, csPin machine.Pin, rstPin machine.Pin) Device {

	k := Device{
		spi:    spi,
		csPin:  csPin,
		rstPin: rstPin,
	}

	k.rxPktBuf = make([]byte, 0, MAX_PACKET_SIZE)
	k.rxPktChan = make(chan []byte)
	return k

}

// Initializes the LoRa module
func (d *Device) Init(cfg Config) (err error) {
	d.cnf = cfg

	d.csPin.High()
	d.Reset()

	if d.GetVersion() != 0x12 {
		return errors.New("SX127x module not found")
	}
	return nil
}

// ConfigureLoraModem prepares for LORA communications
func (d *Device) ConfigureLoraModem() {

	// Sleep mode required to go LOra
	d.SetOpMode(OPMODE_SLEEP)
	// Set Lora mode (from sleep)
	d.OpModeLora()
	// Switch to standby mode
	d.SetOpMode(OPMODE_STANDBY)
	// Configure Frequency
	d.ConfigureChannel(d.cnf.Frequency)
	// Set Bandwidth
	d.SetBandwidth(d.cnf.Bandwidth)
	// Set Coding Rate
	d.SetCodingRate(d.cnf.CodingRate)
	// Disable IQ Polarization
	d.SetInvertedIQ(false)
	// Set implicit header
	d.SetImplicitHeaderModeOn(false)
	// We want CRC
	d.SetRxPayloadCrcOn(true)
	d.SetAgcAutoOn(true)
	if d.GetBandwidth() == 125000 && (d.GetSpreadingFactor() == 11 || d.GetSpreadingFactor() == 12) {
		d.SetLowDataRateOptimOn(true)
	}
	// Set Lora Sync
	d.SetSyncWord(0x34)
	// Configure Output Power
	d.WriteRegister(REG_PA_RAMP, (d.ReadRegister(REG_PA_RAMP)&0xF0)|0x08) // set PA ramp-up time 50 uSec
	d.WriteRegister(REG_PA_CONFIG, 0xFF)                                  //PA_BOOST MAX
	d.SetOCP(140)                                                         // Over Current protection

	// RX and premamble
	d.WriteRegister(REG_PREAMBLE_MSB, 0x00)     // Preamble set to 8 symp
	d.WriteRegister(REG_PREAMBLE_LSB, 0x08)     // -> 0x0008 + 4 = 12
	d.WriteRegister(REG_SYMB_TIMEOUT_LSB, 0x25) //Rx Timeout 37 symbol

	// set FIFO base addresses
	d.WriteRegister(REG_FIFO_TX_BASE_ADDR, 0)
	d.WriteRegister(REG_FIFO_RX_BASE_ADDR, 0)
}

// TxLora sends a packet in Lora mode
func (d *Device) TxLora(payload []byte) {

	// Are we already in Lora mode ?
	r := d.ReadRegister(REG_OP_MODE)
	if (r & OPMODE_LORA) != OPMODE_LORA {
		println("LoraTx Error 1 ... Aborting, opmode=", d.ReadRegister(REG_OP_MODE))
	}
	// Switch to standby mode
	d.SetOpMode(OPMODE_STANDBY)

	//Switch DIO0 to TxDone
	d.WriteRegister(REG_DIO_MAPPING_1, 0x40)
	// Clear all radio IRQ Flags
	d.WriteRegister(REG_IRQ_FLAGS, 0xff)
	// Mask all but TxDone
	d.WriteRegister(REG_IRQ_FLAGS, ^IRQ_TX_DONE_MASK)

	// initialize the payload size and address pointers
	d.WriteRegister(REG_FIFO_TX_BASE_ADDR, 0)
	d.WriteRegister(REG_FIFO_ADDR_PTR, 0)
	d.WriteRegister(REG_PAYLOAD_LENGTH, uint8(len(payload)))

	// Copy payload to FIFO
	for i := 0; i < len(payload); i++ {
		d.WriteRegister(REG_FIFO, payload[i])
	}
	// Enable TX
	d.SetOpMode(OPMODE_TX)
}

// GetRxPktChannel return the Go channel for Rx Packets
func (d *Device) GetRxPktChannel() chan []byte {
	return d.rxPktChan
}

//GetVersion returns hardware version of sx1276 chipset
func (d *Device) GetVersion() uint8 {
	return (d.ReadRegister(REG_VERSION))
}

// SendPacket transmits a packet
// Note that this will return before the packet has finished being sent,
// use the IsTransmitting() function if you need to know when sending is done.
func (d *Device) SendPacket(packet []byte) {

	// wait for any previous SendPacket to be done
	for d.IsTransmitting() {
		time.Sleep(1 * time.Millisecond)
	}

	// reset TX_DONE, FIFO address and payload length
	d.WriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK)
	d.WriteRegister(REG_FIFO_ADDR_PTR, 0)
	d.WriteRegister(REG_PAYLOAD_LENGTH, 0)

	if uint8(len(packet)) > MAX_PKT_LENGTH {
		packet = packet[0:MAX_PKT_LENGTH]
	}

	for i := 0; i < len(packet); i++ {
		d.WriteRegister(REG_FIFO, packet[i])
	}

	d.WriteRegister(REG_PAYLOAD_LENGTH, uint8(len(packet)))
	d.SetOpMode(OPMODE_LORA | OPMODE_TX)
}

// IsTransmitting tests if a packet transmission is in progress
func (d *Device) IsTransmitting() bool {
	return (d.ReadRegister(REG_OP_MODE) & OPMODE_TX) == OPMODE_TX
}

// ParsePacket returns the size of a received packet waiting to be read
// if size=0, this is explicit header mode (payload size is specified in packet header )
// if size>0, this is implicit header mode (payload size is not in packet header and specified by user)
// Refs: https://github.com/adafruit/RadioHead/blob/master/RH_RF95.cpp
func (d *Device) DioIntHandler() {

	// Read the interrupt register
	irqFlags := d.ReadRegister(REG_IRQ_FLAGS)
	//println("DioIntHandler: irqStatus=", irqFlags)

	if (irqFlags & IRQ_RX_DONE_MASK) == IRQ_RX_DONE_MASK {
		//	println("DioIntHandler: RxDoneMask Found")

		d.rxPktBuf = d.rxPktBuf[:0] // Clear slice

		// Read packet size
		packetLength := d.ReadRegister(REG_RX_NB_BYTES)
		//	println("DioIntHandler: RX Packet size: ", packetLength)

		// Reset the fifo read ptr to the beginning of the packet
		d.WriteRegister(REG_FIFO_ADDR_PTR, d.ReadRegister(REG_FIFO_RX_CURRENT_ADDR))

		for i := uint8(0); i < packetLength; i++ {
			d.rxPktBuf = append(d.rxPktBuf, d.ReadRegister(REG_FIFO))
		}
		//	println("DioIntHAndler: rx packet: ", string(d.rxPktBuf))
		d.rxPktChan <- d.rxPktBuf
		//	println("DioIntHAndler: xxx")

		// clear IRQ's
		d.WriteRegister(REG_IRQ_FLAGS, irqFlags)

	}

	// Sigh: on some processors, for some unknown reason, doing this only once does not actually
	// clear the radio's interrupt flag. So we do it twice. Why?
	d.WriteRegister(REG_IRQ_FLAGS, 0xff) // Clear all IRQ flags
	d.WriteRegister(REG_IRQ_FLAGS, 0xff) // Clear all IRQ flags

}

// ReadPacket reads a received packet into a byte array
func (d *Device) ReadPacket(packet []byte) int {
	available := int(d.ReadRegister(REG_RX_NB_BYTES) - d.packetIndex)
	if available > len(packet) {
		available = len(packet)
	}

	for i := 0; i < available; i++ {
		d.packetIndex++
		packet[i] = d.ReadRegister(REG_FIFO)
	}

	return available
}

// LastPacketRSSI gives the RSSI of the last packet received
// TO BE CHECKED
func (d *Device) LastPacketRSSI() uint8 {
	// section 5.5.5
	var adjustValue uint8 = 157
	if d.GetFrequency() < 868000000 {
		adjustValue = 164
	}
	return d.ReadRegister(REG_PKT_RSSI_VALUE) - adjustValue
}

// LastPacketSNR gives the SNR of the last packet received
func (d *Device) LastPacketSNR() uint8 {
	return uint8(d.ReadRegister(REG_PKT_SNR_VALUE) / 4)
}

// LastPacketFrequencyError gives the frequency error of the last packet received
// You can use this to adjust this transeiver frequency to more closly match the
// frequency being used by the sender, as this can drift over time
func (d *Device) LastPacketFrequencyError() int32 {
	// TODO
	// int32_t freqError = 0;
	// freqError = static_cast<int32_t>(ReadRegister(REG_FREQ_ERROR_MSB) & B111);
	// freqError <<= 8L;
	// freqError += static_cast<int32_t>(ReadRegister(REG_FREQ_ERROR_MID));
	// freqError <<= 8L;
	// freqError += static_cast<int32_t>(ReadRegister(REG_FREQ_ERROR_LSB));
	//
	// if (ReadRegister(REG_FREQ_ERROR_MSB) & B1000) { // Sign bit is on
	//    freqError -= 524288; // B1000'0000'0000'0000'0000
	// }
	//
	// const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
	// const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37
	//
	// return static_cast<long>(fError);
	return 0
}

func (d *Device) uint8ToString(v uint8) string {

	c1 := "00"
	return c1
}

// PrintRegisters outputs the sx127x transceiver registers
func (d *Device) PrintRegisters(compact bool) {
	for i := 0; i < 128; i++ {
		if compact {
			if (i % 0x10) == 0 {
				println("\n")
			}
			print(d.uint8ToString(uint8(i)), ":", d.uint8ToString(d.ReadRegister(uint8(i))))

		} else {
			println(d.uint8ToString(uint8(i)), ":", d.uint8ToString(d.ReadRegister(uint8(i))))
		}
	}
	println()
}

// Reset the sx127x device
func (d *Device) Reset() {
	d.rstPin.Low()
	time.Sleep(10 * time.Millisecond)
	d.rstPin.High()
	time.Sleep(10 * time.Millisecond)
}

// SetOpMode changes sx127x modes (RX/TX/IDLE...), but not LORA/FSK
func (d *Device) SetOpMode(mode uint8) {
	r := d.ReadRegister(REG_OP_MODE)
	d.WriteRegister(REG_OP_MODE, (r&^OPMODE_MASK)|mode)
}

// OpModeLora switch radio to Lora mode
func (d *Device) OpModeLora() {
	r := OPMODE_LORA
	r |= 0x8 // High frequency
	d.WriteRegister(REG_OP_MODE, r)
}

// OpModeFSK switch radio to FSK mode
func (d *Device) OpModeFSK() {
	r := uint8(0)
	r |= 0x8 // High frequency
	d.WriteRegister(REG_OP_MODE, r)
}

// Set Mode Receive Continuous
func (d *Device) ReceiveContinuous() {
	d.SetOpMode(OPMODE_LORA | OPMODE_RX)
	d.WriteRegister(REG_DIO_MAPPING_1, 0) // int on rxdone
}

// Set Mode Receive Single
func (d *Device) ReceiveSingle() {
	//Switch DIO0 to RxDone
	d.WriteRegister(REG_DIO_MAPPING_1, 0x00)
	// Clear all radio IRQ Flags
	d.WriteRegister(REG_IRQ_FLAGS, 0xff)
	// Mask all but TxDone
	d.WriteRegister(REG_IRQ_FLAGS, ^(IRQ_RX_DONE_MASK | IRQ_RX_TMOUT_MASK))
	// Rx Mode
	d.SetOpMode(OPMODE_LORA | OPMODE_RX_SINGLE)
}

// Standby puts the sx127x device into lora + standby mode
func (d *Device) Standby() {
	d.SetOpMode(OPMODE_LORA | OPMODE_STANDBY)
}

// Sleep puts the sx127x device into sleep mode
func (d *Device) Sleep() {
	d.SetOpMode(OPMODE_LORA | OPMODE_SLEEP)
}

// GetFrequency returns the frequency the LoRa module is using
func (d *Device) GetFrequency() uint32 {
	f := uint64(d.ReadRegister(REG_FRF_LSB))
	f += uint64(d.ReadRegister(REG_FRF_MID)) << 8
	f += uint64(d.ReadRegister(REG_FRF_MSB)) << 16
	f = (f * 32000000) >> 19 //FSTEP = FXOSC/2^19
	return uint32(f)
}

// SetFrequency updates the frequency the LoRa module is using
func (d *Device) ConfigureChannel(frequency uint32) {
	var frf = (uint64(frequency) << 19) / 32000000
	d.WriteRegister(REG_FRF_MSB, uint8(frf>>16))
	d.WriteRegister(REG_FRF_MID, uint8(frf>>8))
	d.WriteRegister(REG_FRF_LSB, uint8(frf>>0))
}

// GetSpreadingFactor returns the spreading factor the LoRa module is using
func (d *Device) GetSpreadingFactor() uint8 {
	return d.ReadRegister(REG_MODEM_CONFIG_2) >> 4
}

// GetRSSI returns current RSSI
func (d *Device) GetRSSI() uint8 {
	return d.ReadRegister(REG_RSSI_VALUE)
}

// GetBandwidth returns the bandwidth the LoRa module is using
func (d *Device) GetBandwidth() int32 {
	var bw = d.ReadRegister(REG_MODEM_CONFIG_1) >> 4

	switch bw {
	case 0:
		return 7800
	case 1:
		return 10400
	case 2:
		return 15600
	case 3:
		return 20800
	case 4:
		return 31250
	case 5:
		return 41700
	case 6:
		return 62500
	case 7:
		return 125000
	case 8:
		return 250000
	case 9:
		return 500000
	}

	return -1
}

//SetSyncWord defines sync word
func (d *Device) SetSyncWord(syncWord uint8) {
	d.WriteRegister(REG_SYNC_WORD, syncWord)
}

//SetInvertIQ defines Invert IQ on RX and TX
/*
func (d *Device) SetInvertIQ(invertRx, invertTx bool) {
	reg := d.ReadRegister(REG_INVERTIQ)
	if invertRx {
		reg = reg | 0x40
	}
	if invertTx {
		reg = reg | 0x01
	}
	d.WriteRegister(REG_INVERTIQ, reg)
}
*/

func (d *Device) SetInvertedIQ(invert bool) {
	if invert {
		//Invert IQ Back
		d.WriteRegister(0x33, 0x67)
		d.WriteRegister(0x3B, 0x19)
	} else {
		//Set IQ to normal values
		d.WriteRegister(0x33, 0x27)
		d.WriteRegister(0x3B, 0x1D)
	}
}

func (d *Device) GoSingleReceive() {
	// ReceiveSingle
	d.WriteRegister(REG_DIO_MAPPING_1, 0)       //Change DIO 0 back to RxDone
	d.WriteRegister(0x33, 0x67)                 // Set IQ
	d.WriteRegister(0x3B, 0x19)                 // Set IQ
	d.SetOpMode(OPMODE_LORA | OPMODE_RX_SINGLE) // Single RX Mode
}

/*
// setLdoFlag() enables LowDataRateOptimize bit (mandated when symbol length >16ms)
// LGTM
func (d *Device) setLdoFlag() {
	// Section 4.1.1.5
	var symbolDuration = 1000 / (d.GetBandwidth() / (1 << d.GetSpreadingFactor()))

	var config3 = d.ReadRegister(REG_MODEM_CONFIG_3)

	// Section 4.1.1.6
	if symbolDuration > 16 {
		config3 = config3 | 0x08
	} else {
		config3 = config3 & 0xF7
	}

	d.WriteRegister(REG_MODEM_CONFIG_3, config3)
}
*/

// SetTxPower sets the transmitter output power
func (d *Device) SetTxPower(txPower int8, paBoost bool) {
	if !paBoost {
		// RFO
		if txPower < 0 {
			txPower = 0
		} else if txPower > 14 {
			txPower = 14
		}
		d.WriteRegister(REG_PA_CONFIG, uint8(0x70)|uint8(txPower))

	} else {
		//PA_BOOST
		if txPower > 17 {
			if txPower > 20 {
				txPower = 20
			}

			txPower -= 3

			// High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
			d.WriteRegister(REG_PA_DAC, 0x87)
			d.SetOCP(140)
		} else {
			if txPower < 2 {
				txPower = 2
			}

			d.WriteRegister(REG_PA_DAC, 0x84)
			d.SetOCP(100)

		}

		d.WriteRegister(REG_PA_CONFIG, uint8(PA_BOOST)|uint8(txPower-2))

	}
}

// SetOCP defines Overload Current Protection configuration
func (d *Device) SetOCP(mA uint8) {

	ocpTrim := uint8(27)

	if mA < 45 {
		mA = 45
	}
	if mA <= 120 {
		ocpTrim = (mA - 45) / 5
	} else if mA <= 240 {
		ocpTrim = (mA + 30) / 10
	}

	d.WriteRegister(REG_OCP, 0x20|(0x1F&ocpTrim))
}

// ---------------
// RegModemConfig1
// ---------------

// SetBandwidth updates the bandwidth the LoRa module is using
func (d *Device) SetBandwidth(sbw int32) {
	var bw uint8

	if sbw <= 7800 {
		bw = 0
	} else if sbw <= 10400 {
		bw = 1
	} else if sbw <= 15600 {
		bw = 2
	} else if sbw <= 20800 {
		bw = 3
	} else if sbw <= 31250 {
		bw = 4
	} else if sbw <= 41700 {
		bw = 5
	} else if sbw <= 62500 {
		bw = 6
	} else if sbw <= 125000 {
		bw = 7
	} else if sbw <= 250000 {
		bw = 8
	} else {
		bw = 9
	}
	d.WriteRegister(REG_MODEM_CONFIG_1, (d.ReadRegister(REG_MODEM_CONFIG_1)&0x0f)|(bw<<4))
	d.cnf.Bandwidth = sbw
}

// SetCodingRate updates the coding rate the LoRa module is using
func (d *Device) SetCodingRate(denominator uint8) {
	if denominator < 5 {
		denominator = 5
	} else if denominator > 8 {
		denominator = 8
	}
	var cr = denominator - 4
	d.WriteRegister(REG_MODEM_CONFIG_1, (d.ReadRegister(REG_MODEM_CONFIG_1)&0xf1)|(cr<<1))
	d.cnf.CodingRate = denominator
}

// SetImplicitHeaderModeOn Enables implicit header mode
func (d *Device) SetImplicitHeaderModeOn(val bool) {
	if val {
		d.WriteRegister(REG_MODEM_CONFIG_1, d.ReadRegister(REG_MODEM_CONFIG_1)|0x01)
	} else {
		d.WriteRegister(REG_MODEM_CONFIG_1, d.ReadRegister(REG_MODEM_CONFIG_1)&0xfe)
	}
	d.cnf.ImplicitHeaderModeOn = val
}

// ---------------
// RegModemConfig2
// ---------------

// SetSpreadingFactor updates the spreading factor the LoRa module is using
func (d *Device) SetSpreadingFactor(spreadingFactor uint8) {
	if spreadingFactor < 6 {
		spreadingFactor = 6
	} else if spreadingFactor > 12 {
		spreadingFactor = 12
	}

	if spreadingFactor == 6 {
		d.WriteRegister(REG_DETECTION_OPTIMIZE, 0xc5)
		d.WriteRegister(REG_DETECTION_THRESHOLD, 0x0c)
	} else {
		d.WriteRegister(REG_DETECTION_OPTIMIZE, 0xc3)
		d.WriteRegister(REG_DETECTION_THRESHOLD, 0x0a)
	}

	var newValue = (d.ReadRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((spreadingFactor << 4) & 0xf0)
	d.WriteRegister(REG_MODEM_CONFIG_2, newValue)
	d.cnf.SpreadingFactor = spreadingFactor
}

// SetTxContinuousMode enable Continuous Tx mode
func (d *Device) SetTxContinuousMode(val bool) {
	if val {
		d.WriteRegister(REG_MODEM_CONFIG_2, d.ReadRegister(REG_MODEM_CONFIG_2)|0x08)
	} else {
		d.WriteRegister(REG_MODEM_CONFIG_2, d.ReadRegister(REG_MODEM_CONFIG_2)&0xf7)
	}
}

// SetRxPayloadCrcOn Enable CRC generation and check on payload
func (d *Device) SetRxPayloadCrcOn(val bool) {
	if val {
		d.WriteRegister(REG_MODEM_CONFIG_2, d.ReadRegister(REG_MODEM_CONFIG_2)|0x04)
	} else {
		d.WriteRegister(REG_MODEM_CONFIG_2, d.ReadRegister(REG_MODEM_CONFIG_2)&0xfb)
	}
}

// ---------------
// RegModemConfig3
// ---------------

// SetAgcAutoOn enables Automatic Gain Control
func (d *Device) SetAgcAutoOn(val bool) {
	if val {
		d.WriteRegister(REG_MODEM_CONFIG_3, d.ReadRegister(REG_MODEM_CONFIG_3)|0x04)
	} else {
		d.WriteRegister(REG_MODEM_CONFIG_3, d.ReadRegister(REG_MODEM_CONFIG_3)&0xfb)
	}
}

// SetLowDataRateOptimize enables Low Data Rate Optimization
func (d *Device) SetLowDataRateOptimOn(val bool) {
	if val {
		d.WriteRegister(REG_MODEM_CONFIG_3, d.ReadRegister(REG_MODEM_CONFIG_3)|0x08)
	} else {
		d.WriteRegister(REG_MODEM_CONFIG_3, d.ReadRegister(REG_MODEM_CONFIG_3)&0xf7)
	}
}

// -------------------
// Read/Write SPI Regs
// -------------------

// ReadRegister returns register value
func (d *Device) ReadRegister(reg uint8) uint8 {
	d.csPin.Low()
	d.spi.Tx([]byte{reg & 0x7f}, nil)
	var value [1]byte
	d.spi.Tx(nil, value[:])
	d.csPin.High()
	return value[0]
}

// WriteRegister sets a value to register
func (d *Device) WriteRegister(reg uint8, value uint8) uint8 {
	var response [1]byte
	d.csPin.Low()
	d.spi.Tx([]byte{reg | 0x80}, nil)
	d.spi.Tx([]byte{value}, response[:])
	d.csPin.High()
	return response[0]
}
