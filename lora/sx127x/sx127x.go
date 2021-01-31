// Package sx127x provides a driver for SX127x LoRa transceivers.
// https://electronics.stackexchange.com/questions/394296/can-t-get-simple-lora-receiver-to-work
package sx127x

import (
	"errors"
	"machine"
	"time"
)

const (
	debug = true
)

// This should be passed to the DioIntHandler
// So we can keep track of the origin of interruption
const (
	IntDIO0 = 0
	IntDIO1 = 1
)

// RadioEvent are send to application through a channel
type RadioEvent struct {
	EventType int
	EventData []byte
}

const (
	EventRxDone    = iota
	EventTxDone    = iota
	EventRxTimeout = iota
)

// Device wraps an SPI connection to a SX127x device.
type Device struct {
	spi                machine.SPI
	csPin              machine.Pin
	rstPin             machine.Pin
	packetIndex        uint8
	implicitHeaderMode bool
	radioEventChan     chan RadioEvent
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

// New creates a new SX127x connection. The SPI bus must already be configured.
func New(spi machine.SPI, csPin machine.Pin, rstPin machine.Pin) Device {
	k := Device{
		spi:    spi,
		csPin:  csPin,
		rstPin: rstPin,
	}
	return k
}

// Reset re-initialize the sx127x device
// sx1276 have nRST pin
func (d *Device) Reset() {
	d.rstPin.Low()
	time.Sleep(100 * time.Millisecond)
	d.rstPin.High()
	time.Sleep(100 * time.Millisecond)
}

// ReadRegister reads register value
func (d *Device) ReadRegister(reg uint8) uint8 {
	d.csPin.Low()
	d.spi.Tx([]byte{reg & 0x7f}, nil)
	var value [1]byte
	d.spi.Tx(nil, value[:])
	d.csPin.High()
	return value[0]
}

// WriteRegister writes value to register
func (d *Device) WriteRegister(reg uint8, value uint8) uint8 {
	var response [1]byte
	d.csPin.Low()
	d.spi.Tx([]byte{reg | 0x80}, nil)
	d.spi.Tx([]byte{value}, response[:])
	d.csPin.High()
	return response[0]
}

// SetOpMode changes the sx1276 mode
func (d *Device) OpMode(mode uint8) {
	cur := d.ReadRegister(REG_OP_MODE)
	new := (cur & (^OPMODE_MASK)) | mode
	d.WriteRegister(REG_OP_MODE, new)
	if debug {
		chk := d.ReadRegister(REG_OP_MODE)
		println("sx1276 : SetOpMode cur:", cur, " new:", new, "chk:", chk)
	}
}

// SetOpMode changes the sx1276 mode
func (d *Device) OpModeLora() {
	d.WriteRegister(REG_OP_MODE, OPMODE_LORA)
}

// SetupLora configures sx127x Lora mode
func (d *Device) SetupLora(loraCnf Config) error {

	d.cnf = loraCnf
	println("Setup Lora:")
	println("Freq: ", d.cnf.Frequency, "CR: ", d.cnf.CodingRate, "SF: ", d.cnf.SpreadingFactor)

	// Reset the device first
	d.Reset()
	if d.GetVersion() != 0x12 {
		return errors.New("SX1276 module not found")
	}

	// Switch to Lora mode
	d.OpModeLora()
	d.OpMode(OPMODE_SLEEP)

	// Access High Frequency Mode
	d.SetLowFrequencyModeOn(false)

	// Set PA Ramp time 50 uS
	d.WriteRegister(REG_PA_RAMP, (d.ReadRegister(REG_PA_RAMP)&0xF0)|0x08) // set PA ramp-up time 50 uSec

	// Enable power (manage Over Current, PA_Boost ... etc)
	d.SetTxPower(11, true)

	// Set Low Noise Amplifier to MAX
	d.WriteRegister(REG_LNA, LNA_MAX_GAIN)

	// Set Frequency
	d.SetFrequency(d.cnf.Frequency)

	// Set DataRate
	d.SetBandwidth(d.cnf.Bandwidth)

	//Set Coding Rate (TODO : Check)
	d.SetCodingRate(d.cnf.CodingRate)

	// Set implicit header
	d.SetImplicitHeaderModeOn(false)

	// Enable CRC
	d.SetRxPayloadCrcOn(true)

	// Disable IQ Polarization
	d.SetInvertedIQ(false)

	// Disable HOP PERIOD
	d.SetHopPeriod(0x00)

	// Continuous Mode
	d.SetTxContinuousMode(false)

	// Set Lora Sync
	d.SetSyncWord(0x34)

	//Set Max payload length (default value)
	d.WriteRegister(REG_MAX_PAYLOAD_LENGTH, 0xFF)
	// Mandatory in Implicit header Mode (default value)
	d.WriteRegister(REG_PAYLOAD_LENGTH, 0x01)

	// AGC On
	d.SetAgcAutoOn(true)

	// set FIFO base addresses
	d.WriteRegister(REG_FIFO_TX_BASE_ADDR, 0)
	d.WriteRegister(REG_FIFO_RX_BASE_ADDR, 0)
	return nil
}

// TxLora sends a packet in Lora mode
func (d *Device) TxLora(payload []byte) {

	// Are we already in Lora mode ?
	r := d.ReadRegister(REG_OP_MODE)
	if (r & OPMODE_LORA) != OPMODE_LORA {
		println("FATAL: module is not in LORA MODE")
		return
	}

	// set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
	d.WriteRegister(REG_DIO_MAPPING_1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP)
	// Clear all radio IRQ Flags
	d.WriteRegister(REG_IRQ_FLAGS, 0xFF)
	// Mask all but TxDone
	d.WriteRegister(REG_IRQ_FLAGS_MASK, ^IRQ_LORA_TXDONE_MASK)

	// initialize the payload size and address pointers
	d.WriteRegister(REG_FIFO_TX_BASE_ADDR, 0)
	d.WriteRegister(REG_FIFO_ADDR_PTR, 0)
	d.WriteRegister(REG_PAYLOAD_LENGTH, uint8(len(payload)))

	// Copy payload to FIFO // TODO: Bulk
	for i := 0; i < len(payload); i++ {
		d.WriteRegister(REG_FIFO, payload[i])
	}
	// Enable TX
	//d.PrintRegisters(true)
	println("sx127x : TxLora: swith to TX mode")
	d.OpMode(OPMODE_TX)
}

//DioIntHandler should be called on DIO0/1 rising edge
//intSource should be set to IntDIO0 or IntDIO1 consts
//in case we need to track the origin of the interrupt
func (d *Device) DioIntHandler(intSource int) {

	irqFlags := d.ReadRegister(REG_IRQ_FLAGS)

	var event *RadioEvent = nil

	// We have a packet
	if (irqFlags & IRQ_LORA_RXDONE_MASK) > 0 {

		// Read current packet
		buf := []byte{}
		packetLength := d.ReadRegister(REG_RX_NB_BYTES)
		d.WriteRegister(REG_FIFO_ADDR_PTR, d.ReadRegister(REG_FIFO_RX_CURRENT_ADDR)) // Reset FIFO Read Addr
		for i := uint8(0); i < packetLength; i++ {
			buf = append(buf, d.ReadRegister(REG_FIFO))
		}
		// Send RXDONE to the defined event channel
		event = &RadioEvent{EventType: EventTxDone, EventData: buf}
	} else if (irqFlags & IRQ_LORA_TXDONE_MASK) > 0 {
		println("sx1276: TX_DONE int")
		event = &RadioEvent{EventType: EventTxDone, EventData: nil}
	} else if (irqFlags & IRQ_LORA_RXTOUT_MASK) > 0 {
		println("sx1276: RX_TMOUT int")
		event = &RadioEvent{EventType: EventRxTimeout, EventData: nil}
	} else {
		println("sx1276: irqflags:", irqFlags)
	}

	// Sent the event, if we have one
	if event != nil && d.radioEventChan != nil {
		d.radioEventChan <- *event
	}

	// clear IRQ's
	d.WriteRegister(REG_IRQ_FLAGS, irqFlags)

	// Sigh: on some processors, for some unknown reason, doing this only once does not actually
	// clear the radio's interrupt flag. So we do it twice. Why?
	// d.WriteRegister(REG_IRQ_FLAGS, 0xff) // Clear all IRQ flags
	// d.WriteRegister(REG_IRQ_FLAGS, 0xff) // Clear all IRQ flags

}

//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------

// SetRadioEventChan defines a channel so the driver can send its Radio Events
func (d *Device) SetRadioEventChan(channel chan RadioEvent) {
	d.radioEventChan = channel
}

// GetRadioEventChannel returns the current Radio Event channel
func (d *Device) GetRadioEventChan() chan RadioEvent {
	return d.radioEventChan
}

// Init reboots the SX1276 module and tries to read HW version
func (d *Device) Init(cfg Config) (err error) {
	d.cnf = cfg

	d.csPin.High()
	d.Reset()

	return nil
}

// ConfigureLoraModem prepares for LORA communications
func (d *Device) ConfigureLoraModem() {

	// Sleep mode required to go LOra
	d.OpMode(OPMODE_SLEEP)
	// Set Lora mode (from sleep)
	d.OpModeLora()
	// Switch to standby mode
	d.OpMode(OPMODE_STANDBY)
	// Set Bandwidth
	d.SetBandwidth(d.cnf.Bandwidth)
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

//GetVersion returns hardware version of sx1276 chipset
func (d *Device) GetVersion() uint8 {
	return (d.ReadRegister(REG_VERSION))
}

// IsTransmitting tests if a packet transmission is in progress
func (d *Device) IsTransmitting() bool {
	return (d.ReadRegister(REG_OP_MODE) & OPMODE_TX) == OPMODE_TX
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

func (d *Device) uint8ToString(v uint8) string {

	c1 := "00"
	return c1
}

// PrintRegisters outputs the sx127x transceiver registers
func (d *Device) PrintRegisters(compact bool) {
	for i := uint8(0); i < 128; i++ {
		v := d.ReadRegister(i)
		print(v, " ")
	}
	println()
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
func (d *Device) SetFrequency(frequency uint32) {
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

func (d *Device) GoReceive() {
	// ReceiveSingle
	d.WriteRegister(REG_DIO_MAPPING_1, 0) //Change DIO0 back to RxDone, DIO1 RxTmeout
	d.WriteRegister(0x33, 0x67)           // Set IQ
	d.WriteRegister(0x3B, 0x19)           // Set IQ
	d.OpMode(OPMODE_LORA | OPMODE_RX)     // RX Mode
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

// SetLowFrequencyModeOn enables Low Data Rate Optimization
func (d *Device) SetLowFrequencyModeOn(val bool) {
	if val {
		d.WriteRegister(REG_OP_MODE, d.ReadRegister(REG_OP_MODE)|0x04)
	} else {
		d.WriteRegister(REG_OP_MODE, d.ReadRegister(REG_OP_MODE)&0xfb)
	}
}

// SetHopPeriod sets number of symbol periods between frequency hops. (0 = disabled).
func (d *Device) SetHopPeriod(val uint8) {
	d.WriteRegister(REG_HOP_PERIOD, val)
}
