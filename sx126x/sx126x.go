// Package sx126x provides a driver for SX126x LoRa transceivers.
// Inspired from https://github.com/Lora-net/sx126x_driver/

package sx126x

import (
	"encoding/hex"
	"errors"
	"fmt"
	"math"
	"strings"
	"time"

	"machine"

	"tinygo.org/x/drivers"
	"tinygo.org/x/drivers/lora"
)

var (
	errWaitWhileBusyTimeout   = errors.New("WaitWhileBusy Timeout")
	errLowPowerTxNotSupported = errors.New("RFSWITCH_TX_LP not supported")
	errRadioNotFound          = errors.New("LoRa radio not found")
	errUnexpectedRxRadioEvent = errors.New("Unexpected Radio Event during RX")
	errUnexpectedTxRadioEvent = errors.New("Unexpected Radio Event during TX")
	errInvalidRxBandwidth     = errors.New("Invalid Rx Bandwidth")
	errInvalidCRCConfig       = errors.New("Invalid CRC Configuration")
	errInvalidFreqDev         = errors.New("Invalid Frequency Deviation")
	errInvalidBaudrate        = errors.New("Invalid baudrate")
	errNoControler            = errors.New("No controler attached to radio")
	errWrongOperation         = errors.New("Wrong packet type operation")
)

const (
	debug = true
)

const (
	DEVICE_TYPE_SX1261 = iota
	DEVICE_TYPE_SX1262 = iota
	DEVICE_TYPE_SX1268 = iota
)

const (
	RFSWITCH_RX    = iota
	RFSWITCH_TX_LP = iota
	RFSWITCH_TX_HP = iota
)

const (
	PERIOD_PER_SEC      = (uint32)(1000000 / 15.625) // SX1261 DS 13.1.4
	SPI_BUFFER_SIZE     = 256
	RADIOEVENTCHAN_SIZE = 1
)

// Device wraps an SPI connection to a SX126x device.
type Device struct {
	spi            drivers.SPI          // SPI bus for module communication
	rstPin         machine.Pin          // GPIO for reset pin
	radioEventChan chan lora.RadioEvent // Channel for Receiving events
	loraConf       lora.Config          // Current Lora configuration
	controller     RadioController      // to manage interactions with the radio
	deepSleep      bool                 // Internal Sleep state
	deviceType     int                  // sx1261,sx1262,sx1268 (defaults sx1261)
	spiTxBuf       []byte               // global Tx buffer to avoid heap allocations in interrupt
	spiRxBuf       []byte               // global Rx buffer to avoid heap allocations in interrupt

	frequencyDeviation uint32 // Fréquency deviation (after register convertion)
	frequencyKhz       uint32 // Fréquency deviation (after register convertion)
	preambleTypeLora   uint16 // Preamble length while in Lora
	txPower            int8   // TX Power in dB

	// Internal configuration .. Not externaly usable (most are raw register values)
	rxBandwidthKhz    float64
	preambleLengthFsk uint16 // Preamble length while in FSK
	bitRate           uint32
	frequencyDev      uint32
	rxBandwidth       uint8
	pulseShape        uint8
	crcTypeFSK        uint8
	preambleLengthFSK uint16
	addrComp          uint32
	syncWordLength    uint8
	whitening         uint8
	packetType        uint8 // Variable ou fixed
	packetTypeLen     uint8 // length of packet (depends variable/fixed)
	preambleDetectLen uint8
}

// New creates a new SX126x connection.
func New(spi drivers.SPI) *Device {
	return &Device{
		spi:            spi,
		radioEventChan: make(chan lora.RadioEvent, RADIOEVENTCHAN_SIZE),
		spiTxBuf:       make([]byte, SPI_BUFFER_SIZE),
		spiRxBuf:       make([]byte, SPI_BUFFER_SIZE),
	}
}

const (
	SX126X_RTC_FREQ_IN_HZ uint32 = 64000
)

// --------------------------------------------------
//  Helper functions
// --------------------------------------------------

// timeoutMsToRtcSteps converts Timeout (in ms) to RTC Steps
func timeoutMsToRtcSteps(timeoutMs uint32) uint32 {
	r := uint32(timeoutMs * (SX126X_RTC_FREQ_IN_HZ / 1000))
	return r
}

// --------------------------------------------------
//
//	Channel and events
//
// --------------------------------------------------
// GetRadioEventChan() Returns RadioEvent channel of the device
func (d *Device) GetRadioEventChan() chan lora.RadioEvent {
	return d.radioEventChan
}

// SetDeviceType sets the variant of radio module (SX1261/2/8)
func (d *Device) SetDeviceType(devType int) {
	d.deviceType = devType
}

// SetRadioControl associates RadioController to the driver
func (d *Device) SetRadioController(rc RadioController) error {
	d.controller = rc
	if err := d.controller.Init(); err != nil {
		return err
	}
	d.controller.SetupInterrupts(d.HandleInterrupt)

	return nil
}

// --------------------------------------------------
// Operational modes functions
// --------------------------------------------------

// Reset() reinitialize the device through its RESET pin
func (d *Device) Reset() {
	d.rstPin.Low()
	time.Sleep(100 * time.Millisecond)
	d.rstPin.High()
	time.Sleep(100 * time.Millisecond)
}

// DetectDevice() tries to detect the radio module
// it returns true if device is found
func (d *Device) DetectDevice() bool {
	dat, err := d.ReadRegister(SX126X_REG_VERSION_STRING, 16)
	if err == nil {
		ver := string(dat)
		println("SX126X ID:", ver)
		if strings.Contains(ver, SX1261_CHIP_TYPE_STRING_ID) {
			d.SetDeviceType(DEVICE_TYPE_SX1262)
			return true
		}
	}
	return false
}

// SetSleep sets the device in SLEEP mode with the lowest current consumption possible.
func (d *Device) SetSleep() {
	d.ExecSetCommand(SX126X_CMD_SET_SLEEP, []uint8{SX126X_SLEEP_START_WARM | SX126X_SLEEP_RTC_OFF})
}

// SetStandby sets the device in a configuration mode which is at an intermediate level of consumption
func (d *Device) SetStandby() {
	d.ExecSetCommand(SX126X_CMD_SET_STANDBY, []uint8{SX126X_STANDBY_RC})
}

// SetFs sets the device in frequency synthesis mode where the PLL is locked to the carrier frequency.
func (d *Device) SetFs() error {
	d.ExecSetCommand(SX126X_CMD_SET_FS, []uint8{})
	return nil
}

// SetTxContinuousWave set device in test mode to generate a continuous wave (RF tone)
func (d *Device) SetTxContinuousWave() error {
	if d.controller == nil {
		return errNoControler
	}
	d.controller.SetRfSwitchMode(RFSWITCH_TX_HP)
	d.ExecSetCommand(SX126X_CMD_SET_TX_CONTINUOUS_WAVE, []uint8{})
	return nil
}

// SetTxContinuousPreamble set device in test mode to constantly modulate LoRa preamble symbols.
// Take care to initialize all Lora settings like it's done in Tx before calling this function
// If you don't init properly all the settings, it'll fail
func (d *Device) SetTxContinuousPreamble() error {
	if d.controller == nil {
		return errNoControler
	}
	d.controller.SetRfSwitchMode(RFSWITCH_TX_HP)
	d.ExecSetCommand(SX126X_CMD_SET_TX_INFINITE_PREAMBLE, []uint8{})
	return nil
}

// SetTx() sets the device in TX mode
// timeout is expressed in RTC Step unit (15uS)
// The device will stay in Tx until countdown or packet transmitted
// Value of 0x000000 will disable timer and device will stay TX
func (d *Device) SetTx(timeoutRtcStep uint32) {
	var p [3]uint8
	p[0] = uint8((timeoutRtcStep >> 16) & 0xFF)
	p[1] = uint8((timeoutRtcStep >> 8) & 0xFF)
	p[2] = uint8((timeoutRtcStep >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_TX, p[:])
}

// SetRx() sets the device in RX mode
// timeout is expressed in RTC Step unit (15uS)
// Value of 0x000000 => No timeout. Rx Single mode.
// Value of 0xffffff => Rx Continuous mode
// Other values => Timeout active. The device remains in RX until countdown or packet received
func (d *Device) SetRx(timeoutRtcStep uint32) {
	var p [3]uint8
	p[0] = uint8(((timeoutRtcStep >> 16) & 0xFF))
	p[1] = uint8(((timeoutRtcStep >> 8) & 0xFF))
	p[2] = uint8(((timeoutRtcStep >> 0) & 0xFF))
	d.ExecSetCommand(SX126X_CMD_SET_RX, p[:])
}

// StopTimerOnPreamble allows the user to select if the timer is stopped upon preamble detection of SyncWord / header detection.
func (d *Device) StopTimerOnPreamble(enable bool) {
	var p [1]uint8
	if enable {
		p[0] = 1
	} else {
		p[0] = 0
	}
	d.ExecSetCommand(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, p[:])
}

// SetRegulatorMode sets the regulator more (depends on hardware implementation)
func (d *Device) SetRegulatorMode(mode uint8) {
	p := []uint8{mode}
	d.ExecSetCommand(SX126X_CMD_SET_REGULATOR_MODE, p[:])
}

// Calibrate starts the calibration of a block defined by calibParam
func (d *Device) Calibrate(calibParam uint8) {
	p := []uint8{calibParam}
	d.ExecSetCommand(SX126X_CMD_CALIBRATE, p[:])
}

// CalibrateImage calibrates the image rejection of the device for the device operating
func (d *Device) CalibrateImage(freq uint32) {
	var calFreq [2]uint8
	if freq > 900000000 {
		calFreq[0] = 0xE1
		calFreq[1] = 0xE9
	} else if freq > 850000000 {
		calFreq[0] = 0xD7
		calFreq[1] = 0xD8
	} else if freq > 770000000 {
		calFreq[0] = 0xC1
		calFreq[1] = 0xC5
	} else if freq > 460000000 {
		calFreq[0] = 0x75
		calFreq[1] = 0x81
	} else if freq > 425000000 {
		calFreq[0] = 0x6B
		calFreq[1] = 0x6F
	}
	d.ExecSetCommand(SX126X_CMD_CALIBRATE_IMAGE, calFreq[:])
}

// SetPaConfig sets the Power Amplifier configuration
// deviceSel: 0 for SX1262, 1 for SX1261
func (d *Device) SetPaConfig(paDutyCycle, hpMax, deviceSel, paLut uint8) {
	var p [4]uint8
	p[0] = paDutyCycle
	p[1] = hpMax
	p[2] = deviceSel
	p[3] = paLut
	d.ExecSetCommand(SX126X_CMD_SET_PA_CONFIG, p[:])
}

// SetRxTxFallbackMode defines into which mode the chip goes after a successful transmission or after a packet reception.
func (d *Device) SetRxTxFallbackMode(fallbackMode uint8) {
	d.ExecSetCommand(SX126X_CMD_SET_RX_TX_FALLBACK_MODE, []uint8{fallbackMode})
}

// --------------------------------------------------
// Registers and Buffers
// --------------------------------------------------

// ReadRegister reads register value
func (d *Device) ReadRegister(addr, size uint16) ([]uint8, error) {
	d.CheckDeviceReady()
	d.controller.SetNss(false)
	// Send command
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, SX126X_CMD_READ_REGISTER, uint8((addr&0xFF00)>>8), uint8(addr&0x00FF), 0x00)
	d.spi.Tx(d.spiTxBuf, nil)
	// Read registers
	d.spiRxBuf = d.spiRxBuf[0:size]
	d.spi.Tx(nil, d.spiRxBuf)
	d.controller.SetNss(true)
	d.controller.WaitWhileBusy()
	return d.spiRxBuf, nil
}

// WriteRegister writes value to register
func (d *Device) WriteRegister(addr uint16, data []uint8) {
	d.CheckDeviceReady()
	d.controller.SetNss(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, SX126X_CMD_WRITE_REGISTER, uint8((addr&0xFF00)>>8), uint8(addr&0x00FF))
	d.spiTxBuf = append(d.spiTxBuf, data...)
	d.spi.Tx(d.spiTxBuf, nil)
	d.controller.SetNss(true)
	d.controller.WaitWhileBusy()
}

// WriteBuffer write data from current buffer position
func (d *Device) WriteBuffer(data []uint8) {
	p := []uint8{0}
	p = append(p, data...)
	d.ExecSetCommand(SX126X_CMD_WRITE_BUFFER, p)
}

// ReadBuffer Reads size bytes from current buffer position
func (d *Device) ReadBuffer(size uint8) []uint8 {
	ret := d.ExecGetCommand(SX126X_CMD_READ_BUFFER, size)
	return ret
}

// --------------------------------------------------
// DIO and IRQ
// --------------------------------------------------

// SetDioIrqParams configures DIO Irq
func (d *Device) SetDioIrqParams(irqMask, dio1Mask, dio2Mask, dio3Mask uint16) {
	var p [8]uint8
	p[0] = uint8((irqMask >> 8) & 0xFF)
	p[1] = uint8(irqMask & 0xFF)
	p[2] = uint8((dio1Mask >> 8) & 0xFF)
	p[3] = uint8(dio1Mask & 0xFF)
	p[4] = uint8((dio2Mask >> 8) & 0xFF)
	p[5] = uint8(dio2Mask & 0xFF)
	p[6] = uint8((dio3Mask >> 8) & 0xFF)
	p[7] = uint8(dio3Mask & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, p[:])
}

// GetIrqStatus returns IRQ status
func (d *Device) GetIrqStatus() (irqStatus uint16) {
	r := d.ExecGetCommand(SX126X_CMD_GET_IRQ_STATUS, 2)
	ret := (uint16(r[0]) << 8) | uint16(r[1])
	return ret
}

// ClearIrqStatus clears IRQ flags
func (d *Device) ClearIrqStatus(clearIrqParams uint16) {
	var p [2]uint8
	p[0] = uint8((clearIrqParams >> 8) & 0xFF)
	p[1] = uint8(clearIrqParams & 0xFF)
	d.ExecSetCommand(SX126X_CMD_CLEAR_IRQ_STATUS, p[:])
}

// --------------------------------------------------
// Communication Status Information
// --------------------------------------------------

// GetStatus returns radio status(13.5.1)
func (d *Device) GetStatus() (radioStatus uint8) {
	r := d.ExecGetCommand(SX126X_CMD_GET_STATUS, 1)
	return r[0]
}

// GetRxBufferStatus returns the length of the last received packet (PayloadLengthRx)
// and the address of the first byte received (RxStartBufferPointer). (13.5.2)
func (d *Device) GetRxBufferStatus() (payloadLengthRx uint8, rxStartBufferPointer uint8) {
	r := d.ExecGetCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, 2)
	return r[0], r[1]
}

// GetPackeType returns current Packet Type (13.4.3)
func (d *Device) GetPacketType() (packetType uint8) {
	r := d.ExecGetCommand(SX126X_CMD_GET_PACKET_TYPE, 1)
	return r[0]
}

// GetDeviceErrors returns current Device Errors
func (d *Device) GetDeviceErrors() uint16 {
	r := d.ExecGetCommand(SX126X_CMD_GET_DEVICE_ERRORS, 2)
	ret := uint16(r[0]<<8 + r[1])
	return ret
}

// ClearDeviceErrors clears device Errors
func (d *Device) ClearDeviceErrors() {
	p := [2]uint8{0x00, 0x00}
	d.ExecSetCommand(SX126X_CMD_CLEAR_DEVICE_ERRORS, p[:])
}

// GetStats returns the number of informations received on a few last packets
// Lora: NbPktReceived, NbPktCrcError, NbPktHeaderErr
func (d *Device) GetLoraStats() (nbPktReceived, nbPktCrcError, nbPktHeaderErr uint16) {
	r := d.ExecGetCommand(SX126X_CMD_GET_STATS, 6)
	return uint16(r[0]<<8 | r[1]), uint16(r[2]<<8 | r[3]), uint16(r[4]<<8 | r[5])
}

// ---------------------------------------
// PACKET / RADIO / PROTOCOL CONFIGURATION
// ---------------------------------------

// SetPacketType sets the packet type
func (d *Device) SetPacketType(packetType uint8) {
	var p [1]uint8
	p[0] = packetType
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_TYPE, p[:])
}

// SetLoraSyncWord defines the Sync Word to use in LORA mode
func (d *Device) SetLoraSyncWord(sw uint16) {
	var p [2]uint8
	d.loraConf.SyncWord = sw
	p[0] = uint8((d.loraConf.SyncWord >> 8) & 0xFF)
	p[1] = uint8((d.loraConf.SyncWord >> 0) & 0xFF)
	d.WriteRegister(SX126X_REG_LORA_SYNC_WORD_MSB, p[:])
}

// GetSyncWord gets the Sync Word to use
func (d *Device) GetSyncWord() uint16 {
	p, _ := d.ReadRegister(SX126X_REG_LORA_SYNC_WORD_MSB, 2)
	r := uint16(p[0])<<8 + uint16(p[1])
	return r
}

// SetPublicNetwork sets Sync Word to 0x3444 (Public) or 0x1424 (Private)
func (d *Device) SetPublicNetwork(enable bool) error {
	pt := d.GetPacketType()
	if pt != SX126X_PACKET_TYPE_LORA {
		return errWrongOperation
	}
	if enable {
		d.SetLoraSyncWord(SX126X_LORA_MAC_PUBLIC_SYNCWORD)
	} else {
		d.SetLoraSyncWord(SX126X_LORA_MAC_PRIVATE_SYNCWORD)
	}
	return nil
}

// SetPacketParam sets various packet-related params
func (d *Device) SetPacketParam(preambleLength uint16, headerType, crcType, payloadLength, invertIQ uint8) error {
	var p [6]uint8
	p[0] = uint8((preambleLength >> 8) & 0xFF)
	p[1] = uint8(preambleLength & 0xFF)
	p[2] = headerType
	p[3] = payloadLength
	p[4] = crcType
	p[5] = invertIQ
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_PARAMS, p[:])
	return nil
}

// SetPacketParamFSK sets various FSK packet-related params
func (d *Device) SetPacketParamFSK(preambleLen uint16, crcType uint8, syncWordLen uint8, addrCmp uint8, whiten uint8, packType uint8, packTypeLen uint8, preambleDetectorLen uint8) error {
	var p [9]uint8
	p[0] = uint8((preambleLen >> 8) & 0xFF)
	p[1] = uint8(preambleLen & 0xFF)
	p[2] = preambleDetectorLen
	p[3] = syncWordLen
	p[4] = addrCmp
	p[5] = packType
	p[6] = packTypeLen
	p[7] = crcType
	p[8] = whiten
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_PARAMS, p[:])
	println("SetPacketParamFSK values: ", hex.EncodeToString(p[:]))
	return nil
}

// SetBufferBaseAddress sets base address for buffer
func (d *Device) SetBufferBaseAddress(txBaseAddress, rxBaseAddress uint8) {
	var p [2]uint8
	p[0] = txBaseAddress
	p[1] = rxBaseAddress
	d.ExecSetCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, p[:])
}

// SetFrequency sets the radio frequency
// freq is the new frequency, in khz (eg 868120)
func (d *Device) SetFrequency(freq uint32) error {
	var p [4]uint8

	frf := (uint64(freq) * (uint64(1) << SX126X_DIV_EXPONENT)) / (SX126X_CRYSTAL_FREQ_MHZ * 1000)
	frf = 0x364ECCCC
	p[0] = uint8((frf >> 24) & 0xFF)
	p[1] = uint8((frf >> 16) & 0xFF)
	p[2] = uint8((frf >> 8) & 0xFF)
	p[3] = uint8((frf >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_RF_FREQUENCY, p[:])
	println("SetRfFrequency bytes: ", hex.EncodeToString(p[:]))
	return nil
}

// SetCurrentLimit sets max current in the module
func (d *Device) SetCurrentLimit(limit uint8) error {
	if limit > 140 {
		limit = 140
	}
	rawLimit := uint8(float32(limit) / 2.5)
	p := []uint8{rawLimit}
	d.WriteRegister(SX126X_REG_OCP_CONFIGURATION, p[:])
	return nil
}

// SetTxConfig sets power and rampup time
func (d *Device) SetTxParams(power int8, rampTime uint8) {
	var p [2]uint8

	if d.deviceType == DEVICE_TYPE_SX1261 {
		if power == 15 {
			d.SetPaConfig(0x06, 0x00, 0x01, 0x01)
		} else {
			d.SetPaConfig(0x04, 0x00, 0x01, 0x01)
		}
		if power > 14 {
			power = 14
		} else if power < -3 {
			power = -3
		}
		d.SetCurrentLimit(80) // Set max current limit to 80mA
	} else { // sx1262 and sx1268
		d.SetPaConfig(0x04, 0x07, 0x00, 0x01)
		if power > 22 {
			power = 22
		} else if power < -3 {
			power = -3
		}
		d.SetCurrentLimit(140) // Set max current limit to 140 mA
	}

	p[0] = uint8(power)
	p[1] = rampTime
	d.ExecSetCommand(SX126X_CMD_SET_TX_PARAMS, p[:])
}

// SetModulationParams sets the Lora modulation frequency
func (d *Device) SetModulationParams(spreadingFactor, bandwidth, codingRate, lowDataRateOptimize uint8) error {
	pt := d.GetPacketType()
	if pt != SX126X_PACKET_TYPE_LORA {
		return errWrongOperation
	}
	var p [4]uint8
	p[0] = spreadingFactor
	p[1] = bandwidth
	p[2] = codingRate
	p[3] = lowDataRateOptimize
	d.ExecSetCommand(SX126X_CMD_SET_MODULATION_PARAMS, p[:])
	return nil
}

// SetModulationParamsFSK sets the FSK modulation parameters
func (d *Device) SetModulationParamsFSK(baudRate uint32, pulseShape uint8, rxBandwidth uint8, freqDeviation uint32) error {
	pt := d.GetPacketType()
	if pt != SX126X_PACKET_TYPE_GFSK {
		return errWrongOperation
	}
	var p [8]uint8
	p[0] = uint8((baudRate >> 16) & 0xFF)
	p[1] = uint8((baudRate >> 8) & 0xFF)
	p[2] = uint8(baudRate & 0xFF)
	p[3] = pulseShape
	p[4] = rxBandwidth
	p[5] = uint8((freqDeviation >> 16) & 0xFF)
	p[6] = uint8((freqDeviation >> 8) & 0xFF)
	p[7] = uint8(freqDeviation & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_MODULATION_PARAMS, p[:])
	println("SetModulationParamsFSK values: ", hex.EncodeToString(p[:]))

	return nil
}

// CheckDeviceReady sleep until all busy flags clears
func (d *Device) CheckDeviceReady() error {
	if d.deepSleep == true {
		d.controller.SetNss(false)
		time.Sleep(time.Millisecond)
		d.controller.SetNss(true)
		d.deepSleep = false
	}
	return d.controller.WaitWhileBusy()
}

// ExecSetCommand send a command to configure the peripheral
func (d *Device) ExecSetCommand(cmd uint8, buf []uint8) {
	d.CheckDeviceReady()
	if cmd == SX126X_CMD_SET_SLEEP {
		d.deepSleep = true
	} else {
		d.deepSleep = false
	}
	d.controller.SetNss(false)
	// Send command and params
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmd)
	d.spiTxBuf = append(d.spiTxBuf, buf...)
	d.spi.Tx(d.spiTxBuf, nil)
	d.controller.SetNss(true)
	if cmd != SX126X_CMD_SET_SLEEP {
		d.controller.WaitWhileBusy()
	}
}

// ExecGetCommand queries the peripheral the peripheral
func (d *Device) ExecGetCommand(cmd uint8, size uint8) []uint8 {
	d.CheckDeviceReady()
	d.controller.SetNss(false)
	// Send the command and flush first status byte (as not used)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmd, 0x00)
	d.spi.Tx(d.spiTxBuf, nil)
	// Read resp
	d.spiRxBuf = d.spiRxBuf[:size]
	d.spi.Tx(nil, d.spiRxBuf)
	d.controller.SetNss(true)
	d.controller.WaitWhileBusy()
	return d.spiRxBuf
}

//
// Configuration
//

// SetFrequency() Sets current radio frequency
/*
func (d *Device) SetFrequency(freq uint32) error {
	d.frequency = freq
	d.loraConf.Freq = freq
	d.SetRfFrequency(freq)
	return nil // TODO CHECKS
}
*/

// SetIqMode() defines the current IQ Mode (Standard/Inverted)
// NB: Change will be applied at next RX / TX
func (d *Device) SetIqMode(mode uint8) {
	if mode == 0 {
		d.loraConf.Iq = lora.IQStandard
	} else {
		d.loraConf.Iq = lora.IQInverted
	}
}

// SetCodingRate() sets current Lora Coding Rate
// NB: Change will be applied at next RX / TX
func (d *Device) SetCodingRate(cr uint8) {
	d.loraConf.Cr = cr
}

// SetSyncWord defines the Sync Word
func (d *Device) SetSyncWord(sw []uint8) error {
	pt := d.GetPacketType()
	if pt == SX126X_PACKET_TYPE_GFSK {
		// Write sync words
		d.WriteRegister(SX126X_REG_SYNC_WORD_0, sw[:])

		// Save size
		d.syncWordLength = uint8(len(sw)) * 8
		d.SetPacketParamFSK(d.preambleLengthFSK, d.crcTypeFSK, d.syncWordLength, uint8(d.addrComp), d.whitening, d.packetType, d.packetTypeLen, d.preambleDetectLen)

	} else {
		return errWrongOperation
	}
	return nil
}

// SetCrc() sets current CRC
func (d *Device) SetCrc(len uint8, initial uint16, polynomial uint16, inverted bool) error {
	pt := d.GetPacketType()
	if pt == SX126X_PACKET_TYPE_LORA {
		// LoRa CRC doesn't allow to set CRC polynomial, initial value, or inversion
		if len > 0 {
			d.loraConf.Crc = lora.CRCOn
		} else {
			d.loraConf.Crc = lora.CRCOff
		}
		d.SetPacketParam(d.loraConf.Preamble, d.loraConf.HeaderType, d.loraConf.Crc, 0xFF, d.loraConf.Iq)

	} else if pt == SX126X_PACKET_TYPE_GFSK {

		switch len {
		case 0:
			d.crcTypeFSK = SX126X_GFSK_CRC_OFF
		case 1:
			if inverted {
				d.crcTypeFSK = SX126X_GFSK_CRC_1_BYTE_INV
			} else {
				d.crcTypeFSK = SX126X_GFSK_CRC_1_BYTE
			}
		case 2:
			if inverted {
				d.crcTypeFSK = SX126X_GFSK_CRC_2_BYTE_INV
			} else {
				d.crcTypeFSK = SX126X_GFSK_CRC_2_BYTE
			}
		default:
			return errInvalidCRCConfig
		}
		d.SetPacketParamFSK(d.preambleLengthFSK, d.crcTypeFSK, d.syncWordLength, uint8(d.addrComp), d.whitening, d.packetType, d.packetTypeLen, d.preambleDetectLen)

		// Initial CRC
		var p [2]uint8
		p[0] = uint8((initial >> 8) & 0xFF)
		p[1] = uint8(initial & 0xFF)
		d.WriteRegister(SX126X_REG_CRC_INITIAL_MSB, p[:])

		// Polynomial
		p[0] = uint8((polynomial >> 8) & 0xFF)
		p[1] = uint8(polynomial & 0xFF)
		d.WriteRegister(SX126X_REG_CRC_POLYNOMIAL_MSB, p[:])

	} else {
		return errWrongOperation
	}
	return nil
}

// SetSpreadingFactor sets current Lora Spreading Factor
func (d *Device) SetSpreadingFactor(sf uint8) error {
	pt := d.GetPacketType()
	if pt == SX126X_PACKET_TYPE_LORA {
		d.loraConf.Sf = sf

	} else {
		return errWrongOperation
	}
	return d.SetModulationParams(d.loraConf.Sf, d.loraConf.Bw, d.loraConf.Cr, d.loraConf.Ldr)
}

// SetPreambleLength sets current Lora Preamble Length
func (d *Device) SetPreambleLength(pl uint16) error {
	pt := d.GetPacketType()
	if pt == SX126X_PACKET_TYPE_LORA {
		d.preambleTypeLora = pl
		d.loraConf.Preamble = pl
		d.SetPacketParam(d.loraConf.Preamble, d.loraConf.HeaderType, d.loraConf.Crc, 0xFF, d.loraConf.Iq)
	} else if pt == SX126X_PACKET_TYPE_GFSK {
		d.preambleLengthFsk = pl
		d.SetPacketParamFSK(d.preambleLengthFSK, d.crcTypeFSK, d.syncWordLength, uint8(d.addrComp), d.whitening, d.packetType, d.packetTypeLen, d.preambleDetectLen)
	} else {
		return errWrongOperation
	}
	return nil
}

// SetTxPowerDbm sets current Lora TX Power in DBm
// NB: Change will be applied at next RX / TX
func (d *Device) SetTxPower(txpow int8) error {
	d.txPower = txpow
	d.loraConf.LoraTxPowerDBm = txpow
	d.SetTxParams(txpow, SX126X_PA_RAMP_200U)
	return nil // TODO add checks
}

// SetHeaderType sets implicit or explicit header mode
// NB: Change will be applied at next RX / TX
func (d *Device) SetHeaderType(headerType uint8) {
	d.loraConf.HeaderType = headerType
}

// SetBandwidth() sets current LORA Bandwidth
// CHECKED
func (d *Device) SetBandwidth(bw uint8) error {
	pt := d.GetPacketType()
	if pt != SX126X_PACKET_TYPE_LORA {
		return errWrongOperation
	}
	d.loraConf.Bw = bandwidth(bw)
	return d.SetModulationParams(d.loraConf.Sf, d.loraConf.Bw, d.loraConf.Cr, d.loraConf.Ldr)
}

// SetRxBandwidth() sets current FSK Bandwidth
// CHECKED
func (d *Device) SetRxBandwidth(rxbw float64) error {
	pt := d.GetPacketType()
	if pt != SX126X_PACKET_TYPE_GFSK {
		return errWrongOperation
	}
	d.rxBandwidthKhz = rxbw

	if math.Abs(rxbw-4.8) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_4_8
	} else if math.Abs(rxbw-5.8) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_5_8
	} else if math.Abs(rxbw-7.3) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_7_3
	} else if math.Abs(rxbw-9.7) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_9_7
	} else if math.Abs(rxbw-11.7) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_11_7
	} else if math.Abs(rxbw-14.6) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_14_6
	} else if math.Abs(rxbw-19.5) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_19_5
	} else if math.Abs(rxbw-23.4) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_23_4
	} else if math.Abs(rxbw-29.3) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_29_3
	} else if math.Abs(rxbw-39.0) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_39_0
	} else if math.Abs(rxbw-46.9) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_46_9
	} else if math.Abs(rxbw-58.6) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_58_6
	} else if math.Abs(rxbw-78.2) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_78_2
	} else if math.Abs(rxbw-93.8) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_93_8
	} else if math.Abs(rxbw-117.3) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_117_3
	} else if math.Abs(rxbw-187.2) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_187_2
	} else if math.Abs(rxbw-234.3) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_234_3
	} else if math.Abs(rxbw-312.0) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_312_0
	} else if math.Abs(rxbw-373.6) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_373_6
	} else if math.Abs(rxbw-467.0) <= 0.001 {
		d.rxBandwidth = SX126X_GFSK_RX_BW_467_0
	} else {
		return errInvalidRxBandwidth
	}

	return d.SetModulationParamsFSK(d.bitRate, d.pulseShape, d.rxBandwidth, d.frequencyDev)
}

// SetDataShaping() sets Gaussian filter on FSK
// CHECKED
func (d *Device) SetDataShaping(sh uint8) error {
	pt := d.GetPacketType()
	if pt != SX126X_PACKET_TYPE_GFSK {
		return errWrongOperation
	}
	Debug("FSK Set shaping:", sh)
	d.pulseShape = sh
	return d.SetModulationParamsFSK(d.bitRate, d.pulseShape, d.rxBandwidth, d.frequencyDev)
}

// SetFrequencyDeviation configures frequency deviation
// CHECKED
func (d *Device) SetFrequencyDeviation(freqDev float64) error {
	pt := d.GetPacketType()

	if pt != SX126X_PACKET_TYPE_GFSK {
		return errWrongOperation
	}
	if freqDev < 0 {
		freqDev = 0.6
	}

	if freqDev < 0.6 || freqDev > 200 {
		return errInvalidFreqDev
	}

	d.frequencyDev = uint32(((freqDev * 1000.0) * float64(uint32(1)<<25)) / (SX126X_CRYSTAL_FREQ_MHZ * 1000000.0))
	return d.SetModulationParamsFSK(d.bitRate, d.pulseShape, d.rxBandwidth, d.frequencyDev)
}

// SetBitRate configures bitrate
// br is bitrate value un kbaud
// CHECKED
func (d *Device) SetBitRate(br float64) error {
	pt := d.GetPacketType()
	if (pt != SX126X_PACKET_TYPE_GFSK) && (pt != SX126X_PACKET_TYPE_LR_FHSS) {
		return errWrongOperation
	}
	// TODO RANGE CHECKS
	if br < 0.6 || br > 300 {
		return errInvalidBaudrate
	}
	d.bitRate = (uint32)((SX126X_CRYSTAL_FREQ_MHZ * 1000000.0 * 32.0) / (br * 1000.0))
	return d.SetModulationParamsFSK(d.bitRate, d.pulseShape, d.rxBandwidth, d.frequencyDev)
}

// DisableAddressFiltering configures frequency deviation
// CHECKED
func (d *Device) DisableAddressFiltering() error {
	pt := d.GetPacketType()
	if pt != SX126X_PACKET_TYPE_GFSK {
		return errWrongOperation
	}
	d.addrComp = SX126X_GFSK_ADDRESS_FILT_OFF
	return (d.SetPacketParamFSK(d.preambleLengthFSK, d.crcTypeFSK, d.syncWordLength, uint8(d.addrComp), d.whitening, d.packetType, d.packetTypeLen, d.preambleDetectLen))
}

// SetPacketMode
func (d *Device) SetPacketMode(mode, len uint8) error {
	pt := d.GetPacketType()
	if pt != SX126X_PACKET_TYPE_GFSK {
		return errWrongOperation
	}
	d.packetType = mode
	d.packetTypeLen = len
	return (d.SetPacketParamFSK(d.preambleLengthFSK, d.crcTypeFSK, d.syncWordLength, uint8(d.addrComp), d.whitening, d.packetType, d.packetTypeLen, d.preambleDetectLen))
}

// SetWhitening configures whitening
// CHECKED
func (d *Device) SetWhitening(enable bool, initial uint16) error {
	pt := d.GetPacketType()
	if pt != SX126X_PACKET_TYPE_GFSK {
		return errWrongOperation
	}

	if !enable {
		d.whitening = SX126X_GFSK_WHITENING_OFF
	} else {
		d.whitening = SX126X_GFSK_WHITENING_ON
		// Read first as in radiolib
		dd, _ := d.ReadRegister(SX126X_REG_WHITENING_INITIAL_MSB, 1)

		// Initial (mark )
		var p [2]uint8
		p[0] = (dd[0] & 0xFE) | (uint8((initial >> 8) & 0x01))
		p[1] = uint8(initial & 0xFF)
		d.WriteRegister(SX126X_REG_WHITENING_INITIAL_MSB, p[:])

	}
	return (d.SetPacketParamFSK(d.preambleLengthFSK, d.crcTypeFSK, d.syncWordLength, uint8(d.addrComp), d.whitening, d.packetType, d.packetTypeLen, d.preambleDetectLen))
}

//
// Lora functions
//
//

// LoraConfig() defines Lora configuration for next Lora operations
func (d *Device) LoraConfig(cnf lora.Config) {
	// Save given configuration
	d.loraConf = cnf
	d.loraConf.SyncWord = syncword(int(cnf.SyncWord))
	// Switch to standby prior to configuration changes
	d.SetStandby()
	// Clear errors, disable radio interrupts for the moment
	d.ClearDeviceErrors()
	d.ClearIrqStatus(SX126X_IRQ_ALL)
	d.SetDioIrqParams(0x00, 0x00, 0x00, 0x00)
	// Define radio operation mode
	d.SetPacketType(SX126X_PACKET_TYPE_LORA)
	//FIXME d.SetRfFrequency(d.loraConf.Freq)
	d.SetModulationParams(d.loraConf.Sf, bandwidth(d.loraConf.Bw), d.loraConf.Cr, d.loraConf.Ldr)
	d.SetTxParams(d.loraConf.LoraTxPowerDBm, SX126X_PA_RAMP_200U)
	//FXME d.SetSyncWord(d.loraConf.SyncWord)
	d.SetBufferBaseAddress(0, 0)
}

// BeginFSK prepares for FSK operation
func (d *Device) BeginFSK() error {
	Debug("BeginFSK...")

	// Check radio is available
	if !d.DetectDevice() {
		return errRadioNotFound
	}

	// initialize configuration variables (will be overwritten during public settings configuration)
	d.bitRate = 21333      // 48.0 kbps
	d.frequencyDev = 52428 // 50.0 kHz
	d.frequencyKhz = 868925
	d.rxBandwidth = SX126X_GFSK_RX_BW_234_3
	d.rxBandwidthKhz = 234.3
	d.pulseShape = SX126X_GFSK_FILTER_GAUSS_0_5
	d.crcTypeFSK = SX126X_GFSK_CRC_2_BYTE_INV // CCIT CRC configuration
	d.preambleLengthFSK = 512
	d.addrComp = SX126X_GFSK_ADDRESS_FILT_OFF
	d.whitening = SX126X_GFSK_WHITENING_OFF
	d.preambleDetectLen = SX126X_GFSK_PREAMBLE_DETECT_16
	d.packetType = SX126X_GFSK_PACKET_VARIABLE
	d.packetTypeLen = SX126X_MAX_PACKET_LENGTH

	// Reset module
	d.Reset()
	d.SetStandby()

	// Clear errors, disable radio interrupts
	d.ClearDeviceErrors()
	d.ClearIrqStatus(SX126X_IRQ_ALL)
	d.SetDioIrqParams(0x00, 0x00, 0x00, 0x00)

	// radio configuration()
	d.SetPacketType(SX126X_PACKET_TYPE_GFSK)
	d.SetFrequency(d.frequencyKhz) //
	d.SetTxParams(10, SX126X_PA_RAMP_200U)
	d.SetBufferBaseAddress(0, 0) // 5

	//Calibration
	d.Calibrate(SX126X_CALIBRATE_ALL)
	time.Sleep(time.Millisecond * 250) // FIXME ...
	d.SetCurrentLimit(60)

	println("STARTFSK ERRORS:", d.GetDeviceErrors())
	return nil
}

// Tx sends a packet
// pkt is actual payload
// timeoutMs is TX timeout in ms
func (d *Device) Tx(pkt []uint8, timeoutMs uint32) error {

	// Switch TX antenna
	if d.controller != nil {
		err := d.controller.SetRfSwitchMode(RFSWITCH_TX_HP)
		if err != nil {
			return err
		}
	}

	// Clear IRQ
	d.ClearIrqStatus(SX126X_IRQ_ALL)
	irqVal := uint16(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR)
	d.SetDioIrqParams(irqVal, irqVal, SX126X_IRQ_NONE, SX126X_IRQ_NONE)

	// Write packet to buffer and send
	d.SetBufferBaseAddress(0, 0)
	d.WriteBuffer(pkt)
	d.SetTx(timeoutMsToRtcSteps(timeoutMs))

	// Wait radio event
	msg := <-d.GetRadioEventChan()
	if msg.EventType != lora.RadioEventTxDone {
		return errUnexpectedTxRadioEvent
	}
	return nil
}

// LoraRx tries to receive a Lora packet (with timeout in milliseconds)
func (d *Device) Rx(timeoutMs uint32) ([]uint8, error) {

	// Enable RX Switch
	if d.controller != nil {
		err := d.controller.SetRfSwitchMode(RFSWITCH_RX)
		if err != nil {
			return nil, err
		}
	}

	// Disable interrupts
	d.ClearIrqStatus(SX126X_IRQ_ALL)
	irqVal := uint16(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR)

	//	d.SetStandby()
	//	d.SetBufferBaseAddress(0, 0)
	//d.SetPacketParamFSK(d.preambleLengthFSK, d.crcTypeFSK, d.syncWordLength, uint8(d.addrComp), d.whitening, d.packetType, 0, d.preambleDetectLen)

	//Debug("RX() set irq params")
	d.SetDioIrqParams(irqVal, irqVal, SX126X_IRQ_NONE, SX126X_IRQ_NONE)
	//Debug("Set RX Mode")
	d.SetRx(timeoutMsToRtcSteps(timeoutMs))

	msg := <-d.GetRadioEventChan()

	switch msg.EventType {
	case lora.RadioEventTimeout:
		//	Debug("RadioEventTimeout IRQ Status:%d", msg.IRQStatus)
		return nil, nil
	case lora.RadioEventCrcError:
		//	Debug("RadioEventCrcError IRQ Status:%d", msg.IRQStatus)
		return nil, nil
	case lora.RadioEventRxDone:
		//	Debug("RadioEventRxDone IRQ Status:%d", msg.IRQStatus)
		pLen, pStart := d.GetRxBufferStatus()
		d.SetBufferBaseAddress(0, pStart+1)
		pkt := d.ReadBuffer(pLen + 1)
		pkt = pkt[1:]
		return pkt, nil
	case lora.RadioEventWatchdog:
		//	Debug("RadioEventWatchdog IRQ Status:%d", msg.IRQStatus)
		return nil, nil
	case lora.RadioEventUnhandled:
		//	Debug("RadioEventUnhandled IRQ Status:%d", msg.IRQStatus)
		return nil, nil

	}

	/*
		if msg.EventType == lora.RadioEventTimeout {
			Debug("RXTimeout, IRQ Status:%d", msg.IRQStatus)
			return nil, nil
		} else if msg.EventType != lora.RadioEventRxDone {
			Debug("EventType:%d IRQ Status:%d", msg.EventType, msg.IRQStatus)
			return nil, errUnexpectedRxRadioEvent
		} else if msg.EventType != lora.RadioEventUnhandled {
	*/
	return nil, errWrongOperation

}

// HandleInterrupt must be called by main code on DIO state change.
func (d *Device) HandleInterrupt() {
	st := d.GetIrqStatus()
	d.ClearIrqStatus(SX126X_IRQ_ALL)

	if (st & SX126X_IRQ_RX_DONE) > 0 {
		select {
		case d.radioEventChan <- lora.RadioEvent{lora.RadioEventRxDone, uint16(st), nil}:
		default:
		}
	}

	if (st & SX126X_IRQ_TX_DONE) > 0 {
		select {
		case d.radioEventChan <- lora.RadioEvent{lora.RadioEventTxDone, uint16(st), nil}:
		default:
		}
	}

	if (st & SX126X_IRQ_TIMEOUT) > 0 {
		select {
		case d.radioEventChan <- lora.RadioEvent{lora.RadioEventTimeout, uint16(st), nil}:
		default:
		}

	}

	if (st & SX126X_IRQ_CRC_ERR) > 0 {
		select {
		case d.radioEventChan <- lora.RadioEvent{lora.RadioEventCrcError, uint16(st), nil}:
		default:
		}
	}

}

func bandwidth(bw uint8) uint8 {
	switch bw {
	case lora.Bandwidth_7_8:
		return SX126X_LORA_BW_7_8
	case lora.Bandwidth_10_4:
		return SX126X_LORA_BW_10_4
	case lora.Bandwidth_15_6:
		return SX126X_LORA_BW_15_6
	case lora.Bandwidth_20_8:
		return SX126X_LORA_BW_20_8
	case lora.Bandwidth_31_25:
		return SX126X_LORA_BW_31_25
	case lora.Bandwidth_41_7:
		return SX126X_LORA_BW_41_7
	case lora.Bandwidth_62_5:
		return SX126X_LORA_BW_62_5
	case lora.Bandwidth_125_0:
		return SX126X_LORA_BW_125_0
	case lora.Bandwidth_250_0:
		return SX126X_LORA_BW_250_0
	case lora.Bandwidth_500_0:
		return SX126X_LORA_BW_500_0
	default:
		return 0
	}
}

func syncword(sw int) uint16 {
	if sw == lora.SyncPublic {
		return SX126X_LORA_MAC_PUBLIC_SYNCWORD
	}
	return SX126X_LORA_MAC_PRIVATE_SYNCWORD
}

func Debug(a ...interface{}) {
	if debug {
		println("DBG", fmt.Sprint(a...))
	}
}
