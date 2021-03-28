// Package sx126x provides a driver for SX126x LoRa transceivers.

package sx126x

import (
	"device/stm32"
	"errors"
	"machine"
	"time"
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
	spi            machine.SPI
	radioEventChan chan RadioEvent
	cnf            Config
}

// Config holds the LoRa configuration parameters
type Config struct {
}

// New creates a new SX126x connection. The SPI bus must already be configured.
func New(spi machine.SPI) Device {
	k := Device{
		spi:            spi,
		radioEventChan: make(chan RadioEvent, 10),
	}
	return k
}

// --------------------------------------------------
// Radio-specific functions
// --------------------------------------------------

// GetStatus returns radio status
func (d *Device) GetStatus() uint8 {
	r := d.ExecGetCommand(SX126X_CMD_GET_STATUS, 1)
	return r[0]
}

// GetPackeType returns radio status
func (d *Device) GetPacketType() uint8 {
	r := d.ExecGetCommand(SX126X_CMD_GET_PACKET_TYPE, 1)
	return r[0]
}

// GetIrqStatus returns radio status
func (d *Device) GetIrqStatus() uint16 {
	r := d.ExecGetCommand(SX126X_CMD_GET_STATUS, 3)
	ret := uint16(r[0]<<8 | r[1])
	return ret
}

// SetTx enable Tx Mode with Tx Timeout (R)
func (d *Device) SetTx(t uint32) {
	var p [3]uint8
	p[0] = uint8((t >> 16) & 0xFF)
	p[1] = uint8((t >> 8) & 0xFF)
	p[2] = uint8((t >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_TX, p[:])
}

// SetRx enable Rx Mode with Rx Timeout (R)
func (d *Device) SetRx(t uint32) {
	var p [3]uint8
	p[0] = uint8((t >> 16) & 0xFF)
	p[1] = uint8((t >> 8) & 0xFF)
	p[2] = uint8((t >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_RX, p[:])
}

// Sleep switch device to sleep mode
func (d *Device) Sleep() {
	// Todo : Ant switch ? Warm start ?
	d.ExecSetCommand(SX126X_CMD_SET_SLEEP, []uint8{SX126X_SLEEP_START_WARM | SX126X_SLEEP_RTC_OFF})
}

// Standby switch device to RC standby mode (R)
func (d *Device) Standby() {
	d.ExecSetCommand(SX126X_CMD_SET_STANDBY, []uint8{SX126X_STANDBY_RC})
}

// SetPacketType sets the packet type
func (d *Device) SetPacketType(packetType uint8) {
	var p [1]uint8
	p[0] = packetType
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_TYPE, p[:])
}

// SetSyncWord defines the Sync Word to yse
func (d *Device) SetSyncWord(syncword uint16) {
	var p [2]uint8
	p[0] = uint8((syncword >> 8) & 0xFF)
	p[1] = uint8((syncword >> 0) & 0xFF)
	d.WriteRegister(SX126X_REG_LORA_SYNC_WORD_MSB, p[:])
}

// SetLoraPublicNetwork sets Sync Word to 0x3444 (Public) or 0x1424 (Private)
func (d *Device) SetLoraPublicNetwork(enable bool) {
	if enable {
		d.SetSyncWord(SX126X_LORA_MAC_PUBLIC_SYNCWORD)
	} else {
		d.SetSyncWord(SX126X_LORA_MAC_PRIVATE_SYNCWORD)

	}
}

// SetPacketParam sets various packet-related params (R)
func (d *Device) SetPacketParam(preambleLength uint16, crcType, payloadLength, headerType, invertIQ uint8) {
	var p [6]uint8
	p[0] = uint8((preambleLength >> 8) & 0xFF)
	p[1] = uint8(preambleLength & 0xFF)
	p[2] = headerType
	p[3] = payloadLength
	p[4] = crcType
	p[5] = invertIQ
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_PARAMS, p[:])
}

// SetBufferBaseAddress sets base address for buffer
func (d *Device) SetBufferBaseAddress(txBaseAddress, rxBaseAddress uint8) {
	var p [2]uint8
	p[0] = txBaseAddress
	p[1] = rxBaseAddress
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_PARAMS, p[:])
}

// SetRfFrequency sets the radio frequency (R)
func (d *Device) SetRfFrequency(frequency uint32) {
	var p [4]uint8
	freq := uint32(float64(frequency) / float64(SX126X_FREQUENCY_STEP_SIZE)) // Convert to PLL Steps
	println("SetRfFreq:", freq)
	p[0] = uint8((freq >> 24) & 0xFF)
	p[1] = uint8((freq >> 16) & 0xFF)
	p[2] = uint8((freq >> 8) & 0xFF)
	p[3] = uint8((freq >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_RF_FREQUENCY, p[:])
}

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

// SetPaConfig sets the Power Amplifier configuration (R)
func (d *Device) SetPaConfig(paDutyCycle, hpMax, deviceSel, paLut uint8) {
	var p [4]uint8
	p[0] = paDutyCycle
	p[1] = hpMax
	p[2] = deviceSel
	p[3] = paLut
	d.ExecSetCommand(SX126X_CMD_SET_PA_CONFIG, p[:])
}

// SetTxConfig sets power and rampup time (R)
func (d *Device) SetTxParams(power, rampTime uint8) {
	var p [2]uint8
	p[0] = power
	p[1] = rampTime
	d.ExecSetCommand(SX126X_CMD_SET_TX_PARAMS, p[:])
}

// SetModulationParams sets the Lora modulation frequency (R)
func (d *Device) SetModulationParams(spreadingFactor, bandwidth, codingRate, lowDataRateOptimize uint8) {
	var p [4]uint8
	p[0] = spreadingFactor
	p[1] = bandwidth
	p[2] = codingRate
	p[3] = lowDataRateOptimize
	d.ExecSetCommand(SX126X_CMD_SET_MODULATION_PARAMS, p[:])
}

// ClearIrqStatus clears IRQ flags
func (d *Device) ClearIrqStatus(clearIrqParams uint16) {
	var p [2]uint8
	p[0] = uint8((clearIrqParams >> 8) & 0xFF)
	p[1] = uint8(clearIrqParams & 0xFF)
	d.ExecSetCommand(SX126X_CMD_CLEAR_IRQ_STATUS, p[:])
}

// ***************
// BUFFERS
// **************
// WriteBuffer sets base address for buffer
func (d *Device) WriteBuffer(data []uint8) {
	p := []uint8{0} // Zero offset
	p = append(p, data...)
	d.ExecSetCommand(SX126X_CMD_WRITE_BUFFER, p[:])
}

// --------------------------------------------------
// Internal functions
// --------------------------------------------------

// Reset re-initialize the sx127x device
// sx1276 have nRST pin
/*
func (d *Device) Reset() {
	d.rstPin.Low()
	time.Sleep(100 * time.Millisecond)
	d.rstPin.High()
	time.Sleep(100 * time.Millisecond)
}
*/

// ReadRegister reads register value
func (d *Device) ReadRegister(addr, size uint16) []uint8 {
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)

	buf := []byte{SX126X_CMD_READ_REGISTER, uint8((addr >> 8) & 0xFF), uint8(addr & 0xFF), 0x00} // Sec. 13.2.2
	d.spi.Tx(buf, nil)

	ret := make([]uint8, size)
	for i := uint16(0); i < size; i++ {
		d.spi.Tx([]uint8{0x00}, ret)
	}
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
	d.WaitBusy()
	return ret
}

// CheckDeviceReady sleep until all busy flags clears
func (d *Device) CheckDeviceReady() error {
	// Wakeup radio in case of sleep mode: Select-Unselect radio
	// In case of Deep Seep ????
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
	time.Sleep(time.Millisecond)
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)
	return d.WaitBusy()
}

// WaitBusy sleep until all busy flags clears
func (d *Device) WaitBusy() error {
	count := 10000
	// RM0461 Sec 5.3 Radio Busy Management
	for count > 0 {
		fl1 := stm32.PWR.SR2.HasBits(stm32.PWR_SR2_RFBUSYMS)
		fl2 := stm32.PWR.SR2.HasBits(stm32.PWR_SR2_RFBUSYS)
		if !fl1 && !fl2 {
			return nil
		}
		count--
	}
	return errors.New("WaitBusy Timeout")
}

// WriteRegister writes value to register
func (d *Device) WriteRegister(reg uint16, data []uint8) {
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)

	buf := []byte{SX126X_CMD_WRITE_REGISTER, uint8((reg >> 8) & 0xFF), uint8(reg & 0xFF)}

	d.spi.Tx(buf, nil)
	d.spi.Tx(data, nil)

	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
	d.WaitBusy()
}

// ExecSetCommand send a command to configure the peripheral
func (d *Device) ExecSetCommand(cmd uint8, buf []uint8) {
	d.CheckDeviceReady()
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)
	d.spi.Tx([]byte{cmd}, nil)
	d.spi.Tx(buf, nil)
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
	if cmd != SX126X_CMD_SET_SLEEP {
		d.WaitBusy()
	}
}

// ExecGetCommand queries the peripheral the peripheral
func (d *Device) ExecGetCommand(cmd uint8, size int) []uint8 {
	var buf []uint8
	d.CheckDeviceReady()
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)

	d.spi.Tx([]byte{cmd}, nil)
	d.spi.Tx([]byte{0x00}, nil) // Drop first status byte
	var value [1]byte
	for i := 0; i < size; i++ {
		d.spi.Tx(nil, value[:])
		buf = append(buf, value[0]) // Read bytes
	}

	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
	d.WaitBusy()
	return buf
}
