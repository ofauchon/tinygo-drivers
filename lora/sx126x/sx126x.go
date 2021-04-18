// Package sx126x provides a driver for SX126x LoRa transceivers.

package sx126x

import (
	"device/stm32"
	"errors"
	"fmt"
	"machine"
	"runtime/volatile"
	"time"
	"unsafe"
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

	deepSleep bool
}

// Config holds the LoRa configuration parameters
type Config struct {
}

// New creates a new SX126x connection. The SPI bus must already be configured.
func New(spi machine.SPI) Device {
	k := Device{
		spi:            spi,
		deepSleep:      true,
		radioEventChan: make(chan RadioEvent, 10),
	}
	return k
}

// --------------------------------------------------
// Get function (state, irq ...)
// --------------------------------------------------

// GetStatus returns radio status
func (d *Device) GetStatus() uint8 {
	r := d.ExecGetCommand(SX126X_CMD_GET_STATUS, 1)
	return r[0]
}

// GetPackeType returns current Packet Type
func (d *Device) GetPacketType() uint8 {
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

// GetIrqStatus returns radio status
func (d *Device) GetIrqStatus() uint16 {
	r := d.ExecGetCommand(SX126X_CMD_GET_IRQ_STATUS, 2)
	ret := uint16(r[0]<<8 | r[1])
	return ret
}

// -----------------------------------
// STATE MANAGEMENT (RX/TX/STBY/SLEEP)
// -----------------------------------

// Sleep switch device to sleep mode
func (d *Device) SetSleep() {
	// Todo : Ant switch ? Warm start ?
	d.ExecSetCommand(SX126X_CMD_SET_SLEEP, []uint8{SX126X_SLEEP_START_WARM | SX126X_SLEEP_RTC_OFF})
}

// Standby switch device to RC standby mode (R)
func (d *Device) SetStandby() {
	println("SetStandby")
	d.ExecSetCommand(SX126X_CMD_SET_STANDBY, []uint8{SX126X_STANDBY_RC})
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

// SetTxContinuousWave sends continuous Radio tone
func (d *Device) SetTxContinuousWave() {
	d.ExecSetCommand(SX126X_CMD_SET_TX_CONTINUOUS_WAVE, []uint8{})
}

// SetTxContinuousPreamble Send continuous preamble
func (d *Device) SetTxContinuousPreamble() {
	d.ExecSetCommand(SX126X_CMD_SET_TX_INFINITE_PREAMBLE, []uint8{})
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
func (d *Device) SetPacketParam(preambleLength uint16, headerType, crcType, payloadLength, invertIQ uint8) {
	var p [6]uint8
	p[0] = uint8((preambleLength >> 8) & 0xFF)
	p[1] = uint8(preambleLength & 0xFF)
	p[2] = headerType
	p[3] = payloadLength
	//println("******************* P3=", p[3])
	p[4] = crcType
	p[5] = invertIQ
	d.ExecSetCommand(SX126X_CMD_SET_PACKET_PARAMS, p[:])
}

// SetBufferBaseAddress sets base address for buffer
func (d *Device) SetBufferBaseAddress(txBaseAddress, rxBaseAddress uint8) {
	var p [2]uint8
	p[0] = txBaseAddress
	p[1] = rxBaseAddress
	d.ExecSetCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, p[:])
}

// SetRfFrequency sets the radio frequency (R)
func (d *Device) SetRfFrequency(frequency uint32) {
	var p [4]uint8
	//	freq := uint32(float64(frequency) / float64(SX126X_FREQUENCY_STEP_SIZE)) // Convert to PLL Steps
	//  channel = (uint32_t) ((((uint64_t) freq)<<25)/(XTAL_FREQ) );               \

	freq := uint32((uint64(frequency) << 25) / 32000000)
	println("SetRfFreq:", freq)
	p[0] = uint8((freq >> 24) & 0xFF)
	p[1] = uint8((freq >> 16) & 0xFF)
	p[2] = uint8((freq >> 8) & 0xFF)
	p[3] = uint8((freq >> 0) & 0xFF)
	d.ExecSetCommand(SX126X_CMD_SET_RF_FREQUENCY, p[:])
}

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

// CalibrateAll
func (d *Device) CalibrateAll() {
	p := []uint8{SX126X_CALIBRATE_ALL}
	d.ExecSetCommand(SX126X_CMD_CALIBRATE, p[:])
}

// CalibrateImage
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

// SetCurrentLimit sets max current
func (d *Device) SetCurrentLimit(limit uint8) {
	if limit > 140 {
		limit = 140
	}
	rawLimit := uint8(float32(limit) / 2.5)
	p := []uint8{rawLimit}
	d.WriteRegister(SX126X_REG_OCP_CONFIGURATION, p[:])
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

// *******
// BUFFERS
// *******

// WriteBuffer write data from current buffer position
func (d *Device) WriteBuffer(data []uint8) {
	p := []uint8{0} // Zero offset
	p = append(p, data...)
	d.ExecSetCommand(SX126X_CMD_WRITE_BUFFER, p[:])
}

// ReadBuffer Reads size bytes from current buffer position
func (d *Device) ReadBuffer(size int) []uint8 {
	ret := d.ExecGetCommand(SX126X_CMD_READ_BUFFER, size)
	return ret
}

// CheckDeviceReady sleep until all busy flags clears
func (d *Device) CheckDeviceReady() error {
	//println("CheckDeviceReady")
	if d.deepSleep == true {
		//		println("DeepSleep, wake up SubGhz with NSS")
		stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS)
		time.Sleep(time.Millisecond)
		stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)
		d.deepSleep = false
	}
	return d.WaitBusy()
}

// WaitBusy sleep until all busy flags clears
func (d *Device) WaitBusy() error {
	//	println("WaitBusy")
	count := 2000
	var rfbusyms, rfbusys bool
	for count > 0 {
		//rfbusyms = stm32.PWR.SR2.HasBits(stm32.PWR_SR2_RFBUSYMS)
		rfbusys = stm32.PWR.SR2.HasBits(stm32.PWR_SR2_RFBUSYS)

		//if !(rfbusyms && rfbusys) {
		if !rfbusyms {
			return nil
		}
		time.Sleep(time.Millisecond)
		count--
	}
	println("ERR: WaitBusy timeout. rfbusys:", rfbusys, " rfbusyms:", rfbusyms)
	return errors.New("WaitBusy Timeout")
}

// ReadRegister reads register value
func (d *Device) ReadRegister(addr, size uint16) ([]uint8, error) {
	d.CheckDeviceReady()
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS) // NSS=0
	// Send command
	d.SpiTx(SX126X_CMD_READ_REGISTER)
	d.SpiTx(uint8((addr & 0xFF00) >> 8))
	d.SpiTx(uint8(addr & 0x00FF))
	d.SpiTx(uint8(0x00)) // This byte is for status (unused yet)

	var ret []uint8
	for i := 0; i < int(size); i++ {
		b := d.SpiTx(0x00) // Read
		//println("IN: ", b)
		ret = append(ret, b)
	}
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS) // NSS=1
	d.WaitBusy()
	return ret, nil
}

// WriteRegister writes value to register
func (d *Device) WriteRegister(addr uint16, data []uint8) {
	d.CheckDeviceReady()
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS) // NSS=0
	// Send command
	d.SpiTx(SX126X_CMD_WRITE_REGISTER)
	d.SpiTx(uint8((addr & 0xFF00) >> 8))
	d.SpiTx(uint8(addr & 0x00FF))
	for i := 0; i < len(data); i++ {
		d.SpiTx(data[i])
	}
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS) // NSS=1
	d.WaitBusy()
}

// ExecSetCommand send a command to configure the peripheral
func (d *Device) ExecSetCommand(cmd uint8, buf []uint8) {
	d.CheckDeviceReady()
	if cmd == SX126X_CMD_SET_SLEEP {
		d.deepSleep = true
	} else {
		d.deepSleep = false
	}
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS) // NSS=0
	// Send command and params
	d.SpiTx(cmd)
	for i := 0; i < len(buf); i++ {
		d.SpiTx(buf[i])
	}
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS) // NSS=1
	if cmd != SX126X_CMD_SET_SLEEP {
		d.WaitBusy()
	}
}

// ExecGetCommand queries the peripheral the peripheral
func (d *Device) ExecGetCommand(cmd uint8, size int) []uint8 {
	d.CheckDeviceReady()
	stm32.PWR.SUBGHZSPICR.ClearBits(stm32.PWR_SUBGHZSPICR_NSS) // NSS=0
	// Send the command and flush first status byte (as not used)
	d.SpiTx(cmd)
	d.SpiTx(0x00)
	var ret []uint8
	// Receive given number of bytes from radio
	for i := 0; i < size; i++ {
		ret = append(ret, d.SpiTx(0xFF))
	}
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS) // NSS=1
	d.WaitBusy()
	return ret
}

//*********************************
// STM3Cube like SPI communications
//*********************************

//SpiTx Transmit byte over SPI
func (d *Device) SpiTx(cmd uint8) uint8 {
	// Wait until Tx Buffer empty
	for !stm32.SPI3.SR.HasBits(stm32.SPI_SR_TXE) {
	}
	// Write to data register

	//stm32.SPI3.DR.Set(uint32(cmd))
	volatile.StoreUint8((*uint8)(unsafe.Pointer(&stm32.SPI3.DR.Reg)), cmd)

	// Wait for Rx Data
	for !stm32.SPI3.SR.HasBits(stm32.SPI_SR_RXNE) {
	}
	// Flush RX data
	ret := uint8(stm32.SPI3.DR.Get())
	//println("SPITX >", dec2hex(cmd), " <", dec2hex(ret))
	return ret
}

//SpiRx Receive byte over SPI
func (d *Device) SpiRx() uint8 {
	return d.SpiTx(0xFF)
}

func dec2hex(dec uint8) string {
	return fmt.Sprintf("%02x", dec)
}
