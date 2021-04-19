package main

// This is the most minimal blinky example and should run almost everywhere.
//
import (
	"device/stm32"
	"errors"
	"machine"
	"time"

	"github.com/ofauchon/go-lorawan-stack"
	"tinygo.org/x/drivers/lora/sx126x"
)

func dbg(msg string) {
	println(msg)
}

const ()

// byteToHex return string hex representation of byte
func ByteToHex(b byte) string {
	bb := (b >> 4) & 0x0F
	ret := ""
	if bb < 10 {
		ret += string(rune('0' + bb))
	} else {
		ret += string(rune('A' + (bb - 10)))
	}

	bb = (b) & 0xF
	if bb < 10 {
		ret += string(rune('0' + bb))
	} else {
		ret += string(rune('A' + (bb - 10)))
	}
	return ret
}

// BytesToHexString converts byte slice to hex string representation
func BytesToHexString(data []byte) string {
	s := ""
	for i := 0; i < len(data); i++ {
		s += ByteToHex(data[i])
	}
	return s
}

// SubGhzInit enable radio module
func SubGhzInit() error {

	// Enable APB3 Periph clock
	stm32.RCC.APB3ENR.SetBits(stm32.RCC_APB3ENR_SUBGHZSPIEN)
	_ = stm32.RCC.APB3ENR.Get() //Delay after RCC periph clock enable

	// Enable TXCO and HSE
	stm32.RCC.CR.SetBits(stm32.RCC_CR_HSEBYPPWR)
	stm32.RCC.CR.SetBits(stm32.RCC_CR_HSEON)
	for !stm32.RCC.CR.HasBits(stm32.RCC_CR_HSERDY) {
	}

	// Disable radio reset and wait it's ready
	stm32.RCC.CSR.ClearBits(stm32.RCC_CSR_RFRST)
	for stm32.RCC.CSR.HasBits(stm32.RCC_CSR_RFRSTF) {
	}

	// Disable radio NSS=1
	stm32.PWR.SUBGHZSPICR.SetBits(stm32.PWR_SUBGHZSPICR_NSS)

	// Enable Exti Line 44: Radio IRQ ITs for CPU1 (RM0461-14.3.1)
	stm32.EXTI.IMR2.SetBits(0x1000) // IM44 ===> TEST

	// Enable radio busy wakeup from Standby for CPU
	stm32.PWR.CR3.SetBits(stm32.PWR_CR3_EWRFBUSY)

	// Clear busy flag
	//stm32.PWR.SCR.SetBits(stm32.PWR_SCR_CWRFBUSYF)
	stm32.PWR.SCR.Set(stm32.PWR_SCR_CWRFBUSYF)
	//println("GSPI PWR_SCR:", stm32.PWR.SCR.Get(), " PWR_CR3:", stm32.PWR.CR3.Get(), " EXTI_IMR2:", stm32.EXTI.IMR2.Get(), " RCC_CR:", stm32.RCC.CR.Get())
	//println("GSPI RCC_APB3ENR:", stm32.RCC.APB3ENR.Get(), " PWR_SUBGHZSPICR:", stm32.PWR.SUBGHZSPICR.Get())

	//  SUBGHZSPI configuration
	stm32.SPI3.CR1.ClearBits(stm32.SPI_CR1_SPE)                                                   // Disable SPI
	stm32.SPI3.CR1.Set(stm32.SPI_CR1_MSTR | stm32.SPI_CR1_SSI | (0b010 << 3) | stm32.SPI_CR1_SSM) // Software Slave Management (NSS) + /8 prescaler
	stm32.SPI3.CR2.Set(stm32.SPI_CR2_FRXTH | (0b111 << 8))                                        // FIFO Threshold and 8bit size
	stm32.SPI3.CR1.SetBits(stm32.SPI_CR1_SPE)                                                     // Enable SPI
	//println("GSPI REG CR1:", stm32.SPI3.CR1.Get(), " CR2:", stm32.SPI3.CR2.Get())

	return nil
}

// configureLora initialize
func configureLora(radio sx126x.Device) {

	radio.SetStandby()

	radio.SetPacketType(sx126x.SX126X_PACKET_TYPE_LORA)
	radio.SetRfFrequency(868100000) // Needs to be done in FS/RX/TX mode ?

	radio.SetBufferBaseAddress(0, 0)

	radio.ClearIrqStatus(sx126x.SX126X_IRQ_ALL)
	radio.SetDioIrqParams(sx126x.SX126X_IRQ_TX_DONE|sx126x.SX126X_IRQ_TIMEOUT|sx126x.SX126X_IRQ_RX_DONE, sx126x.SX126X_IRQ_TX_DONE, 0x00, 0x00)

	radio.CalibrateAll()
	time.Sleep(10 * time.Millisecond)

	radio.SetCurrentLimit(60)

	radio.SetModulationParams(12, sx126x.SX126X_LORA_BW_125_0, sx126x.SX126X_LORA_CR_4_7, sx126x.SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF)
	radio.SetPaConfig(0x04, 0x07, 0x00, 0x01)
	radio.SetTxParams(0x16, sx126x.SX126X_PA_RAMP_200U)
	radio.SetBufferBaseAddress(0, 0)

	radio.SetLoraPublicNetwork(true)

	radio.ClearDeviceErrors()
	radio.ClearIrqStatus(sx126x.SX126X_IRQ_ALL)

	// Configure RF GPIO
	// LoRa-E5 module ONLY transmits through RFO_HP:
	// Receive: PA4=1, PB5=0
	// Transmit(high output power, SMPS mode): PA4=0, PB5=1
	machine.PA4.Configure(machine.PinConfig{Mode: machine.PinOutput})
	machine.PB5.Configure(machine.PinConfig{Mode: machine.PinOutput})
}

func xstatus(lora sx126x.Device) {
	sta := lora.GetStatus()
	sti := lora.GetIrqStatus()
	chipmode := (sta & (0x7 << 4)) >> 4
	cmdstatus := (sta & (0x7 << 1)) >> 1
	println(">> STATUS : mode:", chipmode, " cmd:", cmdstatus, " irq:", sti, "deverr:", lora.GetDeviceErrors())
	lora.ClearDeviceErrors()
}

// Helper Error
func checkErr(e error) {
	if e != nil {
		println("!!! Error:", e, " -> Stopping")
		for {
		}
	}
}

// LoraTx
func LoraTx(radio sx126x.Device, pkt []uint8) error {

	radio.ClearIrqStatus(sx126x.SX126X_IRQ_ALL)
	timeout := (uint32)(1000000 / 15.625) // 1sec

	// Set correct output (LoraE5 specific)
	machine.PA4.Set(false)
	machine.PB5.Set(true)

	// Define packet and modulation configuration (CRC ON, IQ OFF)
	radio.SetRfFrequency(868100000)
	radio.SetModulationParams(8, sx126x.SX126X_LORA_BW_125_0, sx126x.SX126X_LORA_CR_4_7, sx126x.SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF)
	radio.SetPacketParam(8, sx126x.SX126X_LORA_HEADER_EXPLICIT, sx126x.SX126X_LORA_CRC_ON, uint8(len(pkt)), sx126x.SX126X_LORA_IQ_STANDARD)

	// Copy and send packet
	radio.SetBufferBaseAddress(0, 0)
	radio.WriteBuffer(pkt)
	radio.SetTx(timeout)

	for {
		irq := radio.GetIrqStatus()
		radio.ClearIrqStatus(sx126x.SX126X_IRQ_ALL)

		if irq&sx126x.SX126X_IRQ_TX_DONE == sx126x.SX126X_IRQ_TX_DONE {
			return nil
		} else if irq&sx126x.SX126X_IRQ_TIMEOUT == sx126x.SX126X_IRQ_TIMEOUT {
			return errors.New("Tx timeout")
		} else if irq > 0 {
			println("IRQ value", irq)
			return errors.New("Unexpected IRQ value")
		}
		time.Sleep(time.Millisecond * 100) // Check status every 100ms
	}
}

// LoraRx
func LoraRx(radio sx126x.Device, timeoutSec uint8) ([]uint8, error) {

	radio.ClearIrqStatus(sx126x.SX126X_IRQ_ALL)
	timeout := uint32(float32(timeoutSec) * 1000000 / 15.625)

	// Wait RX
	machine.PA4.Set(true)
	machine.PB5.Set(false)

	// Define packet and modulation configuration (CRC OFF, IQ ON)
	radio.SetRfFrequency(868100000)
	radio.SetModulationParams(8, sx126x.SX126X_LORA_BW_125_0, sx126x.SX126X_LORA_CR_4_7, sx126x.SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF)
	radio.SetPacketParam(8, sx126x.SX126X_LORA_HEADER_EXPLICIT, sx126x.SX126X_LORA_CRC_OFF, 1, sx126x.SX126X_LORA_IQ_INVERTED)

	for { // We'll leave the loop either with RXDone or with Timeout
		radio.SetRx(timeout)
		irq := radio.GetIrqStatus()
		radio.ClearIrqStatus(sx126x.SX126X_IRQ_ALL)

		if irq&sx126x.SX126X_IRQ_RX_DONE == sx126x.SX126X_IRQ_RX_DONE {
			st := radio.GetRxBufferStatus()
			println("Rx Buffer Status", st[0], st[1])
			radio.SetBufferBaseAddress(0, st[1]) // Skip first byte
			pkt := radio.ReadBuffer(st[0] + 1)
			pkt = pkt[1:] // Skip first char ??? checkthat
			return pkt, nil
		} else if irq&sx126x.SX126X_IRQ_TIMEOUT == sx126x.SX126X_IRQ_TIMEOUT {
			return nil, errors.New("Rx timeout")
		} else if irq > 0 {
			println("IRQ value", irq)
			return nil, errors.New("RX:Unexpected IRQ value")
		}
		time.Sleep(time.Millisecond * 100) // Check status every 100ms
	}

}

func LoraJoin(radio sx126x.Device, lorastack *lorawan.LoraWanStack) error {
	// Generate Lora Join Request
	pktJoin, err := lorastack.GenerateJoinRequest()
	if err != nil {
		return err
	}
	// Sent Join Request
	err = LoraTx(radio, pktJoin)
	if err != nil {
		return err
	}
	// Wait for Join Accept for 15sec
	var pkt []uint8
	pkt, err = LoraRx(radio, 15)
	if err == nil {
		err = lorastack.DecodeJoinAccept(pkt)
		if err == nil {
			println("Lora: Now joined !")
			return nil
		}
	}
	return errors.New("Lora Join Accept timeout")

}

func LoraUplink(radio sx126x.Device, lorastack *lorawan.LoraWanStack, msg []uint8) error {
	// Generate Lora Join Request
	pkt, err := lorastack.GenMessage(0, msg)
	if err != nil {
		return err
	}
	// Sent Join Request
	println("Uplink:", BytesToHexString(pkt))
	println("Nwskey:", BytesToHexString(lorastack.Session.NwkSKey[:]))
	println("Appsley:", BytesToHexString(lorastack.Session.AppSKey[:]))
	println("Uplink:", BytesToHexString(pkt))
	err = LoraTx(radio, pkt)
	if err != nil {
		return err
	}
	return nil
}

/**************************
 *
 ***************************/
func main() {

	// Configure LED GPIO
	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})
	machine.LED.Set(true)
	time.Sleep(time.Millisecond * 250)
	machine.LED.Set(false)

	// Enable uart 9600 baud
	machine.UART0.Configure(machine.UARTConfig{TX: machine.UART_TX_PIN, RX: machine.UART_TX_PIN, BaudRate: 9600})

	// Init embedded radio module
	SubGhzInit()

	// Create the driver
	radio := sx126x.New(machine.SPI0)

	// Configure all Lora parameters
	configureLora(radio)

	lorastack := &lorawan.LoraWanStack{}
	// APP_KEY 2B7E151628AED2A6ABF7158809CF4F3C
	lorastack.Otaa.AppEUI = [8]byte{0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}
	lorastack.Otaa.DevEUI = [8]byte{0x00, 0x80, 0xE1, 0x01, 0x01, 0x01, 0x01, 0x01}
	lorastack.Otaa.AppKey = [16]byte{0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C}
	r, _ := radio.ReadRegister(sx126x.SX126X_REG_RANDOM_NUMBER_0, 2)
	lorastack.Otaa.DevNonce[0] = r[0]
	lorastack.Otaa.DevNonce[1] = r[1]

	// Join LORA
	err := LoraJoin(radio, lorastack)
	if err == nil {
		println("Lora Join Success")
		err = LoraUplink(radio, lorastack, []byte("TinyGo Lorawan"))
		if err != nil {
			println("Lora Uplink failed ", err)
		}
	} else {
		println("Lora Join Failed ", err)
	}

}
