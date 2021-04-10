package main

// This is the most minimal blinky example and should run almost everywhere.
//
import (
	"device/stm32"
	"fmt"
	"machine"
	"time"

	"tinygo.org/x/drivers/lora/sx126x"
)

func dbg(msg string) {
	println(msg)
}
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

// DS.SX1261-2.W.APP section 14.2 p99
func configureLora(lora sx126x.Device) {

	// Force Standby mode for configuration
	// Maybe we could replace with a check over current state
	// and fire an error if not in standby mode ?
	lora.SetStandby()

	lora.SetPacketType(sx126x.SX126X_PACKET_TYPE_LORA)
	lora.SetRfFrequency(868100000) // Needs to be done in FS/RX/TX mode ?

	lora.SetBufferBaseAddress(0, 0)

	// Here: Add fallback to STDBY_RC after RX/TX
	// See: https://github.com/jgromes/RadioLib/blob/86ca714d0017419aebc5bae5b4e8010e71e66874/src/modules/SX126x/SX126x.cpp#L1550

	// Here: Add CAD Configuration
	// See: https://github.com/jgromes/RadioLib/blob/86ca714d0017419aebc5bae5b4e8010e71e66874/src/modules/SX126x/SX126x.cpp#L1561

	dbg("Configure Dio Irq")
	lora.ClearIrqStatus(sx126x.SX126X_IRQ_ALL)
	lora.SetDioIrqParams(sx126x.SX126X_IRQ_NONE, sx126x.SX126X_IRQ_NONE, sx126x.SX126X_IRQ_NONE, sx126x.SX126X_IRQ_NONE)

	dbg("Calibrate all blocks")
	lora.CalibrateAll()
	time.Sleep(10 * time.Millisecond) //(See Man sec.13.1.12)

	// Here: Add optionnal TCXO configuration
	// https://github.com/jgromes/RadioLib/blob/86ca714d0017419aebc5bae5b4e8010e71e66874/src/modules/SX126x/SX126x.cpp#L110

	dbg("Limit current")
	lora.SetCurrentLimit(60)

	dbg("SetModulation")
	// Set modulation params
	// SF7 / 125 KHz / CR 4/7 / No Optimis
	lora.SetModulationParams(7, sx126x.SX126X_LORA_BW_125_0, sx126x.SX126X_LORA_CR_4_7, sx126x.SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF)

	dbg("SetPaConfig")
	lora.SetPaConfig(0x02, 0x02, 0x00, 0x01)

	dbg("SetTxParams") //Power and Ramping Time
	// Set output power and ramping time (0x16:HP22Db)
	lora.SetTxParams(0x16, sx126x.SX126X_PA_RAMP_200U) // CHECKME

	// Set Lora Sync Word
	lora.SetLoraPublicNetwork(true)

	// TODO: Set regulator DCDC/LDO

}

func xstatus(lora sx126x.Device) {
	dat := lora.GetStatus()
	chipmode := (dat & (0x7 << 4)) >> 4
	cmdstatus := (dat & (0x7 << 1)) >> 1
	println(">> STATUS : mode:", chipmode, " cmd:", cmdstatus, " irq:", dat, "deverr", lora.GetDeviceErrors())
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

func main() {

	// Configure LED GPIO
	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})
	machine.LED.Set(true)
	time.Sleep(time.Millisecond * 250)
	machine.LED.Set(false)

	// Test UART 9600 baud
	machine.UART0.Configure(machine.UARTConfig{TX: machine.UART_TX_PIN, RX: machine.UART_TX_PIN, BaudRate: 9600})
	println("Lora_tx test A")

	dbg("Init SubGhz device")
	SubGhzInit()

	lora := sx126x.New(machine.SPI0)

	dbg("lora:Get Status")
	//	xstatus(lora)

	// Sleep -> Standby
	lora.SetSleep()
	lora.SetStandby()

	configureLora(lora)

	dbg("LORA: Read Sync word MSB")
	addr := uint16(0x0740)
	rs, err := lora.ReadRegister(addr, 2)
	println("Lora Sync reg : ", fmt.Sprintf("%02x", rs[0]), fmt.Sprintf("%02x", rs[1]))
	checkErr(err)

	dbg("SetRfFrequency")
	lora.SetRfFrequency(868100000) // Needs to be done in FS/RX/TX mode ?

	// Configure RF GPIO
	// LoRa-E5 module ONLY transmits through RFO_HP:
	// Receive: PA4=1, PB5=0
	// Transmit(high output power, SMPS mode): PA4=0, PB5=1
	rfswitchPA4 := machine.PA4
	rfswitchPA4.Configure(machine.PinConfig{Mode: machine.PinOutput})
	rfswitchPB5 := machine.PB5
	rfswitchPB5.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Tx
	rfswitchPA4.Set(false)
	rfswitchPB5.Set(true)

	msg := []byte{0x00, 0xDC, 0x00, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70, 0x1E, 0x6F, 0xED, 0xF5, 0x7C, 0xEE, 0xAF, 0x00, 0x85, 0xCC, 0x58, 0x7F, 0xE9, 0x13}
	dbg("SetPacketParam")
	//	lora.SetPacketParam(10, 0x04, uint8(len(msg)), sx126x.SX126X_LORA_CRC_ON, sx126x.SX126X_LORA_IQ_STANDARD)
	lora.SetPacketParam(8, sx126x.SX126X_LORA_HEADER_EXPLICIT, uint8(len(msg)), sx126x.SX126X_LORA_CRC_ON, sx126x.SX126X_LORA_IQ_STANDARD)

	// Send 3 packet
	for u := 0; u < 2; u++ {
		// write the payload
		dbg("Write TX packet ")
		lora.WriteBuffer(msg)
		lora.SetTx(sx126x.SX126X_TX_TIMEOUT_NONE)
		xstatus(lora)
		time.Sleep(time.Second * 5)
	}

}
