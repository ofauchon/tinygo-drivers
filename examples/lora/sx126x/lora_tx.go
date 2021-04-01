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

func SubGhzInit() error {

	// TODO : NVIC, VOS

	// Enable APB3 Periph clock
	stm32.RCC.APB3ENR.SetBits(stm32.RCC_APB3ENR_SUBGHZSPIEN)
	time.Sleep(time.Millisecond) //Delay after RCC periph clock enable

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
	stm32.EXTI.IMR2.SetBits(1 << 12) // IM44

	// Enable radio busy wakeup from Standby for CPU
	// Is it required ?
	stm32.PWR.CR3.SetBits(stm32.PWR_CR3_EWRFBUSY)

	// Clear busy flag
	stm32.PWR.SCR.SetBits(stm32.PWR_SCR_CWRFBUSYF)

	//  SUBGHZSPI configuration
	stm32.SPI3.CR1.ClearBits(stm32.SPI_CR1_SPE)                                                   // Disable SPI
	stm32.SPI3.CR1.Set(stm32.SPI_CR1_MSTR | stm32.SPI_CR1_SSI | (0b010 << 3) | stm32.SPI_CR1_SSM) // Software Slave Management (NSS) + /8 prescaler
	//stm32.SPI3.CR1.Set(stm32.SPI_CR1_MSTR | stm32.SPI_CR1_SSI | (0b110 << 3) | stm32.SPI_CR1_SSM) // Software Slave Management (NSS) + /64 prescaler
	stm32.SPI3.CR2.Set(stm32.SPI_CR2_FRXTH | (0b111 << 8)) // FIFO Threshold and 8bit size
	stm32.SPI3.CR1.SetBits(stm32.SPI_CR1_SPE)              // Enable SPI

	println("CR1", stm32.SPI3.CR1.Get())
	println("CR2", stm32.SPI3.CR2.Get())
	return nil
}

// DS.SX1261-2.W.APP section 14.2 p99
func configureLora(lora sx126x.Device) {

	// Force Standby mode for configuration
	// Maybe we could replace with a check over current state
	// and fire an error if not in standby mode ?
	lora.SetStandby()

	println("SetBufferBaseAddress")
	lora.SetBufferBaseAddress(0, 0)

	println("SetPacketType")
	lora.SetPacketType(sx126x.SX126X_PACKET_TYPE_LORA)

	// Here: Add fallback to STDBY_RC after RX/TX
	// See: https://github.com/jgromes/RadioLib/blob/86ca714d0017419aebc5bae5b4e8010e71e66874/src/modules/SX126x/SX126x.cpp#L1550

	// Here: Add CAD Configuration
	// See: https://github.com/jgromes/RadioLib/blob/86ca714d0017419aebc5bae5b4e8010e71e66874/src/modules/SX126x/SX126x.cpp#L1561

	println("Configure Dio Irq")
	lora.ClearIrqStatus(sx126x.SX126X_IRQ_ALL)
	lora.SetDioIrqParams(sx126x.SX126X_IRQ_NONE, sx126x.SX126X_IRQ_NONE, sx126x.SX126X_IRQ_NONE, sx126x.SX126X_IRQ_NONE)

	println("Calibrate all blocks")
	lora.CalibrateAll()
	time.Sleep(10 * time.Millisecond) //(See Man sec.13.1.12)

	// Here: Add optionnal TCXO configuration
	// https://github.com/jgromes/RadioLib/blob/86ca714d0017419aebc5bae5b4e8010e71e66874/src/modules/SX126x/SX126x.cpp#L110

	println("Limit current")
	lora.SetCurrentLimit(60)

	println("SetModulation")
	// Set modulation params
	// SF7 / 125 KHz / CR 4/7 / No Optimis
	lora.SetModulationParams(7, sx126x.SX126X_LORA_BW_125_0, sx126x.SX126X_LORA_CR_4_7, sx126x.SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF)

	println("SetPaConfig")
	//	lora.SetPaConfig(0x04, 0x07, 0x00, 0x01)
	lora.SetPaConfig(0x02, 0x02, 0x00, 0x01)

	println("SetTxParams") //Power and Ramping Time
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
	println("mode:", chipmode, " cmd:", cmdstatus, " irq:", dat, "deverr", lora.GetDeviceErrors())
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

	// This test : clock, gpio
	for i := 0; i < 3; i++ {
		machine.LED.Set(true)
		time.Sleep(time.Millisecond * 200)
		machine.LED.Set(false)
		time.Sleep(time.Millisecond * 200)
	}

	// Test UART 9600 baud
	machine.UART0.Configure(machine.UARTConfig{TX: machine.UART_TX_PIN, RX: machine.UART_TX_PIN, BaudRate: 9600})
	for i := 0; i < 3; i++ {
		println("Lora_tx test")
	}

	// Init radio module
	println("SubGhzInit")
	SubGhzInit()

	//
	println("LORA: Init Driver")
	lora := sx126x.New(machine.SPI0)
	xstatus(lora)

	println("LORA: Read Sync word MSB")
	addr := uint16(0x0740)
	rs, err := lora.ReadRegister(addr, 2)
	for i := 0; i < len(rs); i++ {
		println("Sync reg ", fmt.Sprintf("%04x", int(addr)+i), fmt.Sprintf("%02x", rs[i]))

	}
	checkErr(err)

	/*


		machine.LED.Set(true)
		time.Sleep(time.Millisecond * 2000)
		machine.LED.Set(false)

		// Configure RF GPIO
		rfswitchPB5 := machine.PB5
		rfswitchPB5.Configure(machine.PinConfig{Mode: machine.PinOutput})

		rfswitchPA4 := machine.PA4
		rfswitchPA4.Configure(machine.PinConfig{Mode: machine.PinOutput})


		println("STM32WL Radio Init Example")


			// Switch Radio to Sleep, then Standby
			println("Go Sleep")
			//lora.SetSleep()
			println("Go Standby")
			//lora.SetStandby()
			println("* Now in Standby")

			//xstatus(lora)

		println("Read SyncWord Registers")



			xstatus(lora)

			// Sets Lora configuration
			configureLora(lora)

			//msg := []byte("Hello TinyGo!")
			msg := []byte{0x00, 0xDC, 0x00, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70, 0x1E, 0x6F, 0xED, 0xF5, 0x7C, 0xEE, 0xAF, 0x00, 0x85, 0xCC, 0x58, 0x7F, 0xE9, 0x13}

			// Switch to RX
			lora.SetRx(sx126x.SX126X_RX_TIMEOUT_NONE)
			time.Sleep(100 * time.Millisecond)

			println("SetRfFrequency")
			lora.SetRfFrequency(868000000) // Needs to be done in FS/RX/TX mode ?

			println("SetPacketParam")
			lora.SetPacketParam(10, 0x04, uint8(len(msg)), sx126x.SX126X_LORA_CRC_ON, sx126x.SX126X_LORA_IQ_STANDARD)

			// Show current status
			xstatus(lora)


			for {

				// ------------------------------> TX
				// write the payload
				println("Write TX packet")
				lora.WriteBuffer(msg)

				// Clear IRQ Flag
				lora.ClearIrqStatus(0x0)

				// Set RF Switch to TX
				rfswitchPA4.Set(false)
				rfswitchPB5.Set(true)

				// Set TX
				lora.SetTx(sx126x.SX126X_TX_TIMEOUT_NONE)

				println("Wait 10s")

				xstatus(lora)
				time.Sleep(10000 * time.Millisecond)

			}

	*/

	// Set RF Switch to TX
	//rfswitchPA4.Set(false)
	//rfswitchPB5.Set(true)

	//lora.SetTxContinuousWave()

}
