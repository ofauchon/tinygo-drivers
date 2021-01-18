package sx127x

const (
	// registers
	REG_FIFO                 = 0x00
	REG_OP_MODE              = 0x01
	REG_FRF_MSB              = 0x06
	REG_FRF_MID              = 0x07
	REG_FRF_LSB              = 0x08
	REG_PA_CONFIG            = 0x09
	REG_PA_RAMP              = 0x0a
	REG_OCP                  = 0x0b
	REG_LNA                  = 0x0c
	REG_FIFO_ADDR_PTR        = 0x0d
	REG_FIFO_TX_BASE_ADDR    = 0x0e
	REG_FIFO_RX_BASE_ADDR    = 0x0f
	REG_FIFO_RX_CURRENT_ADDR = 0x10
	REG_IRQ_FLAGS            = 0x12
	REG_IRQ_FLAGS_MASK       = 0x11
	REG_RX_NB_BYTES          = 0x13
	REG_PKT_SNR_VALUE        = 0x19
	REG_PKT_RSSI_VALUE       = 0x1a
	REG_RSSI_VALUE           = 0x1b
	REG_MODEM_CONFIG_1       = 0x1d
	REG_MODEM_CONFIG_2       = 0x1e
	REG_SYMB_TIMEOUT_LSB     = 0x1f
	REG_PREAMBLE_MSB         = 0x20
	REG_PREAMBLE_LSB         = 0x21
	REG_PAYLOAD_LENGTH       = 0x22
	REG_MODEM_CONFIG_3       = 0x26
	REG_FREQ_ERROR_MSB       = 0x28
	REG_FREQ_ERROR_MID       = 0x29
	REG_FREQ_ERROR_LSB       = 0x2a
	REG_RSSI_WIDEBAND        = 0x2c
	REG_DETECTION_OPTIMIZE   = 0x31
	REG_INVERTIQ             = 0x33
	REG_DETECTION_THRESHOLD  = 0x37
	REG_SYNC_WORD            = 0x39
	REG_INVERTIQ2            = 0x3b
	REG_DIO_MAPPING_1        = 0x40
	REG_VERSION              = 0x42
	REG_PA_DAC               = 0x4d

	// Constants for radio registers
	OPMODE_LORA      = uint8(0x80)
	OPMODE_MASK      = uint8(0x07)
	OPMODE_SLEEP     = uint8(0x00)
	OPMODE_STANDBY   = uint8(0x01)
	OPMODE_FSTX      = uint8(0x02)
	OPMODE_TX        = uint8(0x03)
	OPMODE_FSRX      = uint8(0x04)
	OPMODE_RX        = uint8(0x05)
	OPMODE_RX_SINGLE = uint8(0x06)
	OPMODE_CAD       = uint8(0x07)

	// PA config
	PA_BOOST = 0x80

	// IRQ masks
	IRQ_TX_DONE_MASK           = uint8(0x08)
	IRQ_PAYLOAD_CRC_ERROR_MASK = uint8(0x20)
	IRQ_RX_DONE_MASK           = uint8(0x40)
	IRQ_RX_TMOUT_MASK          = uint8(0x64)

	MAX_PKT_LENGTH = uint8(255)
)
