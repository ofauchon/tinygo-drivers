package lora

import "errors"

// Config holds the LoRa configuration parameters
type Config struct {
	Freq           uint32 // Frequency
	Cr             uint8  // Coding Rate
	Sf             uint8  // Spread Factor
	Bw             uint8  // Bandwidth
	Ldr            uint8  // Low Data Rate
	Preamble       uint16 // PreambleLength
	SyncWord       uint16 // Sync Word
	HeaderType     uint8  // Header : Implicit/explicit
	Crc            uint8  // CRC : Yes/No
	Iq             uint8  // iq : Standard/inverted
	LoraTxPowerDBm int8   // Tx power in Dbm
}

var (
	ErrUndefinedLoraConf = errors.New("Undefined Lora configuration")
)

const (
	SpreadingFactor5  = 0x05
	SpreadingFactor6  = 0x06
	SpreadingFactor7  = 0x07
	SpreadingFactor8  = 0x08
	SpreadingFactor9  = 0x09
	SpreadingFactor10 = 0x0A
	SpreadingFactor11 = 0x0B
	SpreadingFactor12 = 0x0C
)

const (
	CodingRate4_5 = 0x01 //  7     0     LoRa coding rate: 4/5
	CodingRate4_6 = 0x02 //  7     0                       4/6
	CodingRate4_7 = 0x03 //  7     0                       4/7
	CodingRate4_8 = 0x04 //  7     0                       4/8
)

const (
	HeaderExplicit = 0x00 //  7     0     LoRa header mode: explicit
	HeaderImplicit = 0x01 //  7     0                       implicit
)

const (
	LowDataRateOptimizeOff = 0x00 //  7     0     LoRa low data rate optimization: disabled
	LowDataRateOptimizeOn  = 0x01 //  7     0                                      enabled
)

const (
	CRCOff = 0x00 //  7     0     LoRa CRC mode: disabled
	CRCOn  = 0x01 //  7     0                    enabled
)

const (
	IQStandard = 0x00 //  7     0     LoRa IQ setup: standard
	IQInverted = 0x01 //  7     0                    inverted
)

const (
	Bandwidth_7_8   = iota // 7.8 kHz
	Bandwidth_10_4         // 10.4 kHz
	Bandwidth_15_6         // 15.6 kHz
	Bandwidth_20_8         // 20.8 kHz
	Bandwidth_31_25        // 31.25 kHz
	Bandwidth_41_7         // 41.7 kHz
	Bandwidth_62_5         // 62.5 kHz
	Bandwidth_125_0        // 125.0 kHz
	Bandwidth_250_0        // 250.0 kHz
	Bandwidth_500_0        // 500.0 kHz
)

const (
	SyncPublic = iota
	SyncPrivate
)

const (
	MHz_868_1 = 868100000
	MHz_868_5 = 868500000
	MHz_916_8 = 916800000
	MHz_923_3 = 923300000
)

func SFToString(sf uint8) string {
	switch sf {
	case SpreadingFactor5:
		return "SF5"
	case SpreadingFactor6:
		return "SF6"
	case SpreadingFactor7:
		return "SF7"
	case SpreadingFactor8:
		return "SF8"
	case SpreadingFactor9:
		return "SF9"
	case SpreadingFactor10:
		return "SF10"
	case SpreadingFactor11:
		return "SF11"
	case SpreadingFactor12:
		return "SF12"
	}
	return "N/A"
}

func CRToString(sf uint8) string {
	switch sf {
	case CodingRate4_8:
		return "4/8"
	case CodingRate4_7:
		return "4/7"
	case CodingRate4_6:
		return "4/6"
	case CodingRate4_5:
		return "4/5"
	}
	return "N/A"
}

func BWToString(sf uint8) string {
	switch sf {
	case Bandwidth_7_8:
		return "BW7"
	case Bandwidth_10_4:
		return "BW10"
	case Bandwidth_15_6:
		return "BW15"
	case Bandwidth_20_8:
		return "BW20"
	case Bandwidth_31_25:
		return "BW31"
	case Bandwidth_41_7:
		return "BW41"
	case Bandwidth_62_5:
		return "BW62"
	case Bandwidth_125_0:
		return "BW125"
	case Bandwidth_250_0:
		return "BW250"
	case Bandwidth_500_0:
		return "BW500"
	}
	return "N/A"
}
