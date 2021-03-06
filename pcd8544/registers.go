package pcd8544

// Registers
const (
	POWERDOWN           = 0x04
	ENTRYMODE           = 0x02
	EXTENDEDINSTRUCTION = 0x01
	FUNCTIONSET         = 0x20

	DISPLAYCONTROL  = 0x08
	DISPLAYBLANK    = 0x0
	DISPLAYNORMAL   = 0x04
	DISPLAYALLON    = 0x01
	DISPLAYINVERTED = 0x05

	SETYADDR = 0x40
	SETXADDR = 0x80
	SETTEMP  = 0x04
	SETBIAS  = 0x10
	SETVOP   = 0x80
)
