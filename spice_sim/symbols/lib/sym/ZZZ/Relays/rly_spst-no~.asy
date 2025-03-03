Version 4
SymbolType CELL
LINE Normal 16 0 32 0
LINE Normal 16 -16 16 0
LINE Normal 13 -10 16 -16
LINE Normal 19 -10 13 -10
LINE Normal 16 -16 19 -10
LINE Normal -11 -16 -32 -16
LINE Normal 22 -33 -6 -18
LINE Normal 23 -31 22 -33
LINE Normal -5 -16 23 -31
LINE Normal -6 -18 -5 -16
LINE Normal 8 48 -32 48
LINE Normal -8 80 -32 80
LINE Normal 8 69 -8 69
LINE Normal -15 40 -23 40
LINE Normal -19 44 -19 36
LINE Normal -15 86 -23 86
RECTANGLE Normal 32 112 -32 -64
RECTANGLE Normal 8 96 -8 32
CIRCLE Normal -5 -13 -11 -19
CIRCLE Normal -6 -14 -10 -18
CIRCLE Normal -7 -15 -9 -17
ARC Normal 2 69 14 80 8 80 8 69
ARC Normal -2 69 -14 59 -8 59 -8 69
ARC Normal 2 48 14 59 8 59 8 48
WINDOW 0 0 -80 Center 2
SYMATTR Prefix X
SYMATTR Description SPST-NO Relay
SYMATTR SpiceLine VNOM=7 RCOIL=500 VON=0.8 VOFF=0.4 RON=1m ROFF=1G COFF=1f
SYMATTR SpiceLine2 LSER=1f CCOUP=1f  LCOIL=10m CCOIL=1f
SYMATTR ModelFile relay.lib
SYMATTR Value rly_spst-no~
PIN -32 48 NONE 8
PINATTR PinName a
PINATTR SpiceOrder 1
PIN -32 80 NONE 8
PINATTR PinName b
PINATTR SpiceOrder 2
PIN -32 -16 NONE 8
PINATTR PinName c
PINATTR SpiceOrder 3
PIN 32 0 NONE 8
PINATTR PinName d
PINATTR SpiceOrder 4
