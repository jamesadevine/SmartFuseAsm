    LIST P=16F1788
    #include "p16F1788.inc"

nanoCounter     EQU	0x20
milliCounter    EQU	0x21
secCounter		EQU	0x22
nano8Counter	EQU	0x23

;used for storing ADC result
tempStoreL		EQU	0x24
tempStoreH		EQU	0x25

;PACKET BEGIN
;simply describes the structure of the payload
tempIDH			EQU	0x26
tempSampleL		EQU	0x27
startBit		EQU	0x28
deviceIDL		EQU	0x29	;8 bits of 12 bit ID
dIDsampleL		EQU	0x2A	;4 bits of deviceID	;lower 4bits of Sample Count
sampleH			EQU	0x2B	;remaining 8 bits of sample Count
totalL			EQU	0x2C	;first 8 bits of 24 bit total
totalM			EQU	0x2D	;second 8 bits of 24 bit total
totalH			EQU	0x2E	;third 8 bits of 24 bit total
checkSum		EQU	0x2F	;8 bit checksum which is all 6 bytes values summed.

smplOvrfFl		EQU	0x30	 ;sample overflow flag
smplOvrfMx		EQU	0x31	;sample overflow max
smplOvrfCr		EQU	0x32	;sample overflow current how many times the register has overflowed
smplOvrfMa		EQU	0x75	;sample overflow match


;may not be used in future
commReg			EQU	0x72

;bit mask vars
moveAddr		EQU 0x73
storeAddr		EQU	0x74
bitCount		EQU	0x75
bitCountMax		EQU	0x76
twBitMsk		EQU	0x77
tempResult		EQU	0x78

;matchVal used as a comparison register
matchVal		EQU	0x79
hammingReg		EQU	0x7A
nextCount		EQU 0x7B

	goto 	INIT ;jump over the hamming lookup table

HAMMING:
	incf 	nextCount,F
	movf 	hammingReg,W
	addwf	PCL,F
	dt	b'00010101',b'00000010',b'01001001',b'01011110',b'01100100',b'01110011',b'00111000',b'00101111',b'11010000',b'11000111',b'10001100',b'10011011',b'10100001',b'10110110',b'11111101',0xEA


INIT:
	incf 	nextCount,F ;randomish increment of nextCount!
	banksel	smplOvrfCr
	clrf	smplOvrfCr
	clrf	smplOvrfFl
	banksel	OSCCON
	movlw	b'01110000'
	movwf	OSCCON
	;set id
	banksel	deviceIDL
	movlw	0x0A
	movwf	deviceIDL ; device id is one
	clrf	tempIDH
	;set all TRISA to be output
	banksel TRISA
	clrf	TRISA
	;set up RB3 to be analog input
	banksel TRISB
	bcf		TRISB,1
	bsf		TRISB,6

	;set all TRISC to be output
	banksel TRISC
	clrf	TRISC

	;configure all A pins as digital
	banksel ANSELA
	clrf	ANSELA
	;configure all B pins as analogue
	banksel ANSELB
	movlw	b'10111101'
	;configure all C pins as digital
	banksel ANSELC
	clrf	ANSELC
	;configure ADCON1
	banksel ADCON1
	movlw	b'0010000'
	;configure ADCON2
	banksel	ADCON2
	movlw	b'00111111'
	movwf	ADCON2
	;SERIAL COMM SETUP
	banksel	TXSTA
	movlw	b'10100000'
	movwf	TXSTA
	banksel	RCSTA
	movlw	b'10010000'
	movwf	RCSTA
	banksel	BAUDCON
	;movlw	b'00010000' ;inverted
	movlw	b'00000000' ;non inverted
	movwf	BAUDCON
	banksel	SPBRG
	movlw	d'12'
	movwf	SPBRG
	;test porta
	banksel PORTA
	movlw 	0xFF
	movwf	PORTA
	movwf 	PORTC

	movlw	b'00100101'	;set the ADCON0 channel
	banksel	ADCON0
	movwf	ADCON0		;turn on
	nop					;acquire...

	banksel smplOvrfMx
	movlw 	0x80
	movwf	smplOvrfMx	;preload 80 into the max for the init

	;reset porta and clear smplOverfCr
	banksel	PORTA
	clrf	PORTA
	;show the device is on
	bsf 	PORTA,0



MAIN:
	call	ADCONVERT
	call	TEST_TRANS
	goto	MAIN


TEST_TRANS:
	incf 	nextCount,F 	;randomish increment of nextCount!
	banksel	tempSampleL
	btfss	smplOvrfFl,0
	return
	;check if the current count matches the label
	movf	tempSampleL,W
	sublw 	smplOvrfMa
	movf	nextCount,W		;get the random variable
	movwf	smplOvrfMx 		;set the new max variable to the random variable
	btfsc	STATUS,Z
	call	TRANSMIT
	return
;--------------------------ADC-----------------------
;RB3 from analogue to digital, stores the result
;in tempStoreL and tempStoreH
ADCONVERT:
	;begin adc conversion
	banksel	ADCON0
	bsf		ADCON0,1 	;start adc
	btfsc	ADCON0,1	;check go/done
	goto	$-1			;if we aren't done - go back 1
	;adc finished - store result
	banksel	tempStoreH
	clrf	tempStoreH
	clrf	tempStoreL

	incf 	nextCount,F ;randomish increment of nextCount!

	;upper 4 bits of ADRESL -> tempStoreL
	movlw	tempStoreL
	movwf	storeAddr
	movlw	0x00
	movwf	bitCountMax
	movlw	0x10
	movwf	bitCount
	movlw	0x01
	movwf	twBitMsk
	call	GT_FRM_ADRESL

	;lower 4 bits of ADRESH -> tempStoreL
	movlw	tempStoreL
	movwf	storeAddr
	movlw	0x10
	movwf	bitCountMax
	movlw	0x01
	movwf	bitCount
	movlw	0x10
	movwf	twBitMsk
	call	GT_FRM_ADRESH

	;upper 4 bits of ADRESH -> tempStoreH
	movlw	tempStoreH
	movwf	storeAddr
	movlw	0x00
	movwf	bitCountMax
	movlw	0x10
	movwf	bitCount
	movlw	0x01
	movwf	twBitMsk
	call	GT_FRM_ADRESH

	incf 	nextCount,F 	;randomish increment of nextCount!

	call	INCNOSAMPLES	;increment the number of sample
	call	ADDADCRESULT	;add the new sample to total
	return

;Parameters:
;moveAddr - the address to move data from
;storeAddr	- the address of the storage register
;bitCount - where the AND should start
;twBitMsk -	the bit to be set in the storage register
;bitCountMax - the maximum bit bitCount should go to

GT_FRM_ADRESL:
	incf 	nextCount,F
	banksel	ADRESL
	movf	ADRESL,W		;move value to W
	andwf	bitCount,W	;check if the current bit is set
	btfss	STATUS,Z	;if the result is 1
	call	STOREONEDYN 	;store one in storeAddr
	banksel	tempStoreL
	bcf		STATUS,C 				;clear carry flag
	rlf		twBitMsk,F 	;shift bit mask left
	bcf		STATUS,C
	rlf		bitCount,F 	;shift bit count left
	movf	bitCount,W	;load working register
	bcf		STATUS,C
	subwf	bitCountMax,W
	btfss	STATUS,Z	;check if zero
	goto	GT_FRM_ADRESL;if not zero go round again
	return

;Parameters:
;moveAddr - the address to move data from
;storeAddr	- the address of the storage register
;bitCount - where the AND should start
;twBitMsk -	the bit to be set in the storage register
;bitCountMax - the maximum bit bitCount should go to

GT_FRM_ADRESH:
	incf 	nextCount,F
	banksel	ADRESH
	movf	ADRESH,W
	andwf	bitCount,W	;check if the current bit is set
	btfss	STATUS,Z	;if the result is 1
	call	STOREONEDYN 	;store one in storeAddr
	banksel	tempStoreL
	bcf		STATUS,C 				;clear carry flag
	rlf		twBitMsk,F 	;shift bit mask left
	bcf		STATUS,C
	rlf		bitCount,F 	;shift bit count left
	movf	bitCount,W	;load working register
	bcf		STATUS,C
	subwf	bitCountMax,W
	btfss	STATUS,Z	;check if zero
	goto	GT_FRM_ADRESH;if not zero go round again
	return

;----INCREMENT NO OF SAMPLES----
;increments the number of samples made by the PIC
INCNOSAMPLES:
	incf 	nextCount,F
	banksel	tempSampleL
	incf 	tempSampleL,F
	btfsc	STATUS,Z	;check if we have overflowed
	call	ADDTOSAMPLEH	;if we have, add one to sampleH
	return				;return

ADDTOSAMPLEH:
	banksel	sampleH
	incf 	sampleH,F
	movf	sampleH,W		;move it back
	xorlw	0xf0
	btfss	STATUS,Z	;check if we have reach the maximum held in 12 bits
	call	ADDANDRESET	;if we have reset
	return				;otherwise return

;when we have stored too many sample we need to reset
;clear all registers
ADDANDRESET:
	incf 	nextCount,F
	banksel	dIDsampleL
	incf 	smplOvrfCr,F
	movf	smplOvrfCr,W
	subwf	smplOvrfMx,W
	btfsc	STATUS,Z
	comf	smplOvrfFl,F
RESETVARS:
	clrf	dIDsampleL
	clrf	tempSampleL
	clrf	sampleH
	clrf	totalL
	clrf	totalM
	clrf	totalH
	return

;----ADDADCRESULTTOSUM----
;adds the adcresult to the sum registers
ADDADCRESULT:
	;add upper part of adcresult first
	movlw	totalL		;load the address lower 8 bits of the total
	movwf	FSR0L		;move that address to FSR
	call 	ADDLOWER	;add tempstoreL to totalL
	movlw	totalM		;load the address of the middle 8 bits of total
	movwf	FSR0L		;move that address to FSR
	call 	ADDHIGHER	;add tempstoreH to totalM
	return				;return

ADDLOWER:
	banksel	tempStoreL
	moviw	[INDF0]
	addwf	tempStoreL,W		;add working register and the address pointed to by INDF0
	movwi	[INDF0]
	btfsc	STATUS,C	;check if the carry flag is set(overflowed)
	call	INCAVGREG	;if we have overflowed increment the register one space above this register
	return

ADDHIGHER:
	banksel	tempStoreH
	moviw	[INDF0]
	addwf	tempStoreH,W		;add working register and the address pointed to by INDF0
	movwi	[INDF0]
	btfsc	STATUS,C	;check if z flag is set (overflow)
	call	INCAVGREG	;if we have overflowed increment the register one space above this register
	return				;return

INCAVGREG:
	movf	FSR0L,W		;move the address stored in FSR0L
	sublw	totalH+1	;check if we are at the max register
	btfsc	STATUS,Z 	;check if we have overflowed on top register
	call	RESETVARS	;if we have reset
	incf	FSR0L,F		;increment the address
	moviw	[INDF0]		;load the new register contents
	addlw	0x01		;add one to that register
	movwi	[INDF0]		;move from working to INDF0
	btfsc	STATUS,Z	;check if we have overflowed
	goto	INCAVGREG	;if we have go around again
	return				;otherwise return

;-----------------------CHECKSUM---------------------
;add the register that make up the packet ignoring carry
;and overflow.
CHKSUM:
	movf	FSR0L,W		;move the address held in FSR0L to W
	clrc				;clear carry
	sublw	checkSum	;subtract the address of checkSum
	btfsc	STATUS,Z	;check if the result is 0
	return				;if it is then return
	moviw	INDF0++		;move the contents of INDF0 to W
	clrc				;clear carry? - WHY?
	addwf	checkSum,F	;add the value of W to the checksum register
	goto	CHKSUM		;repeat



COMMSTEST:
	banksel	deviceIDL
	movlw	0xAA
	movwf	deviceIDL
	movwf	dIDsampleL
	movwf	checkSum
	movlw	0xAA
	movwf	totalL
	movlw	0xAA
	movwf	totalM

	movwf	tempIDH
	movwf	sampleH
	movwf	totalH

	return

SETUPFORCOMMS:
	;shift sampleH left 4 times to make room for top four of tempsampleL
	banksel	sampleH
	swapf	sampleH,F
	;move lower four bits of tempIDH to dIDSampleL
	movlw	tempIDH
	movwf	moveAddr
	movlw	dIDsampleL
	movwf	storeAddr
	movlw	0x10
	movwf	bitCountMax
	movlw	0x01
	movwf	bitCount
	movwf	twBitMsk
	call	STORE
	;use previous twBitMsk and store address
	;reset bitCount
	;move lower four bits of tempSampleL to dIDSampleL
	movlw	tempSampleL
	movwf	moveAddr
	clrf	bitCount
	bsf		bitCount,0
	movlw	0x10
	movwf	bitCountMax
	call	STORE
	;use previous bit count
	;set new bitcount max
	;reset twBitMsk
	;store in sampleH
	movlw	sampleH
	movwf	storeAddr
	movlw	0x00
	movwf	bitCountMax
	clrf	twBitMsk
	bsf		twBitMsk,0
	call	STORE

	return


;Parameters:
;moveAddr - the address to move data from
;storeAddr	- the address of the storage register
;bitCount - where the AND should start
;twBitMsk -	the bit to be set in the storage register
;bitCountMax - the maximum bit bitCount should go to
STORE:
	movf	moveAddr,W	;load the address of the register we are comparing
	movwf	FSR0
	moviw	[INDF0]		;move value to W
	andwf	bitCount,W	;check if the current bit is set
	btfss	STATUS,Z	;if the result is 1
	call	STOREONEDYN 	;store one in storeAddr
	bcf		STATUS,C 				;clear carry flag
	rlf		twBitMsk,F 	;shift bit mask left
	bcf		STATUS,C
	rlf		bitCount,F 	;shift bit count left
	movf	bitCount,W	;load working register
	bcf		STATUS,C
	subwf	bitCountMax,W
	btfss	STATUS,Z	;check if zero
	goto	STORE;if not zero go round again
	return				;o

STOREONEDYN:
	incf 	nextCount,F ;randomish increment of nextCount!
	banksel	storeAddr
	movf	storeAddr,W
	movwf	FSR0
	moviw	[INDF0]
	xorwf	twBitMsk,W		;xor
	movwi	[INDF0]
	return

STOREZERODYN:
	incf 	nextCount,F ;randomish increment of nextCount!
	banksel	storeAddr
	movf	storeAddr,W
	movwf	FSR0
	moviw	[INDF0]
	bcf		W,twBitMsk		;xor
	movwi	[INDF0]		;store the result in INDF0
	return


;FRAMING!!!
SERIALCOMMS:
	banksel	startBit
	movf	startBit,W
	banksel	TXREG
	movwf	TXREG
	banksel	TXSTA
	btfss 	TXSTA,TRMT
	goto 	$-1
SERIALLOOP:
	moviw	INDF0++
	movwf	tempResult
	;trans lower 4 bits
	andlw	0x0f
	movwf	hammingReg
	call	HAMMING
	banksel	TXREG
	movwf	TXREG
	banksel	TXSTA
	btfss 	TXSTA,TRMT
	goto 	$-1
	;trans upper 4 bits
	swapf	tempResult,F
	movf	tempResult,W
	andlw	0x0F

	;get hamming value
	movwf	hammingReg
	call	HAMMING
	;transmit!!
	banksel	TXREG
	movwf	TXREG
	banksel	TXSTA
	btfss 	TXSTA,TRMT
	goto 	$-1
	movf	FSR0,W
	sublw	checkSum+1
	btfss	STATUS,Z
	goto	SERIALLOOP
	return

;----------------TRUE TIMER INTERRUPT---------------------

TRANSMIT:
	banksel PORTA
	bsf 	PORTA,7		;visualize communication
	call	SETUPFORCOMMS
	movlw	deviceIDL	;move the address of the deviceIDL register
	movwf	FSR0L		;move the address into FSR0
	banksel checkSum
	clrf	checkSum	;clear checkSum
	call	CHKSUM		;call checksu
	;call	COMMSTEST
	banksel	startBit
	movlw 	0x55
	movwf 	startBit
	movlw	startBit+1	;move the address of the deviceIDL register
	movwf	FSR0L		;move the address into FSR0
	call	SERIALCOMMS
	call	RESETVARS
	banksel	smplOvrfFl
	clrf	smplOvrfFl
	clrf	smplOvrfCr
	banksel PORTA
	bcf 	PORTA,7		;communication finished!
	return				;return from interrupt
	end


;----------------------------------------------------------------------------------------

;THE FOLLOWING IS NO LONGER USED.


;-----------------------DELAY CALLS---------------------
;sets registers used for delaying the processer
;should be renamed to DELAYMS
DELAYMS:
	banksel	secCounter
	movlw	d'100'
	movwf	secCounter
    goto	DELAYLOOP

;second loop which counts for 1,000,000 *(8*0.125) ns = 1 ms
DELAYLOOP:
	banksel	milliCounter
	movlw	d'100'
	movwf	milliCounter
	movwf	nanoCounter
	decfsz	secCounter,F
	goto	DELAYMILLI
	return

;milli loop which counts for 100 * 100 ns
DELAYMILLI:
	movlw	d'100'
	movwf	nanoCounter
    decfsz  milliCounter,F
    goto    DELAYNANO
	goto	DELAYLOOP
;nano loop which counts for 100 ns
DELAYNANO:
	decfsz  nanoCounter,F
    goto    DELAYNANO
    goto	DELAYLOOP

;delays 8 * 0.125ns to = 1ns
DELAYNANO8:
	decfsz  nano8Counter,F
    goto    DELAYNANO8
    goto	DELAYLOOP



;-----------------------COMM REG ON PORTB------------------
;set commReg before calling!!
COMMREG:
	clrf	bitCount
	bsf		bitCount,0	;load one into bitcount

;determine if current bit is a one or zero
COMMREGLOOP:
	movf	commReg,W	;load the register we are communicating into W
	clrc
	andwf	bitCount,W	;check if the current bit is set
	btfss	STATUS,Z	;check if the result is zero
	call	COMMONE		;if not zero - communicate a one
	btfsc	STATUS,Z
	call	COMMZERO	;if zer0 communicate 0
	clrc			 	;clear carry flag
	rlf		bitCount,F 	;shift bit count left
	movf	bitCount,W	;load bitCount into working register
	clrc
	xorwf	matchVal,W	;subtract the matchVal register
	btfss	STATUS,Z	;check if the result is 0
	goto	COMMREGLOOP	;if it isn't go around again
	return				;otherwise return

COMMONE:
	movlw	b'00000010'	;set pin RB1 high
	banksel	PORTB
	btfsc	PORTB,1
	clrf	PORTB		;clear PORTB
	movwf	PORTB		;move onto PORTB
	call	DELAYMS	;delay for a sec
	return				;continue

COMMZERO:
	banksel	PORTB		;select bank containing PORTB
	movlw	b'00000000'	;set pin RB1 low
	btfss	PORTB,1
	bsf		PORTB,1		;temporarily set RB1 high
	movwf	PORTB		;move 0 onto portb
	call	DELAYMS		;delay
	return				;continue

;-----------------------TMR INTERRUPT---------------------
;called when an interrupt is generated.
TIMRINTERRUPT:
	movlw	deviceIDL	;move the address of the deviceIDL register
	movwf	FSR0L		;move the address into FSR0
	;clrf	checkSum	;clear checkSum
	;call	CHKSUM		;call checksum
	movlw	deviceIDL	;move the address of the deviceIDL register
	movwf	FSR0L		;move that value into the FSR0 register

	clrf	matchVal	;transmit deviceIDL
	moviw	INDF0++
	movwf	commReg
	clrf	PORTB
	call	COMMONE

	call	COMMREG

	clrf	matchVal	;transmit deviceIDH
	bsf		matchVal,4	;get only first 4 bits of deviceIDH
	moviw	INDF0++
	movwf	commReg
	call	COMMREG

	clrf	matchVal	;transmit sampleL
	moviw	INDF0++
	movwf	commReg
	call	COMMREG

	clrf	matchVal	;transmit sampleH
	bsf		matchVal,4	;get only first 4 bits of sampleH
	moviw	INDF0++
	movwf	commReg
	call	COMMREG

	clrf	matchVal	;transmit totalL
	moviw	INDF0++
	movwf	commReg
	call	COMMREG

	clrf	matchVal	;transmit totalM
	moviw	INDF0++
	movwf	commReg
	call	COMMREG

	clrf	matchVal	;transmit totalH
	moviw	INDF0++
	movwf	commReg
	call	COMMREG

	clrf	matchVal	;transmit checkSum
	moviw	INDF0++
	movwf	commReg
	call	COMMREG
	clrf	PORTB
	return				;return




;-----------------------DISPLAY VAL ON LED------------------
;display adcResult in binary on LEDs
DISPLAY:
	banksel PORTA
	movf	adcResultH,W
	movwf	PORTA
	movf	adcResultL,W
	movwf	PORTC
	return
