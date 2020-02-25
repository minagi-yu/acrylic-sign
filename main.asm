;-------------------------------------------------------------------------
; Acrylic Sign
;
; Copylight (C) 2020 Minagi Yu
;-------------------------------------------------------------------------

; PB0: In,  Switch
; PB1: In,  Volume
; PB2: Out, WS2812B data pin
; PB3: In,  Reset

.include "tn10def.inc"

.def temp	= r16
.def zero	= r17

.def YL_	= r18

.def rlvl	= r19
.def glvl	= r20
.def blvl	= r21

.def tcchclr	= r22		; Timer counter for change color
.def rvchclr	= r23		; Reload value for change color timer

.def arg	= r24

.def stopflag	= r25

.equ SW_PORT	= PORTB
.equ SW_PUE	= PUEB
.equ SW_BIT	= 0
.equ SW_DDR	= DDRB
.equ VR_PORT	= PORTB
.equ VR_BIT	= 1
.equ VR_DDR	= DDRB
.equ LED_PORT	= PORTB
.equ LED_BIT	= 2
.equ LED_DDR	= DDRB

.equ FIFO_SIZE	= 16

;-------------------------------------------------------------------------
; Data Section
.dseg
.org SRAM_START

; FIFO for ADC result
fifo:
	.byte	FIFO_SIZE

;-------------------------------------------------------------------------
; Code Section
.cseg
.org 0
;
; Interrupt Vectors
;
vectors:
vector_reset:				; Reset Handler
	rjmp	reset
vector_int0:				; IRQ0 Handler
;	rjmp	isr_int0
	rjmp	reset
vector_pcint0:				; PCINT0 Handler
;	rjmp	isr_pcint0
	rjmp	reset
vector_tim0_capt:			; Timer0 Capture Handler
;	rjmp	isr_tim0_capt
	rjmp	reset
vector_tim0_ovf:			; Timer0 Overflow Handler
;	rjmp	isr_tim0_ovf
	rjmp	reset
vector_tim0_compa:			; Timer0 Compare A Handler
;	rjmp	isr_tim0_compa
	rjmp	reset
vector_tim0_compb:			; Timer0 Compare B Handler
;	rjmp	isr_tim0_compb
	rjmp	reset
vector_ana_comp:			; Analog Comparator Handler
;	rjmp	isr_ana_comp
	rjmp	reset
vector_wdt:				; Watchdog Interrupt Handler
;	rjmp	isr_wdt
	rjmp	reset
vector_vlm:				; Voltage Level Monitor Handler
;	rjmp	isr_vlm
	rjmp	reset
vector_adc:				; ADC Conversion Handler
;	rjmp	isr_adc
	rjmp	reset

;
; Startup routine
;
reset:
	clr	zero				; Clear zero register
	ldi	YH, high(SRAM_START)		; Clear RAM
	ldi	YL, low(SRAM_START)		; |
	ldi	temp, SRAM_SIZE			; |
	st	Y+, zero			; |
	dec	temp				; |
	brne	PC-2				; v
;	rjmp	main				; Go to main

;
; Main routine
;
main:
	ldi	temp, 0xd8			; Set F_CPU = 8MHz
	out	CCP, temp			; |
	out	CLKPSR, zero			; v

	sbi	LED_DDR, LED_BIT		; Set pin as output
	sbi	SW_PUE, SW_BIT			; Enable pull up

	sbi	ADMUX, 0			; Input channel ADC1 (PB1)
	ldi	temp, 0x86			; ADC enable, 8MHz / 64 = 125kHz
	out	ADCSRA, temp			; v
	sbi	DIDR0, 1			; ADC1 Digital input disable

	ldi	rvchclr, 127			;
	mov	tcchclr, rvchclr		;

	ldi	rlvl, 255			;
	ldi	glvl, 0				;
	ldi	blvl, 0				;

	ldi	ZH, high(r2y)			;
	ldi	ZL, low(r2y)			;

	ldi	YH, high(fifo)			;
	ldi	YL, low(fifo)			;

	clr	stopflag			;
	set					;

	sei					;

loop:						; Main Loop

	sbi	ADCSRA, ADSC			; Start ADC conversion
	sbis	ADCSRA, ADIF			; Wait ADC conversion
	rjmp	PC-1				; v
	sbi	ADCSRA, ADIF			; Clear ADC interrupt flag
	in	temp, ADCL			; Store the result to FIFO
	st	Y+, temp			; v

	mov	YL_, YL				; Save YL

	ldi	YL, low(fifo)			; for (i = 0; i < FIFO_SIZE; i++) {
	clr	XH				; 	X += fifo[i];
	clr	XL				; }
	ld	temp, Y+			; |
	add	XL, temp			; |
	adc	XH, zero			; |
	cpi	YL, low(fifo + FIFO_SIZE)	; |
	brne	PC-4				; v
.if FIFO_SIZE == 32
	swap	XH				; X /= 32
	swap	XL				; |
	lsr	XH				; |
	ror	XL				; |
	andi	XH, 0x78			; |
	andi	XL, 0x87			; |
	or		XL, XH			; v
.elseif	FIFO_SIZE == 16
	swap	XH				; X /= 16
	swap	XL				; |
	andi	XH, 0xf0			; |
	andi	XL, 0x0f			; |
	or		XL, XH			; v
.endif
	inc	XL				;
	mov	rvchclr, XL			;

	cpi	YL_, low(fifo + FIFO_SIZE)	;
	brne	PC+2				;
	ldi	YL_, low(fifo)			;
	mov	YL, YL_				; Restore YL

	in	temp, SW_BIT			; Load switch status
	brtc	PC+3				; if ((T == 1) &&
	sbrs	temp, 0				;	(temp & 1) == 0)
	inc	stopflag			; 		stopflag ^= 1;
	bst	temp, 0				; T = temp;
	sbrc	stopflag, 0			; if (stopflag)
	rjmp	loop				;	goto loop;


	dec	tcchclr				;
	brne	loop				;
	mov	tcchclr, rvchclr		; Reload timer counter value

	ijmp					; Jump to (Z)

r2y:						; Red to Yellow
	rcall	send_led			;
	inc	glvl				;
	cpi	glvl, 255			;
	brne	PC+3				;
	ldi	ZH, high(y2g)			;
	ldi	ZL, low(y2g)			;
	rjmp	loop				;
y2g:						; Yellow to Green
	rcall	send_led			;
	dec	rlvl				;
	brne	PC+3				;
	ldi	ZH, high(g2c)			;
	ldi	ZL, low(g2c)			;
	rjmp	loop				;
g2c:						; Green to Cyan
	rcall	send_led			;
	inc	blvl				;
	cpi	blvl, 255			;
	brne	PC+3				;
	ldi	ZH, high(c2b)			;
	ldi	ZL, low(c2b)			;
	rjmp	loop				;
c2b:						; Cyan to Blue
	rcall	send_led			;
	dec	glvl				;
	brne	PC+3				;
	ldi	ZH, high(b2m)			;
	ldi	ZL, low(b2m)			;
	rjmp	loop				;
b2m:						; Blue to Magenta
	rcall	send_led			;
	inc	rlvl				;
	cpi	rlvl, 255			;
	brne	PC+3				;
	ldi	ZH, high(m2r)			;
	ldi	ZL, low(m2r)			;
	rjmp	loop				;
m2r:						; Magenta to Red
	rcall	send_led			;
	dec	blvl				;
	brne	PC+3				;
	ldi	ZH, high(r2y)			;
	ldi	ZL, low(r2y)			;
	rjmp	loop

;
; send_led()
; Parameters:
;	r18: red data 
;	r19: green data
;	r20: blue data
; Returns:
;	none
;
send_led:
	cli					; Disable interrupt
	mov	arg, glvl			;
	rcall	ws2812b_send			;
	mov	arg, rlvl			;
	rcall	ws2812b_send			;
	mov	arg, blvl			;
	rcall	ws2812b_send			;
	sei					; Enable interrupt
	ret					;

;
; ws2812b_send()
; Parameters:
; 	r24: data to ws2812b
; Returns:
; 	none
;
ws2812b_send:
	ldi	r16, 8				; Loop counter
	sbi	LED_PORT, LED_BIT		; 10
	sbrs	r24, 7				; 1
	cbi	LED_PORT, LED_BIT		; 2
	nop					; 3
	nop					; 4
	cbi	LED_PORT, LED_BIT		; 5
	lsl	r24				; 6
	dec	r16				; 7
	brne	PC-8				; 9
	ret					;
