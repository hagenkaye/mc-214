;;	.device "ATtiny214" 
	.cseg
	.org	0x00
	
	
;; registers used by the program	
	.def    r_acc = r0
	.def    r_acc_b = r1
	.def    r_zero = r2             ; always zero
	.def    r_unprotect = r3        ; set to unprotect signature
	.def    r_fwd_pwm = r4          ; signed 16bit fwd pwm value
	.def    r_fwd_pwm_l = r4
	.def    r_fwd_pwm_h = r5
	.def    r_cw_pwm = r6           ; signed 16bit cw pwm value
	.def    r_cw_pwm_l = r6
	.def    r_cw_pwm_h = r7
	
	.def    r_tmp = r16             ; temporary register
	.def    r_tmp_l = r16
	.def    r_tmp_h = r17
	.def    r_left_pwm = r18         ; left motor pwm value
	.def    r_left_pwm_l = r18
	.def    r_left_pwm_h = r19
	.def    r_right_pwm = r20        ; right motor pwm value
	.def    r_right_pwm_l = r20
	.def    r_right_pwm_h = r21
	
	.def    rx = r26                ; temporary register
	.def    rx_l = r26
	.def    rx_h = r27
	
;; macros
    .macro  unlock
    out     CPU_CCP,r_unprotect
    .endmacro
    
    .macro  left_motor_dir_set
    sbi     GPIO_GPIOR0,0
    .endmacro
    
    .macro  left_motor_dir_clr
    cbi     GPIO_GPIOR0,0
    .endmacro
    
    .macro  left_motor_on
    sbis    GPIO_GPIOR0,0
    cbi     VPORTB_OUT,0
    sbic    GPIO_GPIOR0,0
    cbi     VPORTB_OUT,1
    .endmacro
    
    .macro  right_motor_dir_set
    sbi     GPIO_GPIOR0,1
    .endmacro
    
    .macro  right_motor_dir_clr
    cbi     GPIO_GPIOR0,1
    .endmacro
    
    .macro  right_motor_on
    sbis    GPIO_GPIOR0,1
    cbi     VPORTB_OUT,2
    sbic    GPIO_GPIOR0,1
    cbi     VPORTB_OUT,3
    .endmacro
    

; reset and interupt vector table
    rjmp    power_on_reset          ; RESET 
    rjmp    not_implemented         ; CRCSCAN_NMI     
    rjmp    low_voltage             ; BOD_VLM
    rjmp    port_A_interupt         ; PORTA_PORT
    rjmp    not_implemented         ; PORTB_PORT
    rjmp    not_implemented         ; not used???
    rjmp    not_implemented         ; RTC_CNT
    rjmp    not_implemented         ; RTC_PIT
    rjmp    timer_A_interupt        ; TCA0_LUNF / TCA0_OVF
    rjmp    not_implemented         ; TCA_HUNF
    rjmp    not_implemented         ; TCA0_LCMP0 / TCA_CMP0
    rjmp    not_implemented         ; TCA0_LCMP1 / TCA_CMP1
    rjmp    not_implemented         ; TCA0_LCMP2 / TCA_CMP2
    rjmp    not_implemented         ; TCB0_INT
    rjmp    timer_D_interupt        ; TCD0_OVF
    rjmp    not_implemented         ; TCD0_TRIG
    rjmp    not_implemented         ; AC0_AC
    rjmp    not_implemented         ; ADC0_RESRDY
    rjmp    not_implemented         ; ADC0_WCOMP
    rjmp    not_implemented         ; TWI0_TWIS
    rjmp    not_implemented         ; TWI0_TWIM
    rjmp    not_implemented         ; SPI0_INT
    rjmp    not_implemented         ; USART0_RXC
    rjmp    not_implemented         ; USART0_DRE
    rjmp    not_implemented         ; USART0_TXC
    rjmp    not_implemented         ; NVMCTRL_EE
    
low_voltage:
    ldi     r_tmp_l,BOD_VLMIF_bm
    sts     BOD_INTFLAGS,r_tmp_l        ; clear bod interupt flag
    reti
    
not_implemented:
power_on_reset:
    eor     r_zero,r_zero 
    out     CPU_SREG,r_zero             ; clear status register
    ldi     r_tmp,0xD8                  ; for ccp unprotect registers
    mov     r_unprotect,r_tmp
    
    ldi     r_tmp_l,LOW(INTERNAL_SRAM_END)  ; set stack pointer to top of memory
    ldi     r_tmp_h,HIGH(INTERNAL_SRAM_END)
    out     CPU_SPL,r_tmp_l
    out     CPU_SPH,r_tmp_h
    
;; configure clock
;; The clock is set to use the internal 20Mhz clock, with a divide by 2
;; prescaler, so the CPU will be running at 10Mhz

    unlock
    sts     CLKCTRL_MCLKCTRLA,r_zero
    unlock
    ldi     r_tmp,0b00000001            ; clock prescaler enabled, div by 2
    sts     CLKCTRL_MCLKCTRLB,r_tmp   
    
;; brown out detection
;; generally disabled for now, we do set some values

    unlock
    sts     BOD_CTRLA,r_zero            ; disable bod detection
    unlock
    ldi     r_tmp,0x04
    sts     BOD_CTRLB,r_tmp             ; brown out level = 3.3V
    sts     BOD_VLMCTRLA,r_zero         ; bod level 5% above level
    sts     BOD_INTCTRL,r_zero          ; level goes below, interupts disabled

;; sleep control - disabled
    sts     SLPCTRL_CTRLA,r_zero
 
;; disable watch dog timer
    unlock
    sts     WDT_CTRLA,r_zero    

;; Port A configuration
;; PA0 - not used (used as UPDI)
;; PA1 - Analog in - tilt
;; PA2 - Analog in - tilt reference
;; PA3 - Enable H bridge (input pin, pull up resistor, works as OC input)
;; PA4 - WOA - Timer D output A
;; PA5 - WOB - Timer D output B
;; PA6 - Fwd PWM input
;; PA7 - Cw PWM input

    ldi     r_tmp,0b00110000            ; Port A4/A5 direction is output
    sts     PORTA_DIR,r_zero
    sts     PORTA_OUT,r_zero            ; outputs are set low
    
    ldi     rx_l,LOW(PORTA_PIN0CTRL)
    ldi     rx_h,HIGH(PORTA_PIN0CTRL)
    st      x+,r_zero                   ; Pin A0 NO ISR
    st      x+,r_zero                   ; Pin A1 NO ISR
    st      x+,r_zero                   ; Pin A2 NO ISR
    ldi     r_tmp,0b00001000
    st      x+,r_tmp                    ; Pin A3 pull up res, NO ISR
    ldi     r_tmp,0b00000010
    st      x+,r_tmp                    ; Pin A4 INT on rising edge
    st      x+,r_tmp                    ; Pin A5 INT on rising edge
    ldi     r_tmp,0b00000011
    st      x+,r_tmp                    ; Pin A6 INT on falling edge
    st      x+,r_tmp                    ; Pin A7 INT on falling edge
    
;; Port B configuration
;; PB0 - Output to H Bridge IN1
;; PB1 - Output to H Bridge IN2
;; PB2 - Output to H Bridge IN3
;; PB3 - Output to H Bridge IN4

    ldi     r_tmp,0b00001111            ; Port B0-B3 direction is output
    sts     PORTB_DIR,r_tmp
    sts     PORTB_OUT,r_zero            ; set B0-B3 to low
    
    ldi     rx_l,LOW(PORTB_PIN0CTRL)
    ldi     rx_h,HIGH(PORTB_PIN0CTRL)
    st      x+,r_zero                   ; Pin B0 NO ISR
    st      x+,r_zero                   ; Pin B1 NO ISR
    st      x+,r_zero                   ; Pin B2 NO ISR
    st      x+,r_zero                   ; Pin B3 NO ISR
    
;; Port Multiplexer configuration
;; No alternate assignments, set everything to 0
    ldi     rx_l,LOW(PORTMUX_CTRLA)    
    ldi     rx_h,HIGH(PORTMUX_CTRLA)
    st      x+,r_zero                   ; PORTMUX CTRLA = 0
    st      x+,r_zero                   ; PORTMUX CTRLB = 0
    st      x+,r_zero                   ; PORTMUX CTRLC = 0
    st      x+,r_zero                   ; PORTMUX CTRLD = 0
    
;; Event Gen/User configuration
;; PA6 (Fwd PWM Input) is routed to Timer B
    ldi     rx_l,LOW(EVSYS_ASYNCCH0)     
    ldi     rx_h,HIGH(EVSYS_ASYNCCH0)
    ldi     r_tmp,0x10                  ; async gen0 - Port A6
    st      x+,r_tmp
    st      x+,r_zero                   ; async gen1 - off
    st      x+,r_zero                   ; async gen2 - off
    st      x+,r_zero                   ; async gen3 - off
    st      x+,r_zero                   ; sync gen0 - off
    st      x+,r_zero                   ; sync gen1 - off
    ldi     rx_l,LOW(EVSYS_ASYNCUSER0)
    ldi     rx_h,HIGH(EVSYS_ASYNCUSER0)
    ldi     r_tmp,0x03                  ; TCB0 - use async gen0
    st      x+,r_tmp                    ; (this connects PA6 to TCB0)
    st      x+,r_zero                   ; ADC0 - no connection
    st      x+,r_zero                   ; CCL_LUT0EV0 - no connection
    st      x+,r_zero                   ; CCL_LUT1EV0 - no connection
    st      x+,r_zero                   ; CCL_LUT0EV1 - no connection
    st      x+,r_zero                   ; CCL_LUT1EV1 - no connection
    st      x+,r_zero                   ; TCD0_EV0 - no connection
    st      x+,r_zero                   ; TCD0_EV1 - no connection
    st      x+,r_zero                   ; EVOUT0 - no connection
    st      x+,r_zero                   ; EVOUT1 - no connection
    st      x+,r_zero                   ; EVOUT2 - no connection
    ldi     rx_l,LOW(EVSYS_SYNCUSER0)
    ldi     rx_h,HIGH(EVSYS_SYNCUSER0)
    st      x+,r_zero                   ; TCA0 - no connection
    st      x+,r_zero                   ; USART0 - no connection
    
;; Timer A configuration
;; Timer A is set to interupt at a rate of 500hz.  This is the heartbeat
;; of the controller, during the interupt it will recalculate the PWM
;; settings for the left and right motors based on the forward, clockwise
;; PWM inputs and the tilt/tilt reference analog inputs

    sts     TCA0_SINGLE_CTRLB,r_zero    ; normal operation
    sts     TCA0_SINGLE_CTRLC,r_zero    ; cmpXov disabled
    sts     TCA0_SINGLE_CTRLD,r_zero    ; split mode disabled
    ldi     r_tmp,0x0F
    sts     TCA0_SINGLE_CTRLECLR,r_tmp  ; clear CTRL E 
    sts     TCA0_SINGLE_CTRLFCLR,r_tmp  ; clear CTRL F
    sts     TCA0_SINGLE_EVCTRL,r_zero   ; disable event control
    ldi     r_tmp,0x01
    sts     TCA0_SINGLE_INTCTRL,r_tmp   ; enable interupts
    sts     TCA0_SINGLE_DBGCTRL,r_zero  ; disable timer in debug mode

    ldi     rx_l,LOW(TCA0_SINGLE_PER)   ; set period to 2500
    ldi     rx_h,HIGH(TCA0_SINGLE_PER)
    ldi     r_tmp_l,LOW(2500)
    ldi     r_tmp_h,HIGH(2500)
    st      x+,r_tmp_l
    st      x+,r_tmp_h
    ldi     r_tmp,1                      ; set compare registers to 0x0001
    st      x+,r_tmp                    ; comp0
    st      x+,r_zero
    st      x+,r_tmp                    ; comp1
    st      x+,r_zero
    st      x+,r_tmp                    ; comp2
    st      x+,r_zero
    ldi     rx_l,LOW(TCA0_SINGLE_CNT)    ; set count to 0
    ldi     rx_h,HIGH(TCA0_SINGLE_CNT)
    st      x+,r_zero
    st      x+,r_zero

    ldi     r_tmp,0x07
    sts     TCA0_SINGLE_CTRLA,r_tmp     ; clk/8, timer enabled
    
;; Timer B configuration
;; Timer B is set to measure frequency, the capture event is from PA6
;; which is the fwd pwm input.  PA6 (fwd) and PA7 (cw) pwm inputs are
;; set to interupt on the falling edge.  On these interupts the timer B
;; count is read to determine the width of the pulse on each of the inputs
;; The range of the pulses is 1ms to 2ms.  Timer B uses the TCA clock
;; which is 1.25Mhz, so a pulse of 1ms will have a count of 1250 and
;; a pulse of 2ms will have a count of 2500

    ldi     r_tmp,0x03                  ; frequency measurement mode
    sts     TCB0_CTRLB,r_tmp
    ldi     r_tmp,0x01                  ; filter event capture input
    sts     TCB0_EVCTRL,r_tmp
    sts     TCB0_INTCTRL,r_zero         ; disable interupts
    sts     TCB0_DBGCTRL,r_zero         ; don't run in debug mode
    sts     TCB0_CNTL,r_zero            ; set count to zero
    sts     TCB0_CNTH,r_zero
    ldi     r_tmp,0xFF
    sts     TCB0_CCMPL,r_tmp            ; set compare to 0xFFFF
    sts     TCB0_CCMPH,r_tmp
    ldi     r_tmp,0x05                  ; use clock from TCA0, enable timer
    sts     TCB0_CTRLA,r_tmp

;; Timer D configuration
;; Timer D is used to create the 10khz PWM signals for the H bridge
;; motor controller.  These PWM signals are output as WOA (PA4) and WOB (PA5)
;; which contains the speed information.  Since we need to apply the direction
;; information PA4/PA5 generate interupts on the rising edge so we can
;; use the outputs on Port B to actually drive the H bridge.  Two signals
;; are sent per motor, one of them is held low while the other has the PWM
;; signal.  Depending on which signal is low, will determine the direction

    sts     TCD0_CTRLB,r_zero           ; one ramp mode
    ldi     r_tmp,0x82
    sts     TCD0_CTRLC,r_tmp            ; CMPDSEL PWMB; CMPCSEL PWMA
    sts     TCD0_CTRLD,r_zero           ; no overrides
    sts     TCD0_EVCTRLA,r_zero         ; disable event control
    sts     TCD0_EVCTRLB,r_zero
    ldi     r_tmp,0x01
    sts     TCD0_INTCTRL,r_tmp          ; enable interupt on overflow
    sts     TCD0_INPUTCTRLA,r_zero      ; disable input control
    sts     TCD0_INPUTCTRLB,r_zero
	unlock
    ldi     r_tmp,0x33
    sts     TCD0_FAULTCTRL,r_tmp        ; cmp A and cmp B enabled
    sts     TCD0_DLYCTRL,r_zero         ; disable delay control
    sts     TCD0_DLYVAL,r_zero          ; no delay value
    sts     TCD0_DITCTRL,r_zero         ; no dithering
    sts     TCD0_DITVAL,r_zero
    sts     TCD0_DBGCTRL,r_zero         ; disable in debug mode
    
    ldi     r_tmp_l,LOW(2000)           ; set clear compare to 2000
    ldi     r_tmp_h,HIGH(2000)
    sts     TCD0_CMPACLRL,r_tmp_l       ; this will give us a 10khz pwm
    sts     TCD0_CMPACLRH,r_tmp_h    
    sts     TCD0_CMPBCLRL,r_tmp_l
    sts     TCD0_CMPBCLRH,r_tmp_h
    
    ;; set the cmpA set and cmpB set to give us an initial debug value
    ldi     r_tmp_l,LOW(500)
    ldi     r_tmp_h,HIGH(500)
    sts     TCD0_CMPASETL,r_tmp_l
    sts     TCD0_CMPASETH,r_tmp_h
    ldi     r_tmp_l,LOW(1500)
    ldi     r_tmp_h,HIGH(1500)
    sts     TCD0_CMPBSETL,r_tmp_l
    sts     TCD0_CMPBSETH,r_tmp_h

    ldi     r_tmp,0x01                  ; probably not needed yet
    sts     TCD0_CTRLE,r_tmp
    
wait_for_timer_d_ready:
    lds     r_tmp,TCD0_STATUS
    sbrs    r_tmp,0
    rjmp    wait_for_timer_d_ready

    
    sts     TCD0_STATUS,r_zero 
    ldi     r_tmp,0x01                  ; 20Mhz clock, enable timer
    sts     TCD0_CTRLA,r_tmp
    
;; initialize values
    eor     r_fwd_pwm_l,r_fwd_pwm_l     ; zero fwd pwm value
    eor     r_fwd_pwm_h,r_fwd_pwm_h
    eor     r_cw_pwm_l,r_cw_pwm_l       ; zero cw pwm value
    eor     r_cw_pwm_h,r_cw_pwm_h 
    left_motor_dir_clr
    right_motor_dir_clr
    
    
;; Interupt configuration
    out     CPU_CCP,r18
    sts     CPUINT_CTRLA,r_zero
    sts     CPUINT_LVL0PRI,r_zero
    sts     CPUINT_LVL1VEC,r_zero
    sei                                 ; enable interupts


loop:
    rjmp     loop                       ; endless loop, program is entirely
                                        ; interupt driven
    
    
;; timer A interupt - 500Hz, recalculate the left and right more PWM values
timer_A_interupt:
    movw    r_left_pwm,r_fwd_pwm        ; left_pwm = fwd_pwm + cw_pwm
    add     r_left_pwm_l,r_cw_pwm_l
    adc     r_left_pwm_h,r_cw_pwm_h
    brmi    left_motor_rev
    left_motor_dir_clr
    rjmp    calc_left_motor_pwm
left_motor_rev:
    left_motor_dir_set                  ; if (left_pwm < 0)
    com     r_left_pwm_h                ;   left_pwm = -left_pwm
    neg     r_left_pwm_l 
    sbci    r_left_pwm_h,255 
calc_left_motor_pwm:
    ldi     r_tmp_l,LOW(2000)           ; value = 2000 - left_pwm *3
    ldi     r_tmp_h,HIGH(2000)          ; if (value <0)
    sub     r_tmp_l,r_left_pwm_l        ;   value = 0
    sbc     r_tmp_h,r_left_pwm_h
    sub     r_tmp_l,r_left_pwm_l
    sbc     r_tmp_h,r_left_pwm_h
    sub     r_tmp_l,r_left_pwm_l
    sbc     r_tmp_h,r_left_pwm_h
    brpl    set_left_motor_pwm
    ldi     r_tmp_l,0
    ldi     r_tmp_h,0
set_left_motor_pwm:
    sts     TCD0_CMPASETL,r_tmp_l
    sts     TCD0_CMPASETH,r_tmp_h
        
calc_right_motor:    
    movw    r_right_pwm,r_fwd_pwm       ; right_pwm = fwd_pwm - cw_pwm
    sub     r_right_pwm_l,r_cw_pwm_l
    sbc     r_right_pwm_h,r_cw_pwm_h
    brmi    right_motor_rev
    right_motor_dir_clr
    rjmp    calc_right_motor_pwm
right_motor_rev:
    right_motor_dir_set                 ; if (right_pwm < 0)
    com     r_right_pwm_h               ;   right_pwm = -right_pwm
    neg     r_right_pwm_l
    sbci    r_right_pwm_h,255
calc_right_motor_pwm:
    ldi     r_tmp_l,LOW(2000)           ; value = 2000 - left_pwm *3
    ldi     r_tmp_h,HIGH(2000)          ; if (value <0)
    sub     r_tmp_l,r_right_pwm_l        ;   value = 0
    sbc     r_tmp_h,r_right_pwm_h
    sub     r_tmp_l,r_right_pwm_l
    sbc     r_tmp_h,r_right_pwm_h
    sub     r_tmp_l,r_right_pwm_l
    sbc     r_tmp_h,r_right_pwm_h
    brpl    set_right_motor_pwm
    ldi     r_tmp_l,0
    ldi     r_tmp_h,0
set_right_motor_pwm:
    sts     TCD0_CMPBSETL,r_tmp_l
    sts     TCD0_CMPBSETH,r_tmp_h
    
    ldi     r_tmp,0x01                  ; update values on next cycle
    sts     TCD0_CTRLE,r_tmp
    
    ldi     r_tmp,0x01
    sts     TCA0_SINGLE_INTFLAGS,r_tmp
    reti

;; Port A interupts

port_A_interupt:
    in      r_acc_b,VPORTA_INTFLAGS
    cpse    r_acc_b,r_zero
    rjmp    test_fwd_pwm
    reti

test_fwd_pwm:
    bst     r_acc_b,6                   
    brtc    test_cw_pwm
    sbi     VPORTA_INTFLAGS,6
    rcall   get_pulse_width
    brcs    test_cw_pwm                 ; if carry set ignore pulse width
    movw    r_fwd_pwm,rx
    
test_cw_pwm:
    bst     r_acc_b,7
    brtc    test_woa
    sbi     VPORTA_INTFLAGS,7
    rcall   get_pulse_width
    brcs    test_woa
    movw    r_cw_pwm,rx

test_woa:
    bst     r_acc_b,4
    brtc    test_wob
    sbi     VPORTA_INTFLAGS,4
    left_motor_on
    
test_wob:
    bst     r_acc_b,5
    brtc    port_A_interupt
    sbi     VPORTA_INTFLAGS,5
    right_motor_on

    rjmp    port_A_interupt

;; we read the count in the timer B to get the pulse width, if the value
;; is less than 1000 or greater then 2750 we will discard it by returning
;; with the carry flag set which the caller should read.  If the value is
;; valid we subtract 1875 to normalize it and cap it to a min/max of -625/625
;; if the value is between -15 and 15 we zero it.
;; pulse width is returned in rx
get_pulse_width:
    lds     rx_l,TCB0_CNTL              
    lds     rx_h,TCB0_CNTH              ; rx = tcb count
    subi    rx_l,LOW(1000)              ; rx -= 1000
    sbci    rx_h,HIGH(1000)             
    brcs    get_pulse_width_ret         ; carry set rx was < 1000
    subi    rx_l,LOW(250)               ; rx -= 250
    sbci    rx_h,HIGH(250)
    brcc    get_pulse_width_2           ; if carry set, then set rx = 0
    eor     rx_l,rx_l                   
    eor     rx_h,rx_h
get_pulse_width_2:
    subi    rx_l,LOW(610)               ; rx -= 610
    sbci    rx_h,HIGH(610)
    brcs    get_pulse_width_4           ; if carry set, then rx < -15
    sbiw    rx_l,30                     ; rx -= 30
    brcc    get_pulse_width_3           ; if carry clear, then rx > 15
    ldi     rx_l,LOW(-15)               ; set rx = -15
    ldi     rx_h,HIGH(-15)              ; since we add 30 then sub 15 this
get_pulse_width_3:                      ; will make rx = 0, so this logic
    adiw    rx_l,30                     ; has zeroed rx if it is between
get_pulse_width_4:                      ; -15 and 15
    sbiw    rx_l,15
	brmi	get_pulse_width_clc
    movw    r_tmp_l,rx_l                ; rx is normalized now, check for max
    subi    r_tmp_l,LOW(625)            ; value of 625
    sbci    r_tmp_h,HIGH(625)
    brcs    get_pulse_width_clc         ; carry set, clear it and return rx
    subi    r_tmp_l,LOW(250)            ; test for tcb value > 2750
    sbci    r_tmp_h,HIGH(250)
    brcs    get_pulse_width_5
    sec                                 ; set carry to indicate value is out
    ret                                 ; of range
get_pulse_width_5:
    ldi     rx_l,LOW(625)               ; rx is max 625
    ldi     rx_h,HIGH(625)

get_pulse_width_clc:
    clc
get_pulse_width_ret:                    ; will ignore value in r_tmp
    ret
  
  
    
timer_D_interupt:
    ldi     r_tmp,0x0F
    sts     PORTB_OUTSET,r_tmp            ; set all motor outputs to high

    ldi     r_tmp,0x01
    sts     TCD0_INTFLAGS,r_tmp
    reti


