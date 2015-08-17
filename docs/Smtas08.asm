; Version 1.0 / 01.04.2002

	list      p=16f876            
	#include <p16f876.inc>        

	__CONFIG _CP_ALL & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _LVP_OFF & _DEBUG_OFF & _CPD_OFF 
	
	CBLOCK 0x20
	counter
	pausecounter
	command
	sensors
	portbvalue
	portavalue
	result1
	result2
	result3
	result4
	ACCaL
	ACCaH
	ACCbL
	ACCbH          
	DataBlock
	ENDC 
 	
	#define count_on    t1con,0 
	#define edge        ccp1con,0  	

	ORG     0x4fc             
HelpContents
	movf pausecounter,w
	movwf pclath
	movf counter,w	
        addwf PCL,f
	dt d'13', d'10',"       S M T A S 8   T E M P E R A T U R E", d'13', d'10'
	dt "       M E A S U R E M E N T   S Y S T E M", d'13', d'10'
	dt "             Version 1.0 / 01.04.2002", d'13', d'10', d'13', d'10'
	dt "     m -> Infinite measurement cycle", d'13', d'10' 
	dt "     1 -> First sensor, single measurement", d'13', d'10'
	dt "     2 -> First two sensors, single measurement", d'13', d'10'
	dt "     3 -> First three sensors, single measurement", d'13', d'10'
	dt "     4 -> First four sensors, single measurement", d'13', d'10'
	dt "     5 -> First five sensors, single measurement", d'13', d'10'
	dt "     6 -> First six sensors, single measurement", d'13', d'10'
	dt "     7 -> First seven sensors, single measurement", d'13', d'10'
	dt "     8 -> First eight sensors, single measurement", d'13', d'10'
	dt "     s -> Stop", d'13', d'10', d'13', d'10'
	dt "     Please Make a Choice :", d'13', d'10', d'13', d'10', "*"

Help
	movwf counter
	movlw 5
	movwf pausecounter
nextchar
	call HelpContents
	movwf command
	movlw a'*'
        xorwf command,w
	btfsc status,z
	goto endhelp 
	movf command,w
	call Send
	movlw 1
	addwf counter,f
	btfsc status,c
	incf pausecounter,f
	goto nextchar
endhelp
	movlw 0
	movwf pclath 
	return	
   	
	ORG     0x000             
	clrf    pclath            
  	goto    Start

GetAsciiValue
        addwf   PCL,f
        dt "0123456789ABCDEF"

CheckForCommand
check	btfss rcsta,oerr
	goto ok
	bcf rcsta,cren
	bsf rcsta,cren

ok	btfss pir1,rcif
	goto nonewcommand
	nop
	nop
	movf rcreg,w
	movwf command

	movf command,w
	sublw a'8'
	btfss status,c
	goto nonewcommand
	
	movlw a'1'
	subwf command,w
	btfss status,c
	goto nonewcommand

	movlw a'0'
	subwf command,w
	movwf sensors
	clrf command	
	return

nonewcommand
	movlw a'm'
        xorwf command,w
        btfss status,z
        goto no_m
        movlw d'8'    
        movwf sensors
        return
no_m	
	movlw a'h'
        xorwf command,w
        btfsc status,z
        call Help
	clrf command
	goto check	

Pause    
	movlw d'255'
	movwf pausecounter
	decfsz pausecounter,f
	goto $-1
	return

Send
	movwf txreg
	nop
	nop
	btfss pir1,txif
	goto $-1
	return

Transmit
	swapf result2,w
	andlw b'00001111'
	call GetAsciiValue
	call Send

	movf result2,w
	andlw b'00001111'
	call GetAsciiValue
	call Send

	swapf result1,w
	andlw b'00001111'
	call GetAsciiValue
	call Send

	movf result1,w
	andlw b'00001111'
	call GetAsciiValue
	call Send

	movlw a' '
	call Send

	swapf result4,w
	andlw b'00001111'
	call GetAsciiValue
	call Send

	movf result4,w
	andlw b'00001111'
	call GetAsciiValue
	call Send

	swapf result3,w
	andlw b'00001111'
	call GetAsciiValue
	call Send

	movf result3,w
	andlw b'00001111'
	call GetAsciiValue
	call Send

	decf sensors,w
	btfsc status,z
	return

	movlw a' '
	call send
	return

Subtract 
	comf ACCbL,f          
	incf ACCbL,f
	btfsc status,z
	decf ACCbH,f
	comf ACCbH,f
	movf ACCbL,w
	addwf ACCaL,f          
	btfsc status,c
	incf ACCaH,f
	movf ACCbH,w
	addwf ACCaH,f          
	return

Addition
	movf    ACCaL,w
	addwf   result1,f         
	btfsc   status,c
	incf    result2,f
	movf    ACCaH,w
	addwf   result2,f
	return

MeasDuty
	clrf tmr1h
	clrf tmr1l
	movlw b'00000101'
	movwf ccp1con
	bcf pir1,ccp1if
	bsf t1con,tmr1on
	bcf pir1,tmr1if

	bsf edge
edge1	btfsc tmr1h,5 
	goto over
	btfss pir1,ccp1if
	goto edge1
	bcf pir1,ccp1if

	movlw DataBlock
	movwf FSR
	movf ccpr1l,w
	movwf INDF
	incf FSR,f
	movf ccpr1h,w
	movwf INDF
	
	movlw d'10'
	movwf counter
nextpulse
	bcf edge
fall	btfsc pir1,tmr1if
	goto over
	btfss pir1,ccp1if
	goto fall
	bcf pir1,ccp1if

	incf FSR,f
	movf ccpr1l,w
	movwf INDF
	incf FSR,f
	movf ccpr1h,w
	movwf INDF

	bsf edge
rise	btfsc pir1,tmr1if
	goto over
	btfss pir1,ccp1if
	goto rise
	bcf pir1,ccp1if

	incf FSR,f
	movf ccpr1l,w
	movwf INDF
	incf FSR,f
	movf ccpr1h,w
	movwf INDF
	decfsz counter,f
	goto nextpulse

	clrf result1
	clrf result2
	clrf result3
	clrf result4

	movlw d'10'
	movwf counter
	movlw DataBlock
	movwf FSR
	decf FSR,f

nextcalculation
	incf FSR,f
	movf INDF,w
	movwf ACCbL
	incf FSR,f
	movf INDF,w
	movwf ACCbH
	incf FSR,f
	movf INDF,w
	movwf ACCaL
	incf FSR,f
	movf INDF,w
	movwf ACCaH
 	call Subtract
	call Addition
	decfsz counter,f
	goto nextcalculation

	incf FSR,f
	movf INDF,w
	movwf ACCaL
	incf FSR,f
	movf INDF,w
	movwf ACCaH

	movlw DataBlock
	movwf FSR
	movf INDF,w
	movwf ACCbL
	incf FSR,f
	movf INDF,w
	movwf ACCbH
 	call Subtract
	movf ACCaL,w
	movwf result3
	movf ACCaH,w
	movwf result4

	call Transmit
	return

over	clrf ccp1con 
	bcf t1con,tmr1on
	movlw h'ff' 
        movwf result1
	movwf result2
	movwf result3
	movwf result4
	call Transmit
	return

MeasAndTransmit
	movlw d'1'
	movwf portbvalue
	clrf portavalue
next    movf portbvalue,w
	movwf portb
	movf portavalue,w
	movwf porta
	call Pause
	call MeasDuty
	movf portbvalue,w
	addwf portbvalue,f
	incf portavalue,f
	decfsz sensors,f
	goto next

	movlw d'13'       
	call Send

	movlw d'10'       
	call Send
	return

Start
	bsf status,rp0 
	movlw b'10000000'  
	movwf option_reg        

	bcf status,rp0 
	clrf portb
	clrf porta
	bsf status,rp0

	movlw b'111000' 
	movwf trisa
	clrf trisb
	movlw b'11111111'
	movwf trisc	   

	movlw d'21'
	movwf spbrg
	bsf txsta,txen
	bsf txsta,brgh
	bcf status,rp0	   
	bsf rcsta,spen
	bsf rcsta,cren        

	clrf t1con  
	clrf ccp1con

	clrf command
Main
	call CheckForCommand
	call MeasAndTransmit
	goto Main
	END
