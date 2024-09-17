/*
 * Trabajo Practico Integrador.asm
 *
 *   Author: Zuretti Agustin
 */ 

.def temp = r20
.def ldiTemp = r16
.def temp2 = r17
.def fondo_detectado = r18
.def estadoSensores = r19
.def contadorSinLinea = r21

.equ PIN_DIR_IZQ = PD5
.equ PIN_DIR_DER = PD6
.equ PIN_SENSOR_IZQ = PC0
.equ PIN_SENSOR_DER = PC2

.equ DUTY_CYCLE_MIN = 20
.equ DUTY_CYCLE_STRONG = 80

.equ F_CPU = 16000000          
.equ PRESCALER = 1024           
.equ SECONDS = 2              
.equ OCR1A_VAL = ((F_CPU / PRESCALER) * SECONDS) - 1

.equ TIMER1_CONTAR = 49911
.equ limiteSinLinea = 2

; macro para ajustar el duty cycle 
.macro dutycycle
    ldi temp2, @0*255/100;
.endmacro 

; macro para cargar registros menores al 16 
.macro ldi_reg 
    ldi ldiTemp, @1
    mov @0, ldiTemp
.endmacro

.cseg

.org 0x0000
    jmp main
.org		0x0016								
	jmp		TIMER1_COMPA_vect
	rjmp loop_select	

.org 		INT_VECTORS_SIZE 	

configurar_puertos:
    ; Inicializo el contador de veces sin ver la linea en 0 
	clr contadorSinLinea

	; Configurar como entrada los pines conectados a los sensores infrarrojos
	cbi DDRC, PIN_SENSOR_IZQ
    cbi DDRC, PIN_SENSOR_DER

	; Configurar como salida los pines conectados a dirección de motores
    sbi DDRD, PIN_DIR_IZQ 
    sbi DDRD, PIN_DIR_DER 

    ; Configurar PB3 y PD3 como salida para PWM (motores)
    sbi DDRB, PB3 
    sbi DDRD, PD3 
    ret

configurar_timer2_PWM:
    ; Configurar el Timer2 para Fast PWM
	clr temp
    ldi temp, (1<<WGM21)|(1<<WGM20)  
    sts TCCR2A, temp

    ; Configurar para Clear OC2A/OC2B on compare match, set at BOTTOM
    ori Temp, (1<<COM2A1)|(1<<COM2B1)
    sts TCCR2A, temp

    ; Establecer prescaler a 64 
    ldi temp, (1<<CS22)  
    sts TCCR2B, temp

    ; Inicializar registros OCR2A y OCR2B a 0
    clr temp                        
    sts OCR2A, temp
    sts OCR2B, temp
    ret

limpiar_timer1:
    ; Limpio todos los registros 
    ldi     temp, 0
    sts     TCCR1B, temp          

    sts     TCCR1A, temp           
    sts     TCNT1H, temp           
    sts     TCNT1L, temp           
    sts     TIMSK1, temp           
	ret

configurar_timer1_primer_delay:
    ldi     temp, 0                
    sts     TCCR1A, temp           
    ldi		temp, (1<<CS12) | (1<<CS10) 
    sts     TCCR1B, temp           
	ret

Config_timer1_delay:
	clr temp
    ; Configurar el modo CTC del Timer1 y establecer el prescaler a 1024
    ldi temp, (1<<WGM12)|(1<<CS12)|(1<<CS10) 
    sts TCCR1B, temp

    ldi temp, high(OCR1A_VAL)    
    sts OCR1AH, temp
    ldi temp, low(OCR1A_VAL)     
    sts OCR1AL, temp

    ; Habilitar la interrupción por comparación en A
    ldi temp, (1<<OCIE1A)        
    sts TIMSK1, temp 
    sei                         
    ret

leer_sensores:
	push r6

    in estadoSensores, PINC               
    andi estadoSensores, 0b00000101       

    ; Comprobar si al menos un sensor detecta la línea durante el ciclo correspondiente 
    
    cpi estadoSensores, 0b00000001        
    breq linea_detectada
    cpi estadoSensores, 0b00000100        
    breq linea_detectada
    cpi estadoSensores, 0b00000000        
    breq linea_detectada

    ; Ningún sensor detecta la línea, cargo 0 en r6 usando la macro 

    ldi_reg r6, 0                             ;  r6 = 0
    rjmp fin_leer_sensores

linea_detectada:
    ldi_reg r6, 1                             ;  r6 = 1 

fin_leer_sensores:
	pop r6
    ret

chequeo_fondo:

    rcall leer_sensores            ; Llama a la subrutina que lee los sensores
    cpi estadoSensores, 0b00000101 
    brne fondo_es_blanco           
    ldi fondo_detectado, 0              ; 'fondo_detectado' = 1
    rjmp fin_deteccion_fondo

fondo_es_blanco:
    ldi fondo_detectado, 1              ; 'fondo_detectado' = 1

fin_deteccion_fondo:
    ret

loop_select:
    cpi fondo_detectado, 0       ; Compara el registro con 0 (fondo negro)
    breq fondo_negro		
    rjmp loop_blanco		
fondo_negro:    
	rjmp loop_negro      

main:
	; Initializar Stack Pointer
    ldi     temp, LOW(RAMEND)
    out     SPL, temp
    ldi     temp, HIGH(RAMEND)
    out     SPH, temp

	rcall configurar_timer1_primer_delay

    ; Delay inicial de 5 segundos
	rcall delay1s
	rcall delay1s
	rcall delay1s
	rcall delay1s
	rcall delay1s

    ;Reseteo el timer 1 
	rcall limpiar_timer1

	rcall configurar_puertos
	rcall configurar_timer2_PWM
	rcall config_timer1_delay
	rcall chequeo_fondo
	rjmp loop_select
	

loop_negro:
	rcall leer_sensores
    rcall decidir_movimiento_negro
    rjmp loop_negro

loop_blanco:
	rcall leer_sensores
    rcall decidir_movimiento_blanco
    rjmp loop_blanco
	

decidir_movimiento_negro:

    cpi estadoSensores, 0x00 
    breq detener                    ; Ambos sensores en 0: detener
    
    ldi temp, 0b00000101
    cp estadoSensores, Temp
    breq avanzar                ; Ambos sensores en 1: avanzar

    ldi Temp, (1 << PIN_SENSOR_DER)  ; Solo sensor derecho en 1, giro derecha
    cp estadoSensores, temp 
    breq giro_derecha

    ldi Temp, (1 << PIN_SENSOR_IZQ) ; Solo sensor izquierdo en 1, giro izquierda
    cp estadoSensores, Temp
    breq giro_izquierda

    rjmp end_decidir_mov_negro

end_decidir_mov_negro:
	ret

decidir_movimiento_blanco:

    cpi estadoSensores, 0b00000101
    breq detener                 ; Ambos sensores en 1: detener
    
    ldi Temp, 0x00
    cp estadoSensores, Temp
    breq avanzar                 ; Ambos sensores en 0: avanzar

    ldi Temp, (1 << PIN_SENSOR_DER)
    cp estadoSensores, Temp
    brne giro_izquierda          ; Solo sensor derecho en 0: giro izquierda

    ldi Temp, (1 << PIN_SENSOR_IZQ)
    cp estadoSensores, Temp
    brne giro_derecha           ; Solo sensor izquierdo en 0: giro derecha

    rjmp end_decidir_mov_blanco

end_decidir_mov_blanco:
    ret

	
avanzar:
	
	sbi PORTD, PIN_DIR_IZQ
	sbi PORTD, PIN_DIR_DER 
    
	
	dutycycle DUTY_CYCLE_MIN
    sts OCR2A, Temp2       
	dutycycle DUTY_CYCLE_MIN
    sts OCR2B, Temp2       
    ret

giro_derecha:
	
	cbi PORTD, PIN_DIR_IZQ
	sbi PORTD, PIN_DIR_DER 
    
	
	dutycycle 75
    sts OCR2A, Temp2       
	dutycycle 0
    sts OCR2B, Temp2       
    ret

giro_izquierda:
	
	sbi PORTD, PIN_DIR_IZQ
	cbi PORTD, PIN_DIR_DER 
    
	
	dutycycle DUTY_CYCLE_MIN
    sts OCR2A, Temp2       
	dutycycle 75
    sts OCR2B, Temp2       
    ret

detener:
	
	cbi PORTD, PIN_DIR_IZQ
	cbi PORTD, PIN_DIR_DER 
    
	dutycycle 0
    sts OCR2A, Temp2      
	dutycycle 0
    sts OCR2B, Temp2       
    ret

TIMER1_COMPA_vect:
	push temp
    ; Verificar si se detectó la línea en el ultimo ciclo 
    ldi temp, 0
    cp r6, temp            
    breq linea_no_detectada  ; Si no se detecta la linea salto 
    clr contadorSinLinea     ; Si se detecta la línea, reinicia el contador
    rjmp fin_interupcion

linea_no_detectada:
    inc contadorSinLinea

    ldi temp, limiteSinLinea
    cp contadorSinLinea, temp
    brlo fin_interupcion

    rjmp detener_loop

detener_loop:
    rcall detener 
    rjmp detener_loop  

fin_interupcion:
	pop temp
    reti


delay1s:
	
    ldi     temp, HIGH(TIMER1_CONTAR)
    sts     TCNT1H, temp
    ldi     temp, LOW(TIMER1_CONTAR)
    sts     TCNT1L, temp

esperar:
    in      temp, TIFR1
    sbrs    temp, TOV1
    rjmp    esperar

    ldi     temp, (1<<TOV1)
    out     TIFR1, temp

    ret
