
.section .bss
.global lfsr_state
lfsr_state:
    .skip 1             ; Reserve 1 byte for LFSR state (uninitialized data)


.section .text
.global init_lfsr
.global get_next_byte


; ---------------------------------------------------------------------------
; void init_lfsr(uint8_t seed);
;
; A zero state locks the LFSR at zero forever, so a seed of 0 is replaced
; with a nonzero default.
; ---------------------------------------------------------------------------
.type init_lfsr, @function
init_lfsr:
    tst	   r24
    brne   1f
    ldi    r24, 0xA5       ; arbitrary nonzero fallback seed
1:
    sts	   lfsr_state, r24 ; Save seed value from r24 into lfsr_state
    ret


; ---------------------------------------------------------------------------
; uint8_t get_next_byte(void);
;
; Generates eight bits by calling get_next_bit eight times.
; On return:
;   r24 = random byte
;   r16 and r17 are preserved (call-saved per the ABI)
;
; originally use `rcall` 8 times... that caused excessive stack usage :(
; now uses loop
; ---------------------------------------------------------------------------
.type get_next_byte, @function
get_next_byte:
    push   r16
    push   r17
    clr    r17
    ldi    r20, 8
.byte_loop:
    rcall  get_next_bit
    dec    r20
    brne   .byte_loop
    mov    r24, r17
    pop    r17
    pop    r16
    ret


; ---------------------------------------------------------------------------
; void get_next_bit(void);
;
; Generates one bit and:
;   - rotates it into r17 through CARRY (so caller must have set up r17)
;   - updates lfsr_state
; Clobbers r16, r18, r19, and the carry flag. r16 is call-saved in the
; avr-gcc ABI, but get_next_byte pushes it on our behalf, so no pushes here.
; ---------------------------------------------------------------------------
.type get_next_bit, @function
get_next_bit:
    clr  r19
    lds  r16, lfsr_state
    mov  r18, r16
    eor  r19, r16 ; (r16 >> 0)
    lsr  r16
    lsr  r16
    eor  r19, r16 ; (r16 >> 2)
    lsr  r16
    eor  r19, r16 ; (r16 >> 3)
    lsr  r16
    eor  r19, r16 ; (r16 >> 4)
    lsr  r19      ; carry flag is the fresh bit!
    ror  r18      ; carry flag is the random bit and r18 is new state
    rol  r17
    sts  lfsr_state, r18
    ret

