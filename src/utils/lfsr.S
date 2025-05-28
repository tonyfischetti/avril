.section .bss
.global lfsr_state
lfsr_state:
    .skip 1             ; Reserve 1 byte for LFSR state (uninitialized data)

.section .text
.global init_lfsr
.global get_next_byte

.type init_lfsr, @function
init_lfsr:
    sts lfsr_state, r24 ; Save seed value from r24 into lfsr_state
    ret

.type get_next_byte, @function
get_next_byte:
    clr   r17
    rcall get_next_bit
    rcall get_next_bit
    rcall get_next_bit
    rcall get_next_bit
    rcall get_next_bit
    rcall get_next_bit
    rcall get_next_bit
    rcall get_next_bit
    mov r24, r17
    ret

.type get_next_bit, @function
get_next_bit:
    clr r19
    lds r16, lfsr_state
    mov r18, r16
    eor r19, r16 ; (r16 >> 0)
    lsr r16
    lsr r16
    eor r19, r16 ; (r16 >> 2)
    lsr r16
    eor r19, r16 ; (r16 >> 3)
    lsr r16
    eor r19, r16 ; (r16 >> 4)
    lsr r16
    lsr r16
    lsr r16
    lsr r19      ; carry flag is the fresh bit!
    ror r18      ; carry flag is the random bit and r18 is new state
    rol r17
    sts lfsr_state, r18
    ret

