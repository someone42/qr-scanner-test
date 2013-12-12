; Hand-optimised image processing functions.

  SECTION .text:CODE(1)
  THUMB

; void calculateBlockStatistics(uint32_t *results, uint8_t *block, uint32_t stride)
;
; Calculates the sum, minimum and maximum of a 8x8 pixel block.
; This relies heavily on ARM Cortex-M4 DSP instructions to achieve 3 cycles
; per pixel performance.
; Upon exit:
; results[0] = sum
; results[1] = min
; results[2] = max
  PUBLIC calculateBlockStatistics
calculateBlockStatistics
  push    {r4, r5, r6, r7, r8, r10, r11}
  ; r5 = sum, r6 = min, r7 = max
  movs    r8, #0 ; dummy zero register for usada8 instruction
  movs    r5, #0
  movs    r6, #0
  subs    r6, r6, #1
  movs    r7, #0
  movs    r11, #8

statsloop
  ; Load a row (8 pixels) of the block.
  ldm     r1, {r3, r4}
  ; usada8 accumulates the sum of differences between each byte in r8 and r3.
  ; But since r8 is 0, this means that it will unsigned add each byte in r3
  ; and accumulate into r5.
  usada8  r5, r8, r3, r5
  ; Likewise, this instruction sums each byte in r4 and accumulates into r5.
  usada8  r5, r8, r4, r5
  ; The usub8/sel combination selects the minimum of each byte in r3 and r6,
  ; then writes the minimum byte back into r6. r10 is used as a dummy register
  ; since the result of usub8 (which we don't want) has to go somewhere.
  usub8   r10, r3, r6
  sel     r6, r6, r3
  ; Likewise, but finds minimum of r4 and r6.
  usub8   r10, r4, r6
  sel     r6, r6, r4
  ; This usub8/sel combination has the two selection operands swapped around
  ; so that instead of finding the minimum, it finds the maximum of each byte.
  usub8   r10, r3, r7
  sel     r7, r3, r7
  usub8   r10, r4, r7
  sel     r7, r4, r7
  ; Move to next row in block.
  adds    r1, r1, r2
  subs    r11, r11, #1
  bne.n   statsloop

  ; The above loop calculates the parallel (byte-wise) minimum in r6.
  ; We still need to find the overall minimum of each byte in r6.
  uxtb    r1, r6
  uxtb    r2, r6, ror #8
  ; usub8/sel is faster than cmp/it/conditional move.
  usub8   r3, r2, r1
  sel     r1, r1, r2
  uxtb    r2, r6, ror #16
  usub8   r3, r2, r1
  sel     r1, r1, r2
  uxtb    r2, r6, ror #24
  usub8   r3, r2, r1
  sel     r6, r1, r2
  ; Likewise, we still need to find the overall maximum of each byte in r7.
  uxtb    r1, r7
  uxtb    r2, r7, ror #8
  ; usub8/sel is faster than cmp/it/conditional move.
  usub8   r3, r2, r1
  sel     r1, r2, r1
  uxtb    r2, r7, ror #16
  usub8   r3, r2, r1
  sel     r1, r2, r1
  uxtb    r2, r7, ror #24
  usub8   r3, r2, r1
  sel     r7, r2, r1

  str     r5, [r0, #0]
  str     r6, [r0, #4]
  str     r7, [r0, #8]
  pop     {r4, r5, r6, r7, r8, r10, r11}
  bx      lr

; void extractLuminance(uint8_t *out, uint8_t *in, uint32_t count)
;
; Extract luminance from YUV422 data. This assumes the data format is:
; Y U Y V or Y V Y U.
; Performance is about 3 cycles per byte.
  PUBLIC extractLuminance
extractLuminance
  push    {r4, r5, r6, r7}
  lsrs    r2, r2, #3

  ; This loop grabs every other byte and packs it. It is unrolled to process
  ; 8 output bytes per loop.
extloop:
  ldm     r1!, {r4, r5, r6, r7} ; read from in
  uxtb    r3, r4 ; first byte
  lsrs    r4, r4, #16
  bfi     r3, r4, #8, #8 ; second byte
  bfi     r3, r5, #16, #8 ; third byte
  lsrs    r5, r5, #16
  bfi     r3, r5, #24, #8 ; fourth byte
  uxtb    r4, r6 ; fifth byte
  lsrs    r6, r6, #16
  bfi     r4, r6, #8, #8 ; sixth byte
  bfi     r4, r7, #16, #8 ; seventh byte
  lsrs    r7, r7, #16
  bfi     r4, r7, #24, #8 ; eighth byte
  stm     r0!, {r3, r4} ; write to out
  subs    r2, r2, #1
  bne.n   extloop

  pop     {r4, r5, r6, r7}
  bx      lr

; void applyThreshold(uint8_t *out, uint8_t *in, uint32_t stride, uint32_t threshold)
;
; Apply threshold to a 8x8 pixel block to create binarized bitmap.
; This relies heavily on ARM Cortex-M4 DSP instructions to achieve 3 cycles
; per pixel performance.
  PUBLIC applyThreshold
applyThreshold
  push    {r4, r5, r6, r7, r8, r10, r11}
  ; Copy threshold into each byte of r3.
  bfi     r3, r3, #8, #8
  bfi     r3, r3, #16, #16
  ; r4 = 0x08040201 and r5 = 0x80402010. These are the bit positions in the
  ; bitmap for each byte.
  mov     r4, #513
  movt    r4, #2052
  lsl     r5, r4, #4
  mov     r8, #0
  movs    r11, #8

thresholdloop
  ; Load a row (8 pixels) of the block.
  ldm     r1, {r6, r7}
  ; Use usub8 to do a parallel compare and select bit set (from r4) or bit
  ; clear (from r8).
  usub8   r10, r3, r6
  sel     r6, r4, r8
  ; Use usub8 to do a parallel compare and select bit set (from r5) or bit
  ; clear (from r8).
  usub8   r10, r3, r7
  sel     r7, r5, r8
  ; Pack all the set bits into a single byte.
  usad8   r6, r6, r8
  usada8  r6, r7, r8, r6
  ; Store the row (8 pixels) in the bitmap.
  strb    r6, [r0]
  ; Move to next row in block.
  adds    r1, r1, r2
  lsrs    r6, r2, #3
  adds    r0, r0, r6
  subs    r11, r11, #1
  bne.n   thresholdloop

  pop     {r4, r5, r6, r7, r8, r10, r11}
  bx      lr

  END

