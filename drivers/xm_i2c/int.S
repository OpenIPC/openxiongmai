.section .rodata
LC0:
    .asciz "<3>i2c_write timeout %#x\n"
LC1:
    .asciz "<3>i2c_read timeout %#x\n"

.text
    .global i2c_read
i2c_read:
		mov	ip, sp
		push	{r4, r5, r6, r7, fp, ip, lr, pc}
		sub	fp, ip, #4
		ldr	r6, [fp, #4]
		dsb	sy
		lsl	r0, r0, #16
		add	r5, r0, #-33554432	@ 0xfe000000
		str	r1, [r5, #12]
		dsb	sy
		str	r3, [r5, #24]
		ldr	r1, [r5]
		dsb	sy
		orr	r0, r1, #64	@ 0x40
		dsb	sy
		str	r0, [r5]
		bic	r1, r1, #64	@ 0x40
		dsb	sy
		cmp	r3, #0
		str	r1, [r5]
		addeq	lr, r5, #16
		beq	A_208
		sub	ip, r3, #-536870911	@ 0xe0000001
		add	lr, r5, #16
		mov	r0, #0
		lsl	ip, ip, #3
A_1e8:
        lsr	r1, r2, ip
		uxtb	r1, r1
		dsb	sy
		add	r0, r0, #1
		str	r1, [lr]
		cmp	r0, r3
		sub	ip, ip, #8
		bne	A_1e8
A_208:
        dsb	sy
		mov	r3, #0
		str	r3, [lr]
		dsb	sy
		mov	r7, r5
		mov	r3, #11
		str	r3, [r7], #72	@ 0x48
		ldr	r3, [r5, #72]	@ 0x48
		dsb	sy
		tst	r3, #16
		movweq	r4, #4097	@ 0x1001
		beq	A_244
		b	A_260
A_23c:
        subs	r4, r4, #1
		beq	A_320
A_244:
        movw	r0, #25164	@ 0x624c
		movt	r0, #16
		bl	__const_udelay	@R_ARM_CALL	__const_udelay
		ldr	r3, [r7]
		dsb	sy
		tst	r3, #16
		beq	A_23c
A_260:
        dsb	sy
		mov	r3, #16
		str	r3, [r5, #68]	@ 0x44
		dsb	sy
		str	r6, [r5, #28]
		ldr	r3, [r5]
		dsb	sy
		orr	r2, r3, #128	@ 0x80
		dsb	sy
		str	r2, [r5]
		bic	r3, r3, #128	@ 0x80
		dsb	sy
		str	r3, [r5]
		dsb	sy
		movw	r3, #2051	@ 0x803
		str	r3, [r5]
		ldr	r3, [r5, #72]	@ 0x48
		dsb	sy
		tst	r3, #32
		movweq	r4, #4097	@ 0x1001
		beq	A_2c0
		b	A_2dc
A_2b8:
        subs	r4, r4, #1
		beq	A_354
A_2c0:
        movw	r0, #25164	@ 0x624c
		movt	r0, #16
		bl	__const_udelay	@R_ARM_CALL	__const_udelay
		ldr	r3, [r7]
		dsb	sy
		tst	r3, #32
		beq	A_2b8
A_2dc:
        cmp	r6, #0
		moveq	r0, r6
		beq	A_310
		mov	r0, #0
		add	r1, r5, #20
		mov	r3, r0
A_2f4:
        lsl	r0, r0, #8
		ldr	r2, [r1]
		dsb	sy
		add	r3, r3, #1
		orr	r0, r2, r0
		cmp	r3, r6
		bne	A_2f4
A_310:
        dsb	sy
		mov	r3, #32
		str	r3, [r5, #68]	@ 0x44
		ldm	sp, {r4, r5, r6, r7, fp, sp, pc}
A_320:
        ldr	r1, [r5, #72]	@ 0x48
		dsb	sy
		ldr     r0, =LC0
A_330:
        bl	printk	@R_ARM_CALL	printk
		dsb	sy
		mov	r3, #1
		str	r3, [r5]
		dsb	sy
		movw	r3, #65535	@ 0xffff
		str	r3, [r5, #68]	@ 0x44
		mvn	r0, #0
		ldm	sp, {r4, r5, r6, r7, fp, sp, pc}
A_354:
        ldr	r1, [r5, #72]	@ 0x48
		dsb	sy
		ldr r0, =LC1
		b	A_330
