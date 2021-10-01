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

	.global i2c_write
	.type	i2c_write, %function
i2c_write:
    L_28:	mov	ip, sp
    L_2c:	push	{r3, r4, r5, r6, fp, ip, lr, pc}
    L_30:	sub	fp, ip, #4
    L_34:	ldmib	fp, {r4, lr}
    L_38:	dsb	sy
    L_3c:	lsl	r0, r0, #16
    L_40:	add	r5, r0, #-33554432	@ 0xfe000000
    L_44:	str	r1, [r5, #12]
    L_48:	dsb	sy
    L_4c:	add	r1, r3, lr
    L_50:	str	r1, [r5, #24]
    L_54:	ldr	r1, [r5]
    L_58:	dsb	sy
    L_5c:	orr	r0, r1, #64	@ 0x40
    L_60:	dsb	sy
    L_64:	str	r0, [r5]
    L_68:	bic	r1, r1, #64	@ 0x40
    L_6c:	dsb	sy
    L_70:	cmp	r3, #0
    L_74:	str	r1, [r5]
    L_78:	addeq	ip, r5, #16
    L_7c:	beq	L_b0
    L_80:	sub	r0, r3, #-536870911	@ 0xe0000001
    L_84:	add	ip, r5, #16
    L_88:	mov	r6, #0
    L_8c:	lsl	r0, r0, #3
    L_90:	lsr	r1, r2, r0
    L_94:	uxtb	r1, r1
    L_98:	dsb	sy
    L_9c:	add	r6, r6, #1
    L_a0:	str	r1, [ip]
    L_a4:	cmp	r6, r3
    L_a8:	sub	r0, r0, #8
    L_ac:	bne	L_90
    L_b0:	cmp	lr, #0
    L_b4:	subne	r2, lr, #-536870911	@ 0xe0000001
    L_b8:	movne	r1, #0
    L_bc:	lslne	r2, r2, #3
    L_c0:	beq	L_e4
    L_c4:	lsr	r3, r4, r2
    L_c8:	uxtb	r3, r3
    L_cc:	dsb	sy
    L_d0:	add	r1, r1, #1
    L_d4:	str	r3, [ip]
    L_d8:	cmp	r1, lr
    L_dc:	sub	r2, r2, #8
    L_e0:	bne	L_c4
    L_e4:	dsb	sy
    L_e8:	mov	r3, #0
    L_ec:	str	r3, [ip]
    L_f0:	dsb	sy
    L_f4:	mov	r6, r5
    L_f8:	mov	r3, #11
    L_fc:	str	r3, [r6], #72	@ 0x48
   L_100:	ldr	r3, [r5, #72]	@ 0x48
   L_104:	dsb	sy
   L_108:	tst	r3, #16
   L_10c:	movweq	r4, #4097	@ 0x1001
   L_110:	beq	L_120
   L_114:	b	L_13c
   L_118:	subs	r4, r4, #1
   L_11c:	beq	L_150
   L_120:	movw	r0, #25164	@ 0x624c
   L_124:	movt	r0, #16
   L_128:	bl	__const_udelay	@R_ARM_CALL	__const_udelay
   L_12c:	ldr	r3, [r6]
   L_130:	dsb	sy
   L_134:	tst	r3, #16
   L_138:	beq	L_118
   L_13c:	dsb	sy
   L_140:	mov	r3, #16
   L_144:	str	r3, [r5, #68]	@ 0x44
   L_148:	mov	r0, #0
   L_14c:	ldm	sp, {r3, r4, r5, r6, fp, sp, pc}
   L_150:	ldr	r1, [r5, #72]	@ 0x48
   L_154:	dsb	sy
            ldr r0, =LC0
   L_160:	bl	printk	@R_ARM_CALL	printk
   L_164:	dsb	sy
   L_168:	mov	r3, #1
   L_16c:	str	r3, [r5]
   L_170:	dsb	sy
   L_174:	movw	r3, #65535	@ 0xffff
   L_178:	str	r3, [r5, #68]	@ 0x44
   L_17c:	mvn	r0, #0
   L_180:	ldm	sp, {r3, r4, r5, r6, fp, sp, pc}
