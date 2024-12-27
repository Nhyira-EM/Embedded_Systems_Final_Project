.syntax unified

.section mysection, "ax"
.global timerHandler
.type timerHandler, "function"

timerHandler:
	# r0-r3, r0 return
	LDR R0, =0x48000014	// ODR for GPIOA
	LDR R1, [R0]
	MOVS R2, #1
	LSLS R2, #5
	EORS R1, R2
	STR R1, [R0]

	LDR R0, =0x40000410	// SR for TIM3
	LDR R1, [R0]
	MOVS R2, #1
	BICS R1, R1, R2
	STR R1, [R0]
	BX LR
