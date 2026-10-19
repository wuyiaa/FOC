# Dual ADC injected acquisition implementation plan

**Goal:** Keep physical pins and sample IA/IC simultaneously, followed by IB/VBUS.
**Architecture:** ADC1 master uses TIM1 CC4 falling edge, ADC2 injected slave; ADC1 JEOS alone drives FOC. ADC2 regular DMA remains independent.
**Tech stack:** STM32G474, STM32G4 HAL, Keil, CubeMX.

## Constraints
- Current sampling: 2.5 cycles; VBUS: 47.5 cycles; ADC clock unchanged.
- No waiting for VBUS in FOC; retain the latest completed result.
- Preserve phase reversal, current reconstruction, user control parameters, and fixed 24 V control supply.
- No CubeMX regeneration, build, hardware tests, or flashing in this change.

## Steps
- [x] Back up the five affected source/configuration files.
- [x] Core/Src/adc.c: ADC_DUALMODE_INJECSIMULT, independent DMA access; ADC1 ranks IN3/IN2, ADC2 ranks IN1/IN11; move PA0 MSP ownership to ADC2.
- [x] Core/Src/main.c and User/bsp/voltage_sample.c: preserve slave-before-master startup, ADC2 regular DMA, and ADC1-only injected IRQ; read VBUS from ADC2 JDR2 only with JEOS set.
- [x] User/motor/foc_drv.c: use ADC1 JDR1/JDR2 and ADC2 JDR1 for IA/IB/IC in both zero-current offset acquisition and runtime acquisition; preserve A/C reversal.
- [x] FOC_Project.ioc: synchronize ranks, PA0, dual mode, and slave trigger settings using the installed CubeMX ADC metadata.
- [x] Review the scoped diff and search active ADC reads/initialization against the agreed mapping; report that hardware validation remains pending.

## Source review result

Scoped before/after diff reviewed. Active current reads and offset reads match IA=ADC1/JDR1, IB=ADC1/JDR2, IC=ADC2/JDR1. VBUS uses ADC2/JDR2 with JEOS polling. ADC2 regular ranks/DMA and ADC1 FOC callback remain unchanged. No build, CubeMX regeneration, or hardware validation performed.
