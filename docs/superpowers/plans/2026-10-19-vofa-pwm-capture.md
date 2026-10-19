# VOFA PWM capture implementation plan

**Goal:** Record physical LIN1/LIN2/LIN3 levels and replay them in VOFA without rewiring the board.

**Architecture:** A dedicated `User/bsp/pwm_capture` module owns TIM6 and DMA2_Channel1. TIM6 update requests copy GPIOB IDR into a normal-mode DMA buffer. The foreground polls completion and VOFA replays the frozen buffer using JustFloat. Motor control continues during recording and playback.

**Tech stack:** STM32G474 HAL, Keil, USB CDC, JustFloat.

## Constraints

- Preserve current FOC, ADC, phase calibration, PWM outputs, and user edits.
- Use physical PB13/PB14/PB15, without logical phase remapping.
- Record 4000 halfwords at 4 MHz: 0.25 us per point, 1 ms capture, 8 KB RAM.
- No per-sample interrupt or blocking acquisition loop. Reserve TIM6 and DMA2_Channel1 for this debug module; initialization is owned by User code, outside CubeMX generated files.
- MCU pin levels do not measure MOS gate voltage or driver deadtime. DMA arbitration can add edge timing uncertainty.
- No tests, firmware build, or flashing requested in this turn; perform source/diff inspection only and disclose this limit.

## Steps

- [x] Add `User/bsp/pwm_capture.h/.c`: start, process, cancel and read frozen samples. Validate timer divisor, handle resource conflict, DMA errors and timeout. Initialize lazily and poll without waiting.
- [x] Extend `User/app/vofa.h/.c`: default capture mode; wait for enabled PWM for automatic capture after host opens; manual capture request in Keil; retain original telemetry mode. Batch frozen samples, preserve the USB buffer until transfer finishes, and advance the replay cursor only after completed transfers. Discard session state on disconnect/reopen.
- [x] Register the module in `MDK-ARM/FOC_Project.uvprojx` and document reserved peripherals and VOFA operation in `docs/vofa-pwm-capture.md`.
- [x] Read back source and inspect the scoped diff, including buffer bounds, startup, mode switching, disconnect, retry, error and replay completion paths.

Source review also identified the USB class-pointer check/use gap. USB LP masking now covers eligibility, pointer capture and TxState accesses, with a separate guarded submission after packing. Independent read-only review found no further actionable issues. Build and on-board timing are unverified.

## Debug interface

- `vofa_mode`: 0 = original live telemetry, 1 = frozen PWM capture/replay.
- `vofa_pwm_capture_request`: set 1 to queue one capture, including when PWM is disabled.
- `vofa_pwm_sent_samples`: completed replay samples, 0 to 4000.
- `pwm_capture`: capture status, error, nominal sample interval and successful capture count.
- Capture mode sends eight floats per JustFloat frame: LIN1/LIN2/LIN3, user-added live replay currents ia/ib/ic, capture ID and point index. Only the GPIO levels are recorded at 4 MHz. VOFA point interval within a capture is 0.00025 ms. Distinct captures are separate records, not a continuous time series.

## Follow-up: repeated capture for observation

User reported the single record stopped too soon, then authorized choosing the most useful observation method. Preserve the capture/replay architecture and enable repeated captures by default, with explicit record markers for offline analysis.

- [x] Add runtime `vofa_pwm_auto_repeat = 1`; wait for the final USB transfer of a record before starting the next capture. Setting 0 finishes the current record and leaves manual requests available. First automatic capture still waits for PWM; subsequent captures can observe PWM OFF. DMA errors remain latched.
- [x] Preserve the user-added current channels and identify them as live replay values, not synchronous GPIO/current samples. Append capture ID and point index as CH6/CH7.
- [x] Size the USB buffer for 16 frames at the maximum supported channel count and guard packing bounds (fixed before the repeat change).
- [x] Update observation instructions, segment gaps, channel selection and Keil controls.
- [x] Read back the follow-up edits and obtain a source-only review; no build, tests or flashing.

Follow-up source review found no actionable issues in replay completion, automatic/manual requests, errors or reconnect handling. The scoped `git diff --check` completed without diagnostics; on-board sampling and USB behavior remain unverified.
