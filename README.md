# PAANCHAJANYA — Adaptive Software-Defined Sonar Transmitter for AUVs

**Smart India Hackathon 2026 · SIH26058 · Team 186474**
Theme: Robotics and Drones · Category: Hardware

## Overview

A low-power-oriented, STM32-based sonar transmitter prototype that adapts waveform and transmission parameters to estimated underwater conditions. The MVP demonstrates digital waveform generation, timer-triggered DAC streaming, environmental-input simulation and Python-based waveform analysis.

## Key Features

- **Adaptive transmission:** Estimates sound speed and acoustic losses to select frequency, bandwidth, pulse duration, amplitude and PRI.
- **Waveform generation:** LFM, Geometric FM, HFM and Barker-13 using DDS, lookup-table interpolation and configurable pulse shaping.
- **Efficient output:** TIM6-triggered DAC streaming with DMA and double buffering.
- **Environmental simulation:** Five potentiometers for turbidity, temperature, depth, salinity and target range, with ADC oversampling and EMA filtering.
- **Window functions:** Tukey, Hann, Hamming and Blackman.
- **Low-power-oriented design:** Stop mode with RTC wake-up, WFI waits and pulse caching for repeated settings.
- **Analysis:** Python tools for serial acquisition, waveform plots, FFT, spectrograms, instantaneous frequency and matched-filter analysis.

## System Architecture

```text
Environmental Inputs (5 ADC channels)
                 ↓
       Filtering and Acoustic Model
                 ↓
       Adaptive Parameter Selection
                 ↓
       DDS Waveform Generation
                 ↓
        TIM6 + DMA + DAC (PA4)
                 ↓
     Planned Filter → Driver → Transducer
```

The current MVP implements the digital generation and DAC-output stages. The external driver, matching network and transducer remain planned payload components.

## Hardware and Pin Map

| Function | Implementation |
|---|---|
| MCU | NUCLEO-F446RE / STM32F446RE |
| DAC | Internal 12-bit DAC, PA4 |
| Turbidity / temperature | PA0 / PA1 |
| Depth / salinity | PC0 / PC1 |
| Target range | PB0 |
| User button | PC13 |
| Serial telemetry | ST-LINK Virtual COM Port, 921600 baud |
| Development / analysis | STM32CubeIDE, C, Python, NumPy, SciPy, Matplotlib, PySerial |

## MVP Specifications

| Parameter | Current MVP |
|---|---|
| DAC sampling rate | 1 MS/s |
| Intended output band | 20–50 kHz |
| Model target band | 100–500 kHz |
| Pulse duration | Approximately 0.5–7 ms |
| Waveforms | LFM, Geometric FM, HFM, Barker-13 |
| Output method | Timer-triggered DAC + DMA |

The hardware frequency mapping is:

```text
f_hw = 20 kHz + 0.075 × (f_model − 100 kHz)
```

The centre frequency and FM sweep bandwidth are scaled by 7.5%. This MVP therefore does not demonstrate full-band 100–500 kHz transmission, and pulse-compression performance is limited by the reduced sweep bandwidth.

## Firmware and Python Analysis

- **Normal telemetry:** Use `Core/Src/main.c` for readable UART output in PuTTY.
- **Waveform analysis:** `main_py.c` supports text telemetry and optional DAC-frame streaming. Set `DAC_STREAM_ENABLE` to `1` at the top of the firmware to enable streaming; binary frames appear as garbage in a serial terminal, so disable streaming for normal telemetry.

Install Python dependencies from `requirements.txt`, then run:

```bash
python dac_waveform_2.py --list
python dac_waveform_2.py --port COM5
```

On Linux, replace `COM5` with the serial device, such as `/dev/ttyACM0`. Build and flash the selected firmware using STM32CubeIDE.

## Reported Digital Results

| Metric | Result |
|---|---|
| Frames analysed | 600+ |
| CRC errors / clipped samples | 0 / 0 |
| FM out-of-band level (Blackman) | −63 to −65 dB |
| Barker-13 peak sidelobe ratio | −21.8 dB |
| Analysed FM peak frequencies | 46.4–46.6 kHz |

These are digital-sample analysis results, not acoustic-performance measurements. The clear-shallow-reef LFM sweep (43.2–50.0 kHz) matched the analysed samples; mapped values for other settings are calculations. Zero-padding smooths plotted spectra but does not improve true frequency resolution.

## Status and Future Scope

**Implemented:** STM32 waveform-generation MVP, internal DAC output, DMA streaming, four waveform modes, window functions, simulated environmental inputs, Python analysis and payload CAD concept.

**Remaining:** Quantitative Vpp, THD and analogue FFT measurements; analogue reconstruction filter; real environmental sensors; turbidity-model calibration; transducer and driver integration; underwater testing; and power/energy measurement.

**Planned payload upgrades:** External DAC rated for at least 10 MS/s, with an STM32H7-class MCU or FPGA if needed, plus amplification, impedance matching, broadband transducer and pressure-rated enclosure.

Potential applications include AUV seabed mapping, marine research, environmental monitoring and underwater infrastructure inspection. Field performance remains subject to completed hardware integration and underwater validation.

## References

STMicroelectronics STM32F446 datasheet, RM0390 and AN3126; Medwin (sound speed); Ainslie & McColm (seawater absorption); Harris (window functions); Francois & Garrison (seawater absorption); Richards et al. (suspended-particle attenuation); Barker (binary phase codes); Lurton, *An Introduction to Underwater Acoustics*.
