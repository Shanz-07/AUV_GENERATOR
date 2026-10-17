# PAANCHAJANYA — Adaptive Software-Defined Sonar Transmitter for AUVs

**Smart India Hackathon 2026 | Problem Statement: SIH26058**

A low-power, real-time, STM32-based software-defined sonar transmitter that adapts its waveform and transmission parameters to estimated underwater conditions.

**Team:** PAANCHAJANYA  
**Theme:** Robotics and Drones  
**Category:** Hardware

---

## Overview

Autonomous Underwater Vehicles (AUVs) operate in changing underwater environments where temperature, salinity, depth, turbidity, and target range influence sonar transmission requirements.

Conventional fixed-parameter sonar transmitters use predetermined frequency bands and pulse settings, which may not be optimal across different operating conditions.

**PAANCHAJANYA** proposes an adaptive sonar transmitter payload that estimates suitable transmission parameters using an underwater acoustic model and generates configurable sonar waveforms using an STM32 microcontroller.

The current Minimum Viable Prototype (MVP) demonstrates digital waveform generation, timer-triggered DAC streaming, real-time parameter selection, and waveform analysis. The design is intended to scale to a higher-frequency external DAC and a dedicated acoustic transmission stage for integration into an AUV.

## Key Features

- **Adaptive transmission:** Calculates frequency, bandwidth, pulse duration, and amplitude from environmental inputs and estimated acoustic losses.
- **Multiple waveform types:** Linear Frequency Modulation (LFM), Geometric FM, Hyperbolic FM (HFM), and Barker-13 phase-coded pulses.
- **Digital waveform synthesis:** DDS-based generation with configurable pulse shaping.
- **Low-CPU streaming:** TIM6-triggered DAC output using DMA and double buffering.
- **Configurable windows:** Tukey, Hann, Hamming, and Blackman window functions.
- **Environmental simulation:** Five potentiometers represent turbidity, temperature, depth, salinity, and target range.
- **Waveform analysis:** Python tools for serial acquisition, time-domain plots, instantaneous-frequency analysis, FFT, spectrograms, and matched-filter analysis.
- **Energy-aware design:** Transmission amplitude is selected according to the estimated link loss, with low-power operation between pings.

## System Architecture

```text
Environmental Inputs
  Temperature | Salinity | Depth
  Turbidity   | Target Range
              |
              v
        ADC Acquisition
              |
              v
       STM32F446RE MCU
  Environmental Model and Adaptation
              |
              v
       DDS Waveform Engine
  LFM | Geometric FM | HFM | Barker-13
              |
              v
    TIM6-Triggered DAC + DMA
       Double-Buffered Output
              |
              v
   Analog Reconstruction Filter
              |
              v
   External Driver and Transducer
       (Final Payload — Planned)
```

The current MVP implements the digital waveform-generation and DAC-output stages. The external high-voltage driver, matching network, and broadband transducer belong to the planned final payload.

## Hardware and Software

| Component | Implementation |
|---|---|
| Microcontroller | STM32F446RE / NUCLEO-F446RE |
| Firmware language | C |
| Development environment | STM32CubeIDE |
| Waveform generation | Direct Digital Synthesis (DDS) |
| Digital-to-analog conversion | Internal 12-bit DAC |
| Sampling rate | 1 MS/s |
| Timer and data transfer | TIM6 + DMA |
| Environmental inputs | Five potentiometers connected to ADC inputs |
| Serial interface | UART over ST-LINK virtual COM port |
| Analysis tools | Python, NumPy, SciPy, Matplotlib, PySerial |
| Circuit and payload design | Autodesk Fusion 2026 |

## MVP Specifications

| Parameter | Current MVP |
|---|---|
| Demonstrated output band | 20–50 kHz |
| DAC resolution | 12-bit |
| DAC sampling rate | 1 MS/s |
| Windowed burst duration | Approximately 0.5–7 ms |
| Waveform modes | LFM, Geometric FM, HFM, Barker-13 |
| Window functions | Tukey, Hann, Hamming, Blackman |
| Streaming architecture | Timer-triggered DAC + DMA |
| Environmental inputs | Five potentiometers |
| Output verification | Digital oscilloscope and Python analysis |

The 20–50 kHz band is a scaled-down MVP implementation. The adaptive algorithm is designed around a wider target payload band of 100–500 kHz.

## Repository Structure

```text
AUV_GENERATOR/
├── Core/
│   ├── Inc/                  # Firmware headers
│   └── Src/                  # STM32 firmware source
├── Drivers/                  # STM32 device and HAL drivers
├── Debug/                    # Build/debug artifacts
├── .settings/                # IDE configuration
├── auv.ioc                   # STM32CubeMX configuration
├── auv1.ioc                  # Alternate CubeMX configuration
├── main_py.c                 # Main firmware implementation
├── dac_waveform.py           # Serial waveform acquisition and analysis
├── dac_waveform_2.py         # Additional waveform analysis implementation
├── requirements.txt          # Python dependencies
├── STM32F446RETX_FLASH.ld    # Flash linker script
└── STM32F446RETX_RAM.ld      # RAM linker script
```

## Getting Started

### 1. Firmware Setup

**Requirements**

- STM32CubeIDE
- NUCLEO-F446RE development board
- USB connection through ST-LINK
- Potentiometers for simulated environmental inputs
- Digital oscilloscope for output verification

**Steps**

1. Clone the repository:

   ```bash
   git clone https://github.com/Shanz-07/AUV_GENERATOR.git
   ```

2. Open STM32CubeIDE and import the project from the cloned directory.
3. Select the NUCLEO-F446RE target and review the project configuration.
4. Build the firmware and flash it to the board.
5. Connect the DAC output to an oscilloscope to inspect the generated waveform.

### 2. Python Analysis Setup

Install Python 3 and the required packages:

```bash
pip install matplotlib pyserial numpy scipy
```

Connect the development board through USB and identify its virtual COM port.

List available serial ports:

```bash
python dac_waveform.py --list
```

Start live waveform analysis, replacing `COM5` with your actual port:

```bash
python dac_waveform.py --port COM5
```

Capture one frame and save the waveform data and plot:

```bash
python dac_waveform.py --port COM5 --once
```

Record the raw serial stream for later analysis:

```bash
python dac_waveform.py --port COM5 --log run.bin
```

Replay a recorded capture:

```bash
python dac_waveform.py --replay run.bin
```

The firmware and analyzer use a serial baud rate of 921600 by default. Close other serial-terminal applications before starting the analyzer, because the COM port can only be accessed by one application at a time.

## Adaptive Algorithm

The firmware uses environmental inputs and an underwater acoustic model to determine transmission parameters.

The intended decision process is:

1. Acquire and smooth the environmental ADC readings.
2. Estimate sound speed and acoustic absorption using temperature, salinity, and depth.
3. Estimate the transmission loss for the specified target range.
4. Select a suitable carrier frequency under the configured loss constraint.
5. Calculate bandwidth, pulse duration, and transmission amplitude.
6. Select the waveform and window function.
7. Generate the pulse and stream its samples through the DAC using timer-triggered DMA.
8. Enter a low-power state between transmissions where supported by the configured firmware.

The current turbidity-dependent attenuation model is an estimate and remains subject to tank calibration with real turbidity measurements.

## Waveform Verification

The Python analyzer processes samples streamed from the actual DAC rather than relying only on the intended digital waveform parameters.

The submitted prototype report records the following results:

| Measurement | Reported result |
|---|---:|
| Captured frames | 600+ |
| CRC errors | 0 |
| Clipped samples | 0 |
| FM out-of-band level | Approximately −63 dB |
| Barker-13 peak sidelobe level | Approximately −21.8 dB |

These results describe the submitted MVP analysis. The reported out-of-band level applies to the tested FM waveform and should not be interpreted as a universal result for every waveform or the final acoustic payload.

## Current Status and Future Scope

### Implemented and demonstrated

- STM32F446RE-based waveform-generation MVP.
- Internal DAC output at 1 MS/s.
- Timer-triggered DMA streaming with double buffering.
- LFM, Geometric FM, HFM, and Barker-13 waveform generation.
- Four configurable window functions.
- Potentiometer-based environmental input simulation.
- Serial capture and Python-based waveform analysis.
- CAD concept for the proposed AUV payload.

### Planned for the final payload

- External DAC supporting at least 10 MS/s for the target 100–500 kHz band.
- Higher-performance controller, such as an STM32H7-class MCU or FPGA, if required by the final implementation.
- Real temperature, salinity, depth, and turbidity sensors.
- Experimental calibration of the turbidity-dependent acoustic attenuation model.
- Reconstruction filtering, high-voltage linear amplification, impedance matching, and a broadband piezocomposite transducer.
- Pressure-rated enclosure and integration interfaces for an AUV.
- Measured power-consumption and energy-per-ping validation.

The final payload requires further hardware integration and underwater testing. The current MVP demonstrates the digital generation and analysis approach, rather than a fully validated underwater sonar system.

## Applications

Potential application areas include:

- AUV-based seabed surveying and mapping.
- Marine research and environmental monitoring.
- Underwater infrastructure inspection.
- Harbour surveying and defence-related underwater sensing.
- Adaptive sonar transmission in environments with changing acoustic conditions.

These are intended application areas; field performance will depend on the completed transmitter, transducer, receiver, and validation process.

## References

1. STMicroelectronics — *STM32F446xC/E Datasheet and RM0390 Reference Manual*.
2. STMicroelectronics — *AN3126: Audio and Waveform Generation Using the STM32 DAC*.
3. M. A. Ainslie and J. G. McColm — “A simplified formula for viscous and chemical absorption in sea water,” *The Journal of the Acoustical Society of America*, 103(3), 1671–1672, 1998.
4. E. Demirors et al. — “Software-defined underwater acoustic networks: toward a high-rate real-time reconfigurable modem,” *IEEE Communications Magazine*, 53(11), 2015.
5. F. J. Harris — “On the use of windows for harmonic analysis with the discrete Fourier transform,” *Proceedings of the IEEE*, 66(1), 51–83, 1978.
6. H. Medwin — “Speed of sound in water: a simple equation for realistic parameters,” *The Journal of the Acoustical Society of America*, 58(6), 1318–1319, 1975.
7. R. E. Francois and G. R. Garrison — “Sound absorption based on ocean measurements, Part II,” *The Journal of the Acoustical Society of America*, 72(6), 1879–1890, 1982.
8. S. D. Richards et al. — “Suspended-particle attenuation in seawater,” *The Journal of the Acoustical Society of America*, 100(3), 1447–1450, 1996.
9. J. J. Kroszczynski — “Pulse compression by means of linear-period modulation,” *Proceedings of the IEEE*, 57(7), 1260–1266, 1969.
10. R. H. Barker — “Group synchronizing of binary digital systems,” *Communication Theory*, Butterworths, 1953.
11. X. Lurton — *An Introduction to Underwater Acoustics*, 2nd edition, Springer, 2010.

## Project Information

- **Team:** PAANCHAJANYA
- **Event:** Smart India Hackathon 2026
- **Problem Statement ID:** SIH26058
- **Repository:** [github.com/Shanz-07/AUV_GENERATOR](https://github.com/Shanz-07/AUV_GENERATOR)

---

*PAANCHAJANYA — Adaptive, low-power, software-defined sonar transmission for autonomous underwater vehicles.*
