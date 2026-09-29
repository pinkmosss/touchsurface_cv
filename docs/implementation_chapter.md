# 4 Implementation

> Converted from *MaryamThesis_2026.pdf*, chapter 4, pages 31–52. Original wording, numbering, and citation references are retained; PDF line wrapping and typography are normalized.

This chapter describes the implementation of TouchSurface CV. This module combines a playable Eurorack interface with an embedded capacitive-sensing system. The electrode geometry determines the measurements available for processing, while the resources of the ATmega328 constrain how these measurements can be handled. At the same time, latency and synchronization requirements affect both circuit and firmware design. The following sections therefore focus on these interactions, with particular attention to ADC usage, interrupt timing, and memory constraints.

The final design was evolved through the experiments described in Chapter 3. It began with handmade sensors, followed by PCB test surfaces, and eventually leading to the hardware and firmware presented in this chapter. The prototypes were evaluated with their intended use as a musical instrument in mind. The touch surface needed to provide sufficient space for hand movement, patch cables had to remain clear of the playing area, and the voltage outputs needed to represent useful dimensions of the touch interaction. The design also needed to remain modifiable. Some implementation decisions are based on the prior work discussed in Chapter 2, while others result from practical limitations identified during prototyping.

## 4.1 Hardware Implementation

The hardware could be considered as a chain of signals from touch interaction to control-voltage output. Movement across the electrode matrix produces changes in mutual capacitance, which are measured by the sensing circuitry. It is then processed by the microcontroller, and converted into voltages at the module outputs. The hardware design therefore had to accommodate the sensing surface, processing and output circuitry within the available dimensions. The following subsections describe the hardware from the electrode matrix to the output stage, while Section 4.1.6 discusses the main design trade-offs.

The hardware architecture builds on the approaches discussed in Chapter 2. In particular, the Multi-Touch Kit [23] and the work of Grosse-Puppendahl et al. [12] provide the basis for implementing a configurable mutual-capacitive matrix using a general-purpose microcontroller. This sensing approach is combined with the voltage-based interface required for integration into a Eurorack system. TouchSurface CV therefore consists of a matrix-based capacitive sensing stage followed by embedded processing and four patchable control-voltage outputs.

### 4.1.1 Overall Hardware Architecture

The module is divided into a front board and a lower control board. The front board contains the elements used directly during operation, including the capacitive matrix, record and playback pads, four outputs, and trigger input. The control board contains the microcontroller, multiplexers, DAC, programming interface, and the power and signal-conditioning circuitry.

The two boards are connected by a 22-pin FPC connector, CN1. The connection carries TX_SIGNAL_0 to TX_SIGNAL_8, RX_SIGNAL_0 to RX_SIGNAL_9, the two mode-pad signals CAP1 and CAP2, and ground. The TX and RX signals are arranged in separate groups, with ground plane between them to avoid signal interference. The signal line for capacitive buttons are placed at the ends of the connector. The ground connection provides a common reference between the two boards. This arrangement also allows the touch-surface board to be replaced independently of the control electronics.

The module can receive power either through USB-C or from the Eurorack power supply. USB-C is primarily used during programming and bench testing, whereas the Eurorack header supplies the module when installed in a rack. Only the USB 2.0 portion of the USB-C interface is required. The duplicated VBUS contacts are connected to VBUS, while D+ and D- are connected to USB_DP and USB_DN. The SBU contacts are left unused. A 5.1 kOhm Rd resistor on each of the CC_1 and CC_2 pins identifies the board as a power sink in either connector orientation. Since the circuit operates from the default USB supply, USB Power Delivery negotiation is not required.

The USB interface also includes protection against unwanted current flow and electrical transients. The USB shield is connected to logic ground through a 100 nF capacitor in parallel with a 1 MOhm resistor. This provides a path for high-frequency interference while maintaining a defined DC potential between the shield and ground. VBUS reaches the local PWR_5V rail through a 1N5819WS Schottky diode. This prevents current from the Eurorack supply from feeding back into a connected USB host when both power sources are present. A 10 uF capacitor provides local supply buffering after the diode. An SMF05CT1G protection array is placed at the USB connector and provides transient protection for VBUS, USB_DN, USB_DP, CC_1, and CC_2.

![Figure 4.1: The module after production](implementation_2026_assets/figure-4-1.png)

*Figure 4.1: The module after production*

When installed in a rack, the module receives power through a 2 by 8 Eurorack header carrying ER_POS_12V, ER_NEG_12V, ER_POS_5V, and several ground connections. The +5 V rail supplies the digital circuitry, while the +12 V and -12 V rails are used by the analog section. In the schematic, the ER_* prefix distinguishes signals at the Eurorack interface from local board supplies such as PWR_5V. A second SMF05CT1G protection array is placed at this interface to protect ER_POS_12V, ER_POS_5V, ER_CV, and ER_GATE.

### 4.1.2 Touch Panel and Electrode Layout

The input surface is implemented as a mutual-capacitive matrix based on the open microcontroller approach of the Multi-Touch Kit [23]. Compared with the resistive and self-capacitive alternatives considered during development, this method allows light touch to be detected without requiring pressure and provides a two-dimensional field from which touch position can be estimated. Although the surface appears continuous to the user, the firmware processes it as a set of individual matrix cells.

Earlier concept studies considered a larger, diamond-shaped electrode layout following the geometry proposed by the Multi-Touch Kit. The touch panel provides eight transmit electrodes and nine receive electrodes. The current firmware scans eight receive channels, producing an active 8 × 8 sensing matrix with 64 cells. The resulting measurements are used to estimate horizontal position, vertical position, and a coarse measure of touch size. A higher matrix resolution would provide more spatial detail, but the 64-cell arrangement remains practical within the processing and memory limits of the ATmega328.

Because the matrix requires more signal lines than can be connected directly to the microcontroller, the transmit and receive electrodes are selected through two CD74HC4067 analog multiplexers. Four digital address lines control each multiplexer, allowing the complete matrix to be scanned with a limited number of microcontroller pins. On the transmit side, Timer2 generates the TX_PWM excitation signal, which is routed through the multiplexer to one transmit electrode at a time. The receive multiplexer selects the corresponding receive electrode and connects it to the ATmega328 ADC through the RX_ADC node.

Multiplexing introduces an additional consideration during measurement. After switching channels, residual charge from the previous selection can influence the next ADC reading if the conversion is performed immediately. The acquisition routine therefore allows the receive path to settle and discards an initial conversion before accepting the measured value. A 100 kOhm resistor weakly pulls the receive node toward ground, helping residual charge dissipate while keeping the loading of the capacitive signal low.

![Figure 4.2: Touch Panel Hardware Design](implementation_2026_assets/figure-4-2.png)

*Figure 4.2: Touch Panel Hardware Design*

The enable inputs of both multiplexers are held in their active state by 10 kOhm pull-down resistors. This prevents the enable pins from floating during reset and ensures that the multiplexers begin in a defined state before the firmware configures the associated microcontroller pins. Electrode selection itself remains under firmware control throughout the scan.

### 4.1.3 Control Board and Microcontroller

The control board is built around the ATmega328, which coordinates the main sensing and control functions of the module. It selects the TX and RX multiplexer channels. After reading the capacitive measurements through the ADC, it processes completed frames. It also updates the MCP4728 DAC over I²C and handles the capacitive mode controls and external trigger input. The same board includes the USB-to-serial interface and the connections required for initial programming.

![Figure 4.3: Touch panel 3D view](implementation_2026_assets/figure-4-3.png)

*Figure 4.3: Touch panel 3D view*

The ATmega328 was selected mainly for its low cost, availability, and compatibility with the Arduino development environment and the Multi-Touch Kit [23]. A larger device such as the ATmega2560 would provide additional RAM and I/O pins and could reduce some of the multiplexing required by the design. However, its substantially higher cost was difficult to justify for the intended platform. The limitations of the ATmega328, particularly its 2 KB of SRAM, single ADC, and limited timer resources, therefore became important constraints on the firmware architecture and are discussed in the following sections.

The surface-mount ATMEGA328-AU in a TQFP package is used to keep the control board compact. Both VCC and AVCC are supplied from PWR_5V, and the analog reference is decoupled to ground with a 100 nF capacitor. Since the ADC is used for the capacitive matrix and the additional touch controls, maintaining a stable analog reference is important for consistent measurements.

The microcontroller is clocked by an external 16 MHz crystal with two 15 pF load capacitors. The external clock provides a stable timing reference for serial communication, PWM generation, ADC acquisition, and the timing routines used by the firmware. Reset circuitry follows the common Arduino arrangement: a 10 kOhm resistor holds the reset line high, while a 100 nF capacitor couples the DTR signal from the USB-to-serial converter to the reset input. This produces the short reset pulse required when starting a firmware upload.

![Figure 4.4: Component Panel Schematic](implementation_2026_assets/figure-4-4.png)

*Figure 4.4: Component Panel Schematic*

A blank ATmega328 is initially programmed through the 2 by 3 CN2 ISP header, which provides SCK, RESET, CIPO, COPI, PWR_5V, and ground. This interface is used to install the bootloader and configure the device fuses. It also remains available as a recovery interface if the bootloader can no longer be reached through the normal serial connection.

![Figure 4.5: Bootloader hardware schematic.](implementation_2026_assets/figure-4-5.png)

*Figure 4.5: Bootloader hardware schematic.*

Once the bootloader is installed, firmware can be uploaded through USB using the standard Arduino-compatible serial workflow. Since the ATmega328 has no native USB peripheral, an FT232RNL converts the USB connection into the UART signals required by the bootloader. On the USB side, it connects to USB_DP and USB_DN; on the microcontroller side, FT_TXD is connected to the ATmega328 receive input on PD0 and FT_RXD to the transmit output on PD1. The two UART lines include 1 kOhm series resistors, which provide limited current protection and isolation in the event of unintended pin contention.

![Figure 4.6: USB and ESD Protection Hardware Schematic](implementation_2026_assets/figure-4-6.png)

*Figure 4.6: USB and ESD Protection Hardware Schematic*

The FT232RNL uses USB VBUS on its USB side, while its separate VCCIO supply sets the logic level of the UART and control signals. VCCIO is therefore connected to the same 5 V logic domain as the ATmega328. The FT232’s internal 3.3 V output is locally decoupled with 100 nF but is not used to power the module. Two LEDs connected to the FT232 CBUS pins provide a simple indication of serial activity during programming and debugging.

The remaining ATmega328 pins are assigned according to the sensing and control requirements. PD3 generates TX_PWM, while four address lines are used for each of the RX and TX multiplexers. PC3 is used as the RX_ADC input, and PC1 and PC2 connect to the capacitive mode controls CAP1 and CAP2. PC4 and PC5 provide the DAC_SDA and DAC_SCL connections to the MCP4728, while the external trigger is connected to the interrupt-capable PD2 pin. These assignments allow the sensing, DAC communication, trigger handling, and user controls to coexist within the limited I/O resources of the device.

Local supply decoupling is provided by 100 nF and 10 uF capacitors between PWR_5V and ground. The smaller capacitor supplies short switching currents close to the microcontroller, while the larger capacitor supports slower changes in load. The control board also includes a power indicator, a firmware-controlled debug LED, and two test points that provide access to internal signals during measurements of scan timing and state transitions.

### 4.1.4 Control-Voltage Outputs

The module uses an MCP4728 DAC to convert the processed touch data into four analog control signals. Its four 12-bit channels represent the X position, Y position, local touch size, and touch presence, with each channel using the available range of 0 to 4095. The X and Y outputs follow the dominant position detected on the matrix, while touch size is estimated from the number of nearby active cells. The fourth channel represents touch presence and is used as a gate signal. Using a single four-channel DAC keeps the interface compact and requires only the two I²C connections to the microcontroller.

![Figure 4.7: Component panel hardware design](implementation_2026_assets/figure-4-7.png)

*Figure 4.7: Component panel hardware design*

Communication with the MCP4728 takes place over DAC_SCL and DAC_SDA, with both lines pulled up to PWR_5V through 4.7 kOhm resistors. The LDAC and RDY/BSY signals are also available to the control circuit, allowing synchronized updates and monitoring of the converter state. The DAC provides the link between the processed touch measurements and the analog voltage interface used by the Eurorack system.

Each DAC output is passed through a non-inverting amplifier before reaching the corresponding output jack. The four amplifier stages are implemented using two dual op-amp packages. Equal 10 k resistors in each feedback network set a nominal gain of two. The output stage was intended to scale the DAC signal to a 0–10 V control-voltage range. However, a component-selection error prevented validation of this intended output range, as described in Section 4.1.6.

The four control voltages are connected to 3.5 mm PJ-3411 through-hole jacks. These connectors have a TRS construction and are used as conventional Eurorack connections. Their through-hole construction also provides mechanical support for repeated patching at the front panel.

A 100 Ohm series resistor is placed between each amplifier output and its corresponding jack. With the relatively high input impedance of typical Eurorack modules, this introduces little voltage loss during normal operation. Capacitors around the output provide filtering and help suppress high-frequency disturbances at the connector. Together, these components provide a protected interface between the DAC circuitry and the external patch.

![Figure 4.8: DAC and analog output-stage schematic of the fabricated prototype. The MCP6002 devices shown are unsuitable for the intended +12 V supply.](implementation_2026_assets/figure-4-8.png)

*Figure 4.8: DAC and analog output-stage schematic of the fabricated prototype. The MCP6002 devices shown are unsuitable for the intended +12 V supply.*

Updating the DAC also has an effect on firmware timing. Sending all four channels over I²C for every matrix frame would introduce unnecessary communication overhead. The firmware therefore keeps track of the previous output values and sends new values only when an output has changed.

### 4.1.5 Trigger Input and Capacitive Control Pads

The trigger input allows recorded touch gestures to be replayed in synchronization with an external clock. This is the key Eurorack-specific input of the module. In modular systems, timing is distributed as trigger and gate pulses, and modules are expected to respond to those pulses directly rather than to an internal tempo. The input is connected to PD2 on the ATmega328, which supports external interrupt 0. The trigger input uses the same PJ-3411 3.5 mm connector family as the CV outputs. Reusing the same connector type also keeps the front-panel mechanics consistent.

Since Eurorack trigger signals can exceed the safe range for the microcontroller and could also contain negative voltage excursions, the signal is conditioned before it reaches PD2. The firmware therefore receives a logic-level signal with a defined idle state and only handles edge detection and synchronization. Protection against negative voltages is implemented in hardware.

Two 100 nF capacitors suppress high-frequency disturbances at the jack, after which the signal passes through a 1N5819WS Schottky diode and a 10 kOhm series resistor. A BZT52C5V1 5.1 V Zener diode then limits the voltage at the TRIG node to approximately the logic range of the ATmega328. This protection is necessary before the signal reaches the microcontroller, for protection against voltage exceeding the safe input range.

![Figure 4.9: The trigger input hardware design.](implementation_2026_assets/figure-4-9.png)

*Figure 4.9: The trigger input hardware design.*

The module concept also includes two capacitive control pads for record and playback. These capacitive electrodes use the ADCTouch approach instead of mechanical switches. Hardware minimalism is the motivation for this choice. Each control requires only one electrode and one wire, with no mechanical switch, which keeps the front panel clean and the parts cost negligible. The cost appears on the software side instead, which is discussed in length in the software implementation section. These pads connect to A2 and A1 pins of the microcontrollers. Since the ATmega328 provides only one physical ADC, the matrix and the two control pads cannot be sampled at the same time. The firmware therefore reads the mode controls between matrix scans, allowing the three inputs to share the same converter without additional hardware.

### 4.1.6 Hardware Challenges and Cut-Offs

Several hardware decisions were made as deliberate cut-offs rather than ideal solutions. Stating them explicitly clarifies the scope of the implementation and marks the points where a future revision could diverge.

The ATmega328 establishes the main boundary of the prototype. It keeps the platform inexpensive and familiar, yet limits memory, pins, ADC access, and processing time. A larger controller would simplify the code but would move the design away from its accessibility objective.

Another cut-off is multiplexed matrix sensing on a commodity microcontroller rather than a dedicated high-resolution touch-controller chip. As discussed in Chapter 2, dedicated chips such as the Cypress CY8C28645 or Microchip MTCH6301 offer sophisticated sensing but are difficult to program, may require complex analog support circuitry, and constrain the electrode layout. The Multi-Touch Kit approach keeps the sensing stack open and adaptable, at the price of firmware complexity.

A component-selection error was identified in the analog output stage late in development. The fabricated design specifies MCP6002 operational amplifiers supplied by the Eurorack +12 V rail. However the MCP6002 has a specified operating supply range of 1.8–6.0 V and is therefore unsuitable for this supply configuration. Consequently, the output stage cannot be considered operational under the intended supply conditions, and the proposed 0–10 V output range was not demonstrated.

The error was identified too late to complete a hardware correction and its validation before submission. The remaining evaluation therefore focused on the capacitive sensing and firmware subsystems. The intended voltage-output architecture is documented as part of the prototype design, but successful operation of the complete touch-to-voltage signal path is not claimed.

A hardware revision requires an operational amplifier suitable for the intended supply voltage and output load. Following this correction, the output stage must be tested for voltage range, and behavior under load.

## 4.2 Software Implementation

This chapter describes the software implementation of the touch interface developed in this project. The implementation runs on an AVR-based Arduino platform and combines capacitive matrix acquisition, touch-state estimation, control-voltage generation, and sequence recording and playback. The main objective of the software is to convert the raw response of the touch matrix into stable and musically useful output signals while remaining responsive to external trigger events and user interaction.

The sensing approach and the starting point of the Arduino library are derived from the open-source Multi-Touch Kit by Pourjafarian et al. [23]. The original project introduced a mutual-capacitance sensing technique for customised touch surfaces using a commodity microcontroller. For this project, the library was substantially adapted to the fixed hardware configuration and to the real-time requirements of a control-voltage instrument. The resulting firmware is divided into two layers.

The `MultiTouchKit` library implements the time-critical acquisition of the capacitive sensor matrix. The main application in `thesis.ino` operates on completed sensor frames and implements calibration, touch detection, dominant-touch selection, DAC output, and the record/playback functions. This separation keeps the interrupt service routines small and moves the more complex processing into the foreground loop. It also separates hardware-specific acquisition from the application behaviour built on top of it.

The implementation was designed for a resource-constrained microcontroller. All buffers are statically allocated, integer arithmetic is used throughout, and repeated peripheral writes are avoided. No dynamic memory allocation is required during operation. These decisions make the timing and memory use predictable, which is important in an embedded musical interface where delayed or irregular responses are directly noticeable to the user.

### 4.2.1 Library Refactoring: From High-Level Prototyping to Low-Level AVR Optimization

The sensor acquisition code is based on the open-source Multi-TouchKit library developed by the HCI Lab at Saarland University [23]. The original library was designed for rapid prototyping with boards such as the Arduino Uno, Mega 2560, and LilyPad. It uses standard Arduino functions, including `digitalWrite()`, `pinMode()`, and `analogRead()`. This makes the library easy to understand and configure for different prototypes. However, the original implementation did not directly match the hardware or timing requirements of TouchSurface CV. It was therefore used as a starting point and substantially modified for this project.

The first difference concerns the sensor connections. The original library uses one multiplexer to select the transmitting electrode, while the receiver electrodes are connected directly to separate analogue inputs on the Arduino. TouchSurface CV instead uses two CD74HC4067 multiplexers: one for the TX electrodes and one for the RX electrodes. This design was chosen to accommodate the limited number of pins available on the ATmega328P. The ATmega328P was selected because it was less expensive than the ATmega2560 and still provided sufficient processing capability for the instrument. Using a second multiplexer allows all RX signals to share a single ADC input on pin A3, thereby reducing the number of microcontroller pins required. The library therefore had to be extended to control both multiplexers.

The second difference concerns how a frame is acquired. In the original library, the `read()` function performs all tasks in sequence. The function does not return until the complete operation has finished. This behaviour was not suitable for the instrument because the main loop must also update the DAC, process the control pads, and react to external trigger pulses. The modified library therefore separates sensor acquisition from touch processing and communication. The library only collects raw ADC readings and stores them in a matrix. The main application processes this matrix after a frame has been completed.

The multiplexer control was also changed. The original implementation selects a channel with four calls to `digitalWrite()`. These calls are convenient but perform several internal steps to translate an Arduino pin number into the corresponding register. In the modified library, the multiplexer address lines are changed directly through the AVR `PORTB` and `PORTD` registers. This reduces the work required for every channel change. The receiver ADC pin is also controlled directly through `DDRC` and `PORTC`.

The original MultiTouchKit library already uses Timer 2 to generate the high-frequency TX excitation signal. This principle was retained in the modified version. On the ATmega328P, Timer 2 produces a 4 MHz signal with a 25% duty cycle on pin D3. The signal is generated by the timer hardware and therefore does not require continuous work from the main program.

The main change is the introduction of interrupt-driven acquisition. Timer 1 regularly requests the start of a new matrix scan. The ADC then measures the cells in the background. After each ADC conversion, an interrupt advances the scanner to the next receiver or transmitter position. The main loop can continue with other work between these interrupts instead of waiting inside `analogRead()`.

The receive input is sensitive to charge left by the previously selected channel. To reduce this effect, the modified library discharges RX lines before measuring a new cell. It then performs two ADC conversions. The first conversion is discarded and provides additional settling time. The second conversion is stored in the touch matrix. The discharge routine still contains short microsecond delays while stepping through the RX channels, but the longer ADC waiting period is handled asynchronously.

In summary, the modified library retains the sensing principle, PWM excitation, and basic transmitter-selection approach of the original MultiTouchKit project. The acquisition architecture, however, was changed for the instrument. The new version supports a second multiplexer, uses direct AVR register access, separates acquisition from Serial output, and scans the matrix through Timer and ADC interrupts. These changes allow the main application to perform touch detection, DAC updates, recording, playback, and trigger handling while sensor acquisition continues in the background.

### 4.2.2 Software Architecture

This section describes in more details the two layer implementation of the software. MultiTouchKit is the low-level sensor driver. It configures the transmit signal, the TX and RX multiplexers, the ADC, Timer1, Timer2, and the interrupt service routines. It also scans the matrix asynchronously in the background, and stores completed frames. thesis.ino is the application layer. It reads completed frames and tracks baselines. After detecting active cells, it then identifies the dominant touch and maps touch state to DAC outputs. Recording samples and replaying recorded data, based on the state of control pads are also handled by this layer.

![Figure 4.10: High level system context and data flow](implementation_2026_assets/figure-4-10.png)

*Figure 4.10: High level system context and data flow*

Conceptually, the library produced a frame and then the firmware translates this frame into related functionalities. The library acquires frames using timer and ADC interrupts. Once a complete frame is available, the application starts processing it. The main loop therefore follows a simple pattern. If a frame is ready the firmware interprets the latest matrix values and updates the system states.

The firmware can be understood as a pipeline with four main stages:

1. The sensor library continuously scans the transmitter and receiver combinations and stores the ADC readings in a matrix.

2. The main application consumes each completed frame, compares it with a per-cell baseline, and constructs a binary map of active cells.

3. A dominant touch position and its local cluster size are extracted from the active-cell map.

4. The resulting values are either sent directly to the DAC or replaced by values read from the playback buffer.

The acquisition and application layers communicate through a shared frame buffer and a frame counter. Timer and ADC interrupts perform the scanning, while the Arduino `loop()` function only processes a frame after `consumeFrame()` reports that a complete frame is available. The interrupt routines therefore remain responsible for deterministic hardware sequencing, whereas touch interpretation and interface state are handled outside interrupt context.

The component choices and signal assignments are collected in the following table.

**Table 4.1: Implementation choices for the controller hardware**

| Component or signal | Implementation choice |
| --- | --- |
| Microcontroller | ATmega328-AU, Arduino-compatible 5 V MCU |
| MCU clock | External 16 MHz crystal with two 15 pF load capacitors |
| MCU reset | 10 kΩ pull-up to PWR_5V; 100 nF DTR coupling capacitor for auto-reset |
| Bootloader / programming header | 2 × 3 AVR ISP header with SCK, RESET, CIPO, COPI, PWR_5V, and ground |
| Active matrix | 8 TX × 8 active RX cells (64 cells) |
| Available RX channels | 9 RX channels; RX9 is skipped in the current firmware |
| Touch-panel connector | 22-pin FPC connector carrying TX, RX, CAP1, CAP2, and ground to the touch-surface PCB |
| TX excitation output | Timer2 PWM on D3, routed through the TX multiplexer |
| TX multiplexer | CD74HC4067; TX_PWM routed to TX_SIGNAL_0 through TX_SIGNAL_15 |
| RX multiplexer | CD74HC4067; RX_SIGNAL_0 through RX_SIGNAL_15 routed to RX_ADC |
| Multiplexer enable state | 10 kΩ pull-down resistors keep the enable pins active during reset and scanning |
| TX multiplexer select pins | D9, D10, D11, and D12; TX_SEL0 through TX_SEL3 |
| RX multiplexer select pins | D5, D6, D7, and D8; RX_SEL0 through RX_SEL3 |
| Matrix ADC input | A3/RX_ADC, with a 100 kΩ pull-down resistor to ground |
| Optional record/playback pads | A2 and A1 via ADCTouch |
| Trigger input | D2/PD2/external interrupt 0, externally conditioned |
| DAC | MCP4728, providing four 12-bit channels over I²C |
| DAC I²C pull-ups | 4.7 kΩ from DAC_SCL and DAC_SDA to PWR_5V |
| DAC control/status pins | DAC_LDAC for synchronized update control and DAC_RDY for ready/busy status |
| CV output gain stage | Four non-inverting op-amp channels, with 10 kΩ feedback and 10 kΩ to ground, giving a gain of 2 |
| Output op-amp supply constraint | The +12 V output-stage supply requires an op-amp rated for that supply rail |
| CV output connectors | PJ-3411 through-hole 3.5 mm TRS jacks used as unbalanced Eurorack outputs |
| CV jack conditioning | 100 Ω series resistor per output, 100 nF jack-side capacitors, and a 10 µF board-side capacitor |
| Trigger connector | PJ-3411 through-hole 3.5 mm TRS jack used as an unbalanced Eurorack trigger input |
| Trigger input conditioning | 100 nF jack-side capacitors, a 1N5819WS Schottky path, a 10 kΩ series resistor, and a BZT52C5V1 Zener clamp |
| USB-to-serial bridge | FT232RNL; USB data on USB_DP/USB_DN and UART on FT_TXD/FT_RXD |
| USB serial logic level | FT232 VCCIO matched to the 5 V AVR logic domain |
| UART protection/isolation | 1 kΩ series resistors on the FT232 TXD and RXD paths |
| FT232 indicators | Two red LEDs on CBUS pins, each with a 330 Ω series resistor |
| Debug indicators | Power LED and DEBUG1 LED, each with a 330 Ω series resistor |
| Test points | TEST1 and TEST2 exposed as probe points |
| USB power and data connector | USB-C 16-pin receptacle; CC_1 and CC_2 are each pulled down with 5.1 kΩ |
| USB 5 V path | VBUS through a 1N5819WS Schottky diode to PWR_5V, with a 10 µF local bulk capacitor |
| USB shield reference | 100 nF capacitor and 1 MΩ resistor from the shield to ground |
| USB ESD protection | SMF05CT1G on VBUS, USB_DN, USB_DP, CC_1, and CC_2 |
| Eurorack power connector | 2 × 8 header carrying ER_POS_12V, ER_NEG_12V, ER_POS_5V, ground, ER_CV, and ER_GATE |
| Eurorack ESD protection | SMF05CT1G on ER_POS_12V, ER_POS_5V, ER_CV, and ER_GATE |

### 4.2.3 Matrix Scanning with the Modified MultiTouchKit Library

MultiTouchKit performs matrix acquisition with two timers, the multiplexers, and the ADC interrupt, following the general scanning strategy of Multi-Touch Kit [23] adapted to this hardware. Timer2 generates the TX waveform on D3 as high-frequency PWM. Timer1 acts purely as a frame scheduler: it is configured in compare-match mode to produce roughly 250 frame-start opportunities per second on the 16 MHz AVR, and its interrupt calls the frame-start routine.

A frame begins by selecting the first TX/RX pair and discharging the receive paths. The ADC pin is briefly driven low as the RX multiplexer steps through its channels, then returned to input mode. One conversion is discarded after each selection; the next is stored. This dummy conversion supplies settling time without a blocking delay and reduces contamination by charge left from the preceding channel.

The ADC interrupt advances through RX and then TX, producing 128 conversions for the 64 cells. After the last valid value, it marks the frame complete and increments a counter. Because the AVR architecture disables nested interrupts by default, the Timer1 and ADC ISRs cannot preempt each other, ensuring the state machine transitions sequentially and safely without requiring additional software locks within the interrupt context. Interrupt routines do no interpretation or communication: they move these small state machines and counters only. Serial output, I²C, DAC updates, and touch analysis remain in the main loop; the trigger ISR is subject to the same restriction.

### 4.2.4 Frame Synchronization

The application synchronizes with the scanner through consumeFrame(). Rather than simply checking whether a new frame is available, this function also marks that frame as consumed. This ensures that each completed sensor frame is processed only once by the main loop.

The need for explicit frame synchronization became clear during debugging. After the sensing code was moved into the library, adding a diagnostic serial print made the interface feel more responsive. The print itself was not improving the sensor data; instead, the additional delay changed the timing of the main loop. Because serial output on the AVR is relatively slow, it gave the interrupt-driven scanner enough time to complete a new frame before the next application pass.

This revealed that the main loop was running faster than the sensor frame rate, and could therefore process unchanged data more than once. The solution was to use frame availability as the condition for touch processing rather than relying on incidental delays. For this reason, `consumeFrame()` is checked before the application interprets the sensor data.

### 4.2.5 Touch Detection and Baseline Tracking

Once a new frame is available, the firmware evaluates the 64 active cells of the matrix. Each cell has a baseline representing its untouched state, from which a positive touch strength is calculated. The active state of the matrix is stored as a bitset to reduce memory use on the ATmega328.

At startup, each baseline is calculated from 16 measurements while the surface is assumed to be untouched. During operation, the raw reading is compared with this baseline and adjusted according to the configured touch polarity, since a touch may either increase or decrease the ADC value depending on the sensing configuration. Negative results are clamped to zero.

To compensate for gradual changes in the capacitive measurements, the baselines are updated using a lightweight exponential moving average. This update is applied only to inactive cells; otherwise, a held touch would gradually be incorporated into the baseline and become less detectable.

The active state is rebuilt for every frame, while the previous state is retained for hysteresis. This prevents released touches from remaining in the detected state, an issue observed in an earlier implementation. An inactive cell becomes active when its touch strength exceeds the threshold of 100, while an active cell remains active until the value falls below the release threshold of 30. Using separate thresholds reduces flickering when measurements fluctuate near the detection boundary.

### 4.2.6 Dominant Touch and Size

Although the detector supports multiple simultaneous active cells, the module’s musical output contains only the coordinate of the strongest active cell. The dominant touch is the active cell with the largest delta from baseline. The size estimate counts active cells in the 3 by 3 neighborhood around that cell.

This is a deliberate simplification. Full multi-touch tracking would require more memory, and a more complex recording and playback representation. A dominant touch plus a local cluster size maps naturally onto the output channels, and the size output preserves a useful trace of contact spread without a tracking algorithm.

### 4.2.7 DAC Output

An MCP4728 four-channel, 12-bit DAC converts the processed touch data into four output values. The channels are assigned as

**Table 4.2: Data to DAC Mapping**

| DAC channel | Software value | Meaning |
| --- | --- | --- |
| A | X | Dominant transmitter position |
| B | Y | Dominant receiver position |
| C | Size | Number of active pads |
| D | Touch flag | Binary indication of touch presence |

The X axis has eight positions, indexed from 0 to 7, also the Y axis has eight active positions. Each coordinate is mapped linearly to the DAC range from 0 to 4095. The local size is mapped from 0 to 9 over the same DAC range. The fourth channel is set directly to either 0 or 4095. When no dominant touch is present, all coordinate and size outputs are set to zero and the touch flag is low. The DAC is accessed over I²C at 400 kHz. All four channel values are sent together using `fastWrite()`. The last successfully transmitted values are cached, and a new I²C transfer is performed only when at least one channel has changed.

### 4.2.8 Recording and Trigger-Synchronized Playback

The recorder uses a fixed 512-byte circular buffer, with one byte for each sequence step. The upper four bits store the TX coordinate and the lower four bits store the physical RX coordinate. The value `0xFF` represents a step without a touch. This compact format provides up to 512 recorded steps while using little SRAM.

Only the dominant touch position and no-touch states are recorded. Touch strength and cluster size are not stored. During playback, the recorded X and Y coordinates are restored and the touch flag is set accordingly, but the size output remains zero. When the buffer is full, new samples overwrite the oldest ones. Playback begins with the oldest retained sample, proceeds in the original order, and stops automatically after all stored steps have been played.

Recording and playback share the same clock. A rising edge on digital pin D2 adds a pending trigger pulse, with the counter limited to 255 to prevent overflow. If no external trigger is waiting, the firmware advances after a fallback interval of 500 ms. Starting recording or playback clears old trigger pulses and resets the clock so that events from the previous mode are not reused.

### 4.2.9 User-Control State Handling

Two capacitive control pads on ADC channels A2 and A1 toggle recording and playback. They are calibrated during start-up and sampled at 200 ms intervals. The two modes are mutually exclusive. Enabling playback disables record mode. A recording can only be toggled when playback is inactive.

These capacitive pads are implemented with ADCTouch library. Since ADCTouch and MultiTouchKit both use the single ADC, they cannot run concurrently: ADCTouch reconfigures ADC registers and performs its own conversions, the scanner’s ADC interrupt could fire mid-measurement and interpret an ADCTouch conversion as a matrix reading, and either side could leave the ADC configured in a way the other does not expect. When the pads were first enabled without coordination, the firmware crashed or behaved erratically, which made the conflict unmistakable.

The firmware handles this by temporarily giving ADCTouch exclusive access to the ADC. Before reading the control pads, matrix scanning is paused and any active ADC conversion is allowed to finish. ADCTouch then samples both pads. Once this is complete, the ADC is restored to the matrix input and scanning resumes. The scan position and frame counter are also reset so that a partially captured frame from before the pause is not processed as valid sensor data.

After this change enabling the control pads made the matrix less responsive, particularly in the first TX column. The problem was caused by the state left on the A1 and A2 pins after ADCTouch finished sampling. Since these pins are next to the matrix input on A3, they could interfere with the analog measurement. Before matrix scanning resumes, the firmware therefore resets A1 and A2 as normal inputs with their pull-up resistors disabled. This restored more consistent sensing across the matrix.

Pad sampling is limited to set intervals to reduce ADC use. Between samples, the firmware uses the most recently stored pad states. At startup, both pads are calibrated in their untouched state, and a touch is detected when the reading differs from this reference by more than a defined threshold. Mode changes occur when a pad is released: the playback pad starts or stops playback, while the record pad toggles recording when playback is inactive. This keeps the matrix as the main user of the ADC, with the control pads accessing it only when needed.

### 4.2.10 Runtime Flow

At startup, the firmware clears the previous touch state and initializes the system. The matrix is calibrated after a short settling period, followed by the control pads. This order ensures that the required components are ready before calibration and normal operation begin. If the MCP4728 is not detected, setup stops with an error.

During operation, the library scans the matrix and signals when a new frame is complete. The application then processes the frame, identifies the dominant touch, handles recording or playback, and updates the DAC when the output values change. If no new frame is available, the loop returns without processing the previous data again. Serial diagnostics are also limited to changes in the touch state to avoid unnecessary output.

### 4.2.11 Software Challenges and Cut-Offs

The firmware implements the main sensing, touch-processing, and recording functions. The evaluation examines these functions under selected test conditions, while external-trigger synchronization remains unvalidated. Several limitations also follow directly from the chosen implementation. Most of the software challenges came from timing and sharing the limited resources of the ATmega328.

The main loop initially ran faster than sensor acquisition. This caused the same data to be processed repeatedly. This was solved by processing only completed frames. The ADC required similar coordination, since the matrix and ADCTouch controls share the same converter. A pause/resume process was therefore introduced to give each function temporary control of the ADC.

Since scanning and external triggers can interrupt the main program at any time, the interrupt routines are kept as short as possible. They only handle essential tasks, such as advancing the scan or updating a counter, while touch processing remains in the main loop. Serial diagnostics are also limited to changes in the touch state to prevent them from affecting the timing of the system.

Memory is another constraint. The 2 KB SRAM does not allow full matrix histories or complete multi-touch recordings. The firmware therefore uses a bitset for active cells and records only the dominant touch. This provides the required gesture recording while accepting that simultaneous touches cannot be reproduced during playback.

Within these constraints, the firmware combines frame-based acquisition, baseline tracking, hysteresis, dominant-touch selection, cached DAC updates, and trigger-synchronized playback. The performance of the resulting system is evaluated in the next chapter.
