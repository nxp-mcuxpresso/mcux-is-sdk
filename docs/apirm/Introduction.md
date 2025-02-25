﻿Introduction   {#index}
=========================

<P>ISSDK 1.8 is designed as a middleware component inside the MCUXpresso SDK 2.13.0 ecosystem. 
This means that ISSDK 1.8 takes advantage of the deployment infrastructure, device specific drivers,
and project creation tools to supply customers with a consistent “out-of-box” experience. In addition
to the base enablement, the ISSDK is augmented with demo sensor and algorithm example projects for
supported board-kits, and API documentation to help users quickly leverage the support of the ISSDK 1.8.

<P>ISSDK 1.8 is fully integrated into the MCUXpresso Web & SDK Builder (Previously known as
Kinetis Expert (KEx)) delivery system. MCUXpresso Web & SDK Builder is a cloud-based system used to
build MCUXpresso SDK 2.13.0 packages. MCUXpresso Web & SDK Builder includes both cloud and locally based
tools to collect and build projects from the MCUXpresso SDK repositories.
MCUXpresso SDK 2.13.0 is built using a hierarchy of deployed Git repositories. Specific project codebases are built
through the online tool.  A given codebase is specified by its target (i.e., the device, board, or kit desired),
the version of MCUXpresso SDK 2.x, the supported IDEs (i.e., MCUXpresso IDE, IAR, Keil MDK, ARM GCC) and the
target Host OS(i.e., Windows, Mac, or Linux).

<P> For each Sensor Driver and communications channel there is a set of Driver Examples which demonstrate the
usage of the Sensor Driver over a variety of conditions that make sense for the sensor.
List of supported Sensors:

<table>
<caption id="multi_row">Supported Sensors</caption>
<tr> <th>Sensor</th><th>Sensor Type</th><th>Interface</th></tr>
<tr><td>FXLS8964AF</td><td>3-Axis Digital Accelerometer</td><td>I2C and SPI</td></tr>
<tr><td>FXLS8967AF</td><td>3-Axis Digital Accelerometer</td><td>I2C and SPI</td></tr>
<tr><td>FXLS8974CF</td><td>3-Axis Digital Accelerometer</td><td>I2C and SPI</td></tr>
<tr><td>FXAS21002C</td><td>3-Axis Digital Gyroscope</td><td>I2C and SPI</td></tr>
<tr><td>FXLC95000CL</td><td>3-Axis Digital Accelerometer</td><td>I2C and SPI</td></tr>
<tr><td>FXLS8471QR1</td><td>3-Axis Digital Accelerometer</td><td>SPI</td></tr>
<tr><td>FXLS896xAFR1</td><td>3-Axis Digital Accelerometer</td><td>SPI</td></tr>
<tr><td>FXOS8700CQ</td><td>6-Axis Accelerometer/Magnetometer Combo</td><td>I2C and SPI</td></tr>
<tr><td>MAG3110</td><td>3-Axis Digital Magnetometer</td><td>I2C</td></tr>
<tr><td>MMA845xQR1</td><td>3-Axis Digital Accelerometer</td><td>I2C</td></tr>
<tr><td>MMA8491Q</td><td>3-Axis Digital Accelerometer</td><td>I2C</td></tr>
<tr><td>MMA8652FCR1</td><td>3-Axis Digital Accelerometer</td><td>I2C</td></tr>
<tr><td>MMA9553</td><td>3-Axis Digital Intelligent Accelerometer</td><td>I2C</td></tr>
<tr><td>MPL3115A2</td><td>Digital Pressure/Altitude Sensor</td><td>I2C</td></tr>
<tr><td>FXPQ3115BV</td><td>Digital Pressure/Bio-Compatible Sensor</td><td>I2C</td></tr>
<tr><td>MPXV5004DP</td><td>Differential Analog Pressure Sensor</td><td>ADC</td></tr>
<tr><td>FXPS7250A4</td><td>Analog Absolute Pressure Sensor</td><td>ADC</td></tr>
<tr><td>FXPS7250D4</td><td>Digital Absolute Pressure Sensor</td><td>I2C</td></tr>
</table>

List of supported development boards:
Following Sensor Kits are supported by ISSDK 1.8:
- Standard Sensor Kits: These are Official MCU Board- Sensor Shield kits which
  are available for end user to order from NXP Sensor Evaluation Boards webpage.
- Custom Sensor Kits: These are Board Shield pairs which will not be available
  for end user to order as Official MCU Board- Sensor Shield kits
  (MCU Board and Sensor Shield must be ordered separately).

<table>
<caption id="multi_row">Standard Sensor Kits</caption>
<tr><th>Sensor Kit</th><th>MCU Board</th><th>Sensor Shield Board</th></tr>
<tr><td>FRDM-K22F-A8974</td><td>FRDM-K22F</td><td>FRDM-STBI-A8974</td></tr>
<tr><td>FRDM-K22F-AGM01</td><td>FRDM-K22F</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>FRDM-K22F-AGMP03</td><td>FRDM-K22F</td><td>FRDM-STBC-AGMP03</td></tr>
<tr><td>FRDM-K22F-SA9500</td><td>FRDM-K22F</td><td>FRDM-STBC-SA9500</td></tr>
<tr><td>FRDM-K64F-AGM01</td><td>FRDM-K64F</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>FRDM-K64F-AGM04</td><td>FRDM-K64F</td><td>FRDM-STBC-AGM04</td></tr>
<tr><td>FRDMKL25-A8471</td><td>FRDM-KL25Z</td><td>FRDMSTBC-A8471</td></tr>
<tr><td>FRDMKL25-A8491</td><td>FRDM-KL25Z</td><td>FRDMSTBC-A8491</td></tr>
<tr><td>FRDMKL25-P3115</td><td>FRDM-KL25Z</td><td>FRDMSTBC-P3115</td></tr>
<tr><td>FRDMKL27-B3115</td><td>FRDM-KL27Z</td><td>FRDMSTBI-B3115</td></tr>
<tr><td>FRDM-KL25Z</td><td>FRDM-KL25Z</td><td>Using on-board MMA8451</td></tr>
<tr><td>FRDM-KL27Z</td><td>FRDM-KL27Z</td><td>Using on-board MMA8451 & MAG3110</td></tr>
<tr><td>FRDMKL27-B3115</td><td>FRDM-KL27Z</td><td>FRDMSTBI-B3115</td></tr>
<tr><td>FRDMKE15-DP5004</td><td>FRDM-KE15Z</td><td>FRDMSTBCDP5004</td></tr>
</table>

<table>
<caption id="multi_row">Custom Sensor Kits</caption>
<tr><th>Sensor Kit</th><th>MCU Board</th><th>Sensor Shield Board</th></tr>
<tr><td>FRDM-K22F with A8964</td><td>FRDM-K22F</td><td>FRDM-STBA-A8964</td></tr>
<tr><td>FRDM-K22F with A8967</td><td>FRDM-K22F</td><td>FRDM-STBA-A8967</td></tr>
<tr><td>FRDM-K64F with MULT2B</td><td>FRDM-K64F</td><td>FRDM-FXS-MULT2-B</td></tr>
<tr><td>FRDM-KE15Z with PA7250</td><td>FRDM-KE15Z</td><td>FRDM-STBA-PA7250</td></tr>
<tr><td>FRDM-KE15Z with PD7250</td><td>FRDM-KE15Z</td><td>FRDM-STBA-PD7250</td></tr>
<tr><td>FRDM-KL27Z with A8471</td><td>FRDM-KL27Z</td><td>FRDMSTBC-A8471</td></tr>
<tr><td>FRDM-KL27Z with A8491</td><td>FRDM-KL27Z</td><td>FRDMSTBC-A8491</td></tr>
<tr><td>FRDM-KL27Z with P3115</td><td>FRDM-KL27Z</td><td>FRDMSTBC-P3115</td></tr>
<tr><td>LPCXpresso54114 with AGM01</td><td>LPXCpresso54114</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>LPCXpresso55S69 with AGM01</td><td>LPCXpresso55S69</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>LPCXpresso55S16 with AGM01</td><td>LPCXpresso55S16</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>LPCXpresso55S16 with A8964</td><td>LPCXpresso55S16</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>LPCXpresso55S16 with A8967</td><td>LPCXpresso55S16</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>LPCXpresso55S16 with A8974</td><td>LPCXpresso55S16</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>LPCXpresso55S06 with AGM01</td><td>LPCXpresso55S06</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>FRDM-K32L3A6 with AGM01</td><td>FRDM-K32L3A6</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>FRDM-K32W042 with AGM01</td><td>FRDM-K32W042</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>EVK-MIMXRT595 with AGM01</td><td>EVK-MIMXRT595</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>MIMXRT685-EVK with AGM01</td><td>MIMXRT685-EVK</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>EVK-MIMXRT1010 with AGM01</td><td>EVK-MIMXRT1010</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>EVK-MIMXRT1015 with AGM01</td><td>EVK-MIMXRT1015</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>EVK-MIMXRT1020 with AGM01</td><td>EVK-MIMXRT1020</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>MIMXRT1024-EVK with AGM01</td><td>MIMXRT1024</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>EVKB-IMXRT1050 with AGM01</td><td>EVKB-IMXRT1050</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>EVK-MIMXRT1060 with AGM01</td><td>EVK-MIMXRT1060</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>EVK-MIMXRT1064 with AGM01</td><td>EVK-MIMXRT1064</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>EVK-MIMXRT1170 with AGM01</td><td>EVK-MIMXRT1170</td><td>FRDM-STBC-AGM01</td></tr>
<tr><td>MEK-MIMX8QM</td><td>MEK-MIMX8QM</td><td>Using on-board FXOS8700, FXAS21002, MPL3115 & ISL29023</td></tr>
</table>

<P> All sensor and algorithm example projects are provided with projects for the following toolchains:
<UL>
<LI>IAR Embedded Workbench
<LI>MCUXpresso IDE
<LI>GNU toolchain for ARM<sup>®</sup> Cortex<sup>®</sup>-M with Cmake build system
<LI>Keil MDK
</UL>
