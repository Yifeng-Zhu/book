<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN">
<html><head>
<style type="text/css">
<!--
A:link {color: blue; text-decoration: none; font-weight: bold}
A:active {color: blue; text-decoration: none; font-size: 100%}
A:visited {color: blue; text-decoration: none; font-weight: bold}
A:hover {color: red; text-decoration: none; font-size: 100%}
.white {
	color: #FFF;
}
.STYLE1 {
	color: #990000;
	font-weight: bold;
}
.STYLE2 {
	color: #FF0000;
	font-weight: bold;
}
-->
</style>
<meta http-equiv="Content-Type" content="text/html; charset=windows-1252"><title>Embedded Systems with ARM Cortex-M Microcontrollers in Assembly Language and C</title>

</head>




<body style="background-color: rgb(255, 255, 255);">
<?php include_once("analyticstracking.php") ?>
<table align="center" border="0" width="100%">
<tbody>
<tr>
<td rowspan="6" width="194">
<div align="center"><img src="zhu_book_front_cover_v3.png" alt="Yifeng Zhu" width="164" height="221"></div></td>
<td width="1442"><strong><font color="#990000" face="Courier New, Courier, mono" size="+3">Embedded Systems with ARM Cortex-M Microcontrollers <br>
       in Assembly Language and C (Third Edition)</font></strong></td>
</tr>
<tr>
<td><strong>ISBN-13: </strong>978-0-9826926-6-0, <strong>Publisher:</strong> E-Man Press LLC; 3rd edition (July 2017)</td>
</tr>
<tr>
</tr>
<tr>
<td><strong><font face="Georgia, Times New Roman, Times, serif">Available from <a href="https://www.amazon.com/dp/0982692668">Amazon</a>, <a href="https://www.walmart.com/ip/Embedded-Systems-with-Arm-Cortex-M-Microcontrollers-in-Assembly-Language-and-C-Third-Edition-9780982692660/624024697">Walmart</a></font></strong></td>
</tr>
<tr>
  <td align="center"><div align="left"></div>    
  <div align="left"></div></td>
  </tr>

<tr>
  <td>
    <div align="left"><a href="index.php">Home</a> <strong>/ </strong> <a href="intro.php">Introduction </a> <strong>/ </strong><a href="adoptions.php">Adoptions </a> <strong>/ </strong><a href="lecture.php">Instructor Site </a><strong> /</strong> <span class="STYLE2">Sample Labs</span>  <strong> / </strong> <a href="kit.php">Lab Kit </a> <strong> / </strong> <a href="tutorials.php"> Tutorials </a><strong> / </strong><a href="faq.php">FAQ</a></div>
    </div>
    <div align="left"><font face="Georgia, Times New Roman, Times, serif"><a href="calendar/index.php"> </a></font></div>  </td>
</tr>
</tbody>
</table>
<table width="100%" border="0" align="center">
  <tbody>
    <tr bgcolor="#ffffff">
      <td height="40"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="26"><a href="#Templates">Keil Project Templates</a> / <a href="#Descriptions">Lab Descriptions</a></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="37"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="39"><span class="STYLE1">Lab Grading Rubics</span> (<a href="lab/Lab_Grading_Rubrics.pdf">pdf</a>) </td>
    </tr>
	    <tr bgcolor="#ffffff">
      <td height="39"><span class="STYLE1">STM32L4 Pins</span> (<a href="STM32L4/STM32L476_Pins.pdf">pdf</a>, <a href="STM32L4/STM32L476_Pins.xlsx">xlsx</a>) </td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="21"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="21"><span class="STYLE1" name="Templates"><a name="Templates"></a>Keil Project Templates (v5)</span></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="132"><table width="1024" align="left" border="1">
              <tr>
          <td><p><a href="https://www.st.com/en/evaluation-tools/nucleo-l476rg.html">STM  NUCLEO-L476RG </a><br>
            <strong>Cortex-M4 with FPU and DSP </strong><br>
            Pin Functions (<a href="NUCLEO-L476RG/STM32L476RG_NUCLEO_Pins.xlsx">xlsx</a>, <a href="NUCLEO-L476RG/STM32L476RG_NUCLEO_Pins.pdf">pdf</a>)<br> Schematic (<a href="NUCLEO-L476RG/Nucleo-L476RG schematics.pdf">pdf</a>)<br>
              <em><strong>(Focus of the Textbook)</strong></em></p></td>
          <td><p>C Project Basic Template (<a href="NUCLEO-L476RG/NUCLEO-L476RG_C_Template.zip">zip</a>)<br>
                 C Project Basic Template with STLink UART (<a href="NUCLEO-L476RG/NUCLEO-L476RG_C_Template_STLink_UART.zip">zip</a>)</p>
            <ul>
              <li><a href="NUCLEO-L476RG/main_basic.c">main_basic.c</a></li>
              <li><a href="STM32L4/main.c">main_STLink_UART.c</a></li>
              <li><a href="STM32L4/startup_stm32l476xx.s">startup_stm32l476xx.s</a> </li>
              <li><a href="STM32L4/stm32l476xx.h">stm32l476xx.h</a></li>
            </ul></td>
          <td><p>Assembly Project Template (<a href="NUCLEO-L476RG/NUCLEO-L476RG_Assembly_Template.zip">zip</a>)<br>
          Assembly Project Template with STLink UART (<a href="NUCLEO-L476RG/NUCLEO-L476RG_Assembly_Template_STLink_UART.zip">zip</a>)
          </p>
            <ul>
              <li><a href="NUCLEO-L476RG/main_basic.s">main.s</a></li>
              <li><a href="NUCLEO-L476RG/main_STLink_UART.s">main_STLink_UART.s</a></li>
              <li><a href="Core/core_cm4_constants.s">core_cm4_constants.s</a></li>
              <li><a href="STM32L4/stm32l476xx_constants.s">stm32l476xx_constants.s</a></li>
              <li><a href="STM32L4/startup_stm32l476xx.s">startup_stm32l476xx.s</a><a href="STM32L/startup_stm32l1xx_md.s"></a></li>
            </ul></td>
        </tr>
        <tr>
          <td><p><a href="http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/LN1848/PF261635?sc=stm32l4-discovery">STM Discovery kit with STM32L476VG</a><br>
            <strong>Cortex-M4 with FPU and DSP </strong><br>
            Pin Functions (<a href="STM32L4/STM32L476 Pins.xlsx">xlsx</a>, <a href="STM32L4/STM32L476 Pins.pdf">pdf</a>)<br> Schematic (<a href="STM32L4/STM32L476-DISCO Schematics.pdf">pdf</a>) <br>
            <em><strong>(Focus of the Textbook)</strong></em></p></td>
          <td><p>C Project Template (<a href="STM32L4/STM32L4_Lab_Template_C.zip">zip</a>)            </p>
            <ul>
              <li><a href="STM32L4/main.c">main.c</a></li>
              <li><a href="STM32L4/startup_stm32l476xx.s">startup_stm32l476xx.s</a> </li>
              <li><a href="STM32L4/stm32l476xx.h">stm32l476xx.h</a></li>
            </ul></td>
          <td><p>Assembly Project Template (<a href="STM32L4/STM32L4_Lab_Template_Assembly.zip">zip</a>)            </p>
            <ul>
              <li><a href="STM32L4/main.s">main.s</a></li>
              <li><a href="Core/core_cm4_constants.s">core_cm4_constants.s</a></li>
              <li><a href="STM32L4/stm32l476xx_constants.s">stm32l476xx_constants.s</a></li>
              <li><a href="STM32L4/startup_stm32l476xx.s">startup_stm32l476xx.s</a><a href="STM32L/startup_stm32l1xx_md.s"></a></li>
            </ul></td>
        </tr>
        <tr>
          <td width="31%"><p><a href="http://www.st.com/web/en/catalog/tools/FM116/SC959/SS1532/LN1848/PF258515">STM Discovery kit with STM32L152RCT6</a><br>
            <strong>Cortex-M3</strong><br> 
            Schematic (<a href="STM32L/STM32L1_Schematics.pdf">pdf</a>) </p></td>
          <td width="33%"><p>C Project Template (<a href="STM32L/STM32L_Lab_Template_C.zip">zip</a>)            </p>
            <ul>
              <li><a href="STM32L/main.c">main.c</a></li>
              <li><a href="STM32L/startup_stm32l1xx_md.s">startup_stm32l1xx_md.s</a> </li>
              <li><a href="STM32L/stm32l1xx.h">stm32l1xx.h</a>              </li>
            </ul>
            <blockquote>
              <p>&nbsp;</p>
            </blockquote></td>
          <td width="36%"><p>Assembly Project Template (<a href="STM32L/STM32L_Lab_Template_Assembly.zip">zip</a>)            </p>
            <ul>
              <li><a href="STM32L/main.s">main.s</a></li>
              <li><a href="Core/core_cm3_constant.s">core_cm3_constant.s</a></li>
              <li><a href="STM32L/stm32l1xx_constants.s">stm32l1xx_constants.s</a></li>
              <li><a href="STM32L/stm32l1xx_tim_constants.s">stm32l1xx_tim_constants.s</a></li>
              <li><a href="STM32L/startup_stm32l1xx_md.s">startup_stm32l1xx_md.s</a></li>
            </ul></td>
        </tr>
        <tr>
          <td><p><a href="http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF252419">STM Discovery kit with STM32F407VG</a><br>
            <strong>Cortex-M4 with FPU and DSP </strong> <br> 
            Schematic (<a href="STM32F4/STM32F4DISCOVERY_schematic.pdf">pdf</a>) <br></p></td>
          <td><p>C Project Template (<a href="STM32F4/STM32F4_Lab_Template_C.zip">zip</a>)            </p>
            <ul>
              <li><a href="STM32F4/main.c">main.c</a></li>
              <li><a href="STM32F4/startup_stm32f407xx.s">startup_stm32f407xx.s</a> </li>
              <li><a href="STM32F4/stm32f407xx.h">stm32f407xx.h</a></li>
            </ul></td>
          <td><p>Assembly Project Template (<a href="STM32F4/STM32F4_Lab_Template_Assembly.zip">zip</a>)            </p>
            <ul>
              <li><a href="STM32F4/main.s">main.s</a></li>
              <li><a href="Core/core_cm4_constants.s">core_cm4_constants.s</a></li>
              <li><a href="STM32F4/stm32f401xc_constants.s">stm32f401xc_constants.s</a></li>
              <li><a href="STM32F4/startup_stm32f407xx.s">startup_stm32f407xx.s</a></li>
            </ul></td>
        </tr>
        <tr>
          <td><p><a href="http://www.ti.com/tool/ek-tm4c123gxl">TI Tiva C Series LaunchPad</a><br>
            <strong>Cortex-M4 with FPU and DSP</strong> <br> 
            Schematic (<a href="TivaC/EK-TM4C123GXL Rev A Schematic.pdf">pdf</a>) <br></p></td>
          <td><p>C Project Template (<a href="TivaC/TivaC_Lab_Template_C.zip">zip</a>)            </p>
            <ul>
              <li><a href="TivaC/main.c">main.c</a></li>
              <li><a href="TivaC/startup_TM4C123.s">startup_TM4C123.s</a></li>
              <li><a href="TivaC/tm4c123gh6pge.h">tm4c123gh6pge.h</a></li>
            </ul></td>
          <td><p>Assembly Project Template (<a href="TivaC/TivaC_Lab_Template_Assembly.zip">zip</a>)            </p>
            <ul>
              <li><a href="TivaC/main.s">main.s</a></li>
              <li><a href="Core/core_cm4_constants.s">core_cm4_constants.s</a></li>
              <li><a href="TivaC/tm4c123gh6pm_constants.s">tm4c123gh6pm_constants.s</a></li>
              <li><a href="TivaC/startup_TM4C123.s">startup_TM4C123.s</a></li>
            </ul></td>
        </tr>
        <tr>
          <td><p><a href="https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/freedom-development-platform-for-kinetis-k64-k63-and-k24-mcus:FRDM-K64F">FRDM-K64F: Freescale Freedom Development Platform</a><br>
            <strong>Cortex-M4 with FPU and DSP</strong><br>
            Schematic (<a href="FRDM-K64F/FRDM-K64F-SCH-E4.pdf">pdf</a>) <br>
            Pin functions  (<a href="FRDM-K64F/FRDM-K64F_pins.xlsx">xlsx</a>, <a href="FRDM-K64F/FRDM-K64F_pins.pdf">pdf</a>)</p></td>
          <td><p>C Project Template (<a href="FRDM-K64F/FRDM_K64F_Template_C.zip">zip</a>)            </p>
            <ul>
              <li><a href="FRDM-K64F/main.c">main.c</a></li>
              <li><a href="FRDM-K64F/startup_MK64F12.s">startup_MK64F12.s</a></li>
              <li><a href="FRDM-K64F/MK64F12.h">MK64F12.h</a></li>
            </ul></td>
          <td><p>Assembly Project Template (<a href="FRDM-K64F/FRDM_K64F_Template_Assembly.zip">zip</a>)            </p>
            <ul>
              <li><a href="FRDM-K64F/main.s">main.s</a></li>
              <li><a href="Core/core_cm4_constants.s">core_cm4_constants.s</a></li>
              <li><a href="FRDM-K64F/MK64F12_constants.s">MK64F12_constants.s</a></li>
              <li><a href="FRDM-K64F/startup_MK64F12.s">startup_MK64F12.s</a></li>
            </ul></td>
        </tr>
      </table></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33"><span class="STYLE1"><a name="Descriptions"></a>Example Lab Descriptions Based on STM32L4 Discovery Kit: </span></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33" align="center"><table width="657" height="278" border="1" align="left">
        <tr>
          <td width="29"><div align="center"></div></td>
          <td width="349">Lab Description </td>
          <td width="257"><div align="center"><strong>STM32L4</strong> Discovery Kit </div></td>
          </tr>
        <tr>
          <td><div align="center">1</div></td>
          <td>Interfacing Pushbutton and LED in C</td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_01_LED_Pushbutton_C.pdf"> Lab Description (pdf)</a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">2</div></td>
          <td>LCD Display Driver in C</td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_02_LCD_C.pdf">Lab Description (pdf) </a><a href="lab/L1_Register_Map_LCD.pdf"></a></li>
            <li><a href="lab/L4_Lab_02_LCD_C_Driver_Template.zip">LCD_Project_Template.zip</a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">3</div></td>
          <td>Interfacing Keypad in C <a href="lab/L1_Lab_03_Keypad_C.pdf"></a></td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_03_Keypad_C.pdf"> Lab Description (pdf)</a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">4</div></td>
          <td>Stepper Motor Control in C <a href="lab/L1_Lab_04_Stepper_Motor_C.pdf"></a></td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_04_Stepper_Motor_C.pdf">Lab Description (pdf)</a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">5</div></td>
          <td>System Timer (SysTick) in C <a href="lab/L1_Lab_05_SystemTimer_C.pdf"></a></td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_05_SystemTimer_C.pdf">Lab Description (pdf)</a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">6</div></td>
          <td>Pulse Width Modulation (PWM) in Assembly <a href="lab/L1_Lab_06_LED_PWM.pdf"></a></td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_06_LED_PWM.pdf">Lab Description (pdf) </a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">7</div></td>
          <td>Input Capture (Ultrasonic Distance Sensor) in C <a href="lab/L1_Lab_07_Timer_Capture_C.pdf"></a></td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_07_Timer_Capture_C.pdf">Lab Description (pdf)</a> </li>
            <li><a href="lab/HC-SR04 User Manual.pdf">HC-SR04 User Manual (pdf) </a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">8</div></td>
          <td>Input Capture (Ultrasonic Distance Sensor) in Asembly</td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_08_Timer_Capture_Assembly.pdf">Lab Description (pdf)</a></li>
            <li><a href="lab/HC-SR04 User Manual.pdf">HC-SR04 User Manual (pdf) </a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">9</div></td>
          <td>Analog to Digital Converter (ADC) <a href="lab/L1_Lab_09_ADC.pdf"></a></td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_09_ADC_C.pdf">Lab Description (pdf)</a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">10</div></td>
          <td>Digital to Analog Converter (DAC) <a href="lab/L1_Lab_10_DAC_1.pdf"></a></td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_10_DAC.pdf">Lab Description (pdf)</a></li>
          </ul></td>
          </tr>
        <tr>
          <td><div align="center">11</div></td>
          <td>Music Synthesizing (DAC) <a href="lab/L1_Lab_11_Musical.pdf"></a></td>
          <td valign="middle"><ul>
              <li><a href="lab/L4_Lab_11_Musical.pdf">Lab Description (pdf) </a></li>
          </ul></td>
          </tr>
      </table></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33"><hr></td>
    </tr>
  </tbody>
</table>
<p>&nbsp;</p>
</body></html>