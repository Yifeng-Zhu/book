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
.STYLE3 {color: #FF00FF; font-weight: bold; }
.STYLE4 {
	color: #FF0000;
	font-weight: bold;
}
.STYLE6 {color: #FF00FF; font-weight: bold; font-size: medium; }
.STYLE7 {font-size: medium}
.STYLE8 {
	color: #990000;
	font-style: italic;
}
.STYLE9 {
	color: #0000FF;
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
    <div align="left"><a href="index.php">Home</a> <strong>/ </strong> <a href="intro.php">Introduction </a>  <strong>/ </strong><a href="adoptions.php">Adoptions </a> <strong>/ </strong><a href="lecture.php">Instructor Site </a><strong> / </strong><a href="lab.php">Sample Labs </a> <strong> / </strong> <a href="kit.php">Lab Kit </a> <strong> / </strong><a href="tutorials.php"> Tutorials </a> /<span class="STYLE4"> FAQ </span></div>
    <div align="left"><font face="Georgia, Times New Roman, Times, serif"><a href="calendar/index.php"> </a></font></div>  </td>
</tr>
</tbody>
</table>
<table width="100%" border="0" align="center">
  <tbody>
    <tr bgcolor="#ffffff">
      <td height="21"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="132"><p><strong>Frequently asked questions</strong>:</p>
        <ol>
          <li><a href="#difference">Difference between STM32L1 and STM32L4<br>
          </a></li>
          <li><a href="#L1L4">Should I choose STM32L1 or STM32L4 Discovery Kit?</a></li>
          <li><a href="#toolchain">Which tool-chain/development environment should we use? </a></li>
          <li><a href="#RegisterLevel">Why do we program at the register level, instead of using ST/ARM API library? </a></li>
          <li><a href="#initialization">Why is the data memory uninitialized? <br>
          </a></li>
          <li><a href="#Firmware"> Firmware Upgrade</a></li>
          <li><a href="#TargetNoFound">&quot;Target not found&quot; error</a></li>
          <li><a href="#ProgramFailed">&quot;No ST-Link detected&quot; error </a></li>
          <li><a href="#FlashFailed">&quot;Flash Download Failed&quot; error </a></li>
          <li><a href="#ConnectionFailed">&quot;ST-Link connection error&quot;</a></li>
          <li><a href="#NoTarget">&quot;No target connected&quot;</a></li>
          <li><a href="#L6314W">Warning: L6314W: No section matches pattern *(InRoot$$Sections)</a></li>
          <li><a href="#BootRam">How to boot from RAM, instead of from Flash? </a></li>
          <li><a href="#SymbolError">Error:  L6218E: Undefined symbol Image$$ER_IROM1$$RO$$Base (referred from  startup_stm32l476xx.o)</a></li>
          <li><a href="#PRESERVE8">Error: L6238E: &nbsp;Invalid call from '~PRES8' function to 'REQ8'  function<br>
          </a></li>
        </ol></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="29"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="29"><p><span class="STYLE3">When you fail to download the code to the board, follow this tutorial to fix most common problems. </span><br>
      <iframe width="640" height="360" vspace="20" style="margin:15px"  src="https://www.youtube.com/embed/OiwwB0AvIBI" frameborder="0" gesture="media" allow="encrypted-media" allowfullscreen></iframe></p></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="29"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="132"><ul>
          <li><a name="difference"></a><span class="STYLE3">Difference between STM32L1 and STM32L4</span>
              <ol>
                <li>Simplified summary: </li>
                  <ul>
                    <li>STM32L1 has ARM Cortex-M3</li>
                    <li>STM32L4 has ARM Cortex-M4F </li>
                    <li><strong>Cortex-M4F = Cortex-M3 + DSP + FPU </strong></li>
                  </ul>
                  <li>Application note: <a href="http://www.st.com/st-web-ui/static/active/jp/resource/technical/document/application_note/DM00141025.pdf">Migrating from STM32L1 series to STM32L4 series microcontrollers </a>(<a href="STM32L4/Migrating from STM32L1 series to STM32L4 series microcontrollers.pdf">local copy</a>) </li>
                  <li><a href="STM32L4/STM32L1 vs STM32L4.pdf">An incomplete comparison between STM32L1 and STM32L4 </a>                  </li>
                <ul>
                  <li><a href="STM32L4/STM32L1 vs STM32L4 RCC.pdf">RCC</a>, <a href="STM32L4/STM32L1 vs STM32L4 GPIO.pdf">GPIO</a>, <a href="STM32L4/STM32L1 vs STM32L4 UART.pdf">UART</a>, <a href="STM32L4/STM32L1 vs STM32L4 SPI.pdf">SPI</a>, <a href="STM32L4/STM32L1 vs STM32L4 I2C.pdf">I2C</a>, <a href="STM32L4/STM32L1 vs STM32L4 ADC.pdf">ADC</a>, <a href="STM32L4/STM32L1 vs STM32L4 DAC.pdf">DAC</a>, <a href="STM32L4/STM32L1 vs STM32L4 RTC.pdf">RTC</a>, <a href="STM32L4/STM32L1 vs STM32L4 EXTI.pdf">EXTI</a></li>
                </ul>
            </ol>
          </li>
		  <hr>
              <li><a name="L1L4"></a><span class="STYLE3">Should I choose STM32L1 or STM32L4 Discovery Kit?</span>
                <ol>
                  <li>STM32L4 Kit is slightly more expensive. (STM32L4 costs ~$20, STM32L1 costs ~$12). </li>
                  <li>If you have to use FPU and DSP, choose STM32L4.</li>
                  <li>If you need more I/O pins, choose STM32L4. </li>
                  <li>If you have to use touch sensensing, choose STM32L1. </li>
                  <li>If you have to use codec and microphone, choose STM32L4 becaue it has on-board mic, CODEC, and earphone jack. </li>
                  <li>The book focuses on STM32L1 and STM32L4. All lab descriptions on this website have both versions. The STM32L4 lab descriptions highlight special attentions not presented in the book.</li>
                  <li class="STYLE4">If you do not know which one to use, I recommend STM32L4. </li>
                </ol>
              </li>
              <hr>
              <li><a name="toolchain"></a> <span class="STYLE6"> Which tool-chain/development environment should we use?</span>  
                <ul>
                  <li>There are both free and commercial tool-chains for STM32 Cortex-M family.
                    <ul>
                      <li>Two most popular commercial tool-chains are <a href="https://www.iar.com/iar-embedded-workbench/tools-for-arm/arm-cortex-m-edition/">IAR for Cortex-M</a> and <a href="https://www.keil.com/demo/eval/arm.htm">Keil</a>. Both are only available on Windows platforms. Both offers an evaluation version, which limits the code size to 32 KB.</li>
                      <li>Free tool-chains include <a href="http://www.coocox.org/">CooCox</a> and <a href="http://www.openstm32.org/">System Workbench for STM32 (OpenSTM32) </a>. They are based on GCC and Eclipse. CooCox supports only Windows. OpenSTM32 supports Windows, Linux, and MacOS. They have no limitations on the code size.</li>
                      <li>I strongly recommend Keil. Here are the reasons.
                        <ul>
                          <li>The free evaluation version works great. The code-size limiation is not a problem for us at all. The code size of our lab and homework never exceeds 32 KB. </li>
                          <li>For Linux and Mac machnes, Keil can work perfectly on a Windows virtual machine. </li>
                          <li>Keil offers many features that free tool-chains do not have. Most importantly, <strong>Keil allows us to view the values of Cortex-M registers and all periperiperal components registers in real-time</strong>. </li>
                          <li><img src="Keil_Core_Registers.png" width="242" height="855" hspace="40"> <img src="Keil_Peripherals.png" width="459" height="850" hspace="40"> <img src="Keil_Peripherals_Registers.png" width="326" height="859" hspace="40"></li>
                        </ul>
                      </li>
                    </ul>
                  </li>
                </ul>
          </li>
			  <hr>
			  <li><span class="STYLE7"><a name="RegisterLevel"></a> <span class="STYLE3"> Why do we program  at the register level, instead of using ST/ARM API library?</span></span>			  
			    <ul>
			      <li>I strongly discrouage students to use any pre-made libraries provided by ST or ARM, such as <a href="https://developer.arm.com/embedded/cmsis">ARM CMSIS</a>, and  <a href="http://www.st.com/en/embedded-software/stm32cube-embedded-software.html?querycriteria=productId=LN1897">STM32Cube</a>  HAL (Hardware Abstraction Layer) and  LL (Low Layer) APIs.</li>
		          <li>Directly controlling, monitoring, and accessing on-chip registers is the best way to learn firmware development. </li>
			      <li>The libraries provided by ST or ARM hide too much details about what's under the hood. Students using provided libraries can get a lab to work,  but they often do not know what  the code did. </li>
			    </ul>
		  </li>
			  <hr>
              <li><a name="initialization"></a><span class="STYLE3">Why is the data memory  uninitialized?</span>
                <ul>
                  <li>If your main function is written in assembly and you use the default startup_stm32l476xx.s, you will find that your data memory is not initialized when the program runs. </li>
                  <li>Solution: use my modified <a href="http://web.eece.maine.edu/~zhu/book/STM32L4/startup_stm32l476xx.s">startup_stm32l476xx.s</a>. The modified code starts the RW data segments and the ZI segments from flash to RAM. </li>
                </ul>
          </li>
		  <hr>
          <li><span class="STYLE7"><a name="Firmware"></a><span class="STYLE3">Firmware Upgrade </span></span>
            <ul>
              <li>When you download the binary code to the STM32 Discover Kit, sometimes it pops up a window and asks you "old ST-Link firmware detected. Do you want to upgrade it?" Select &ldquo;Yes&rdquo; and click &ldquo;Device Connect.&rdquo;</li>
              <li>If it gives you the error message "ST-Link is not in the DFU mode,"&nbsp;<em><strong>unplug the USB cable from the kit, and then reconnect it</strong></em>, and try &ldquo;Device Connect&rdquo; again.</li>
              <li>If the connection is successful, click &ldquo;YES&rdquo; to upgrade the firmware.<br>
              <img src="STLink_Upgrade.png" width="848" height="194"></li>
            </ul>
          </li>		
		  <hr>
          <li><span class="STYLE7"><a name="TargetNoFound"></a><span class="STYLE3">Solving "Target not found" error</span></span>
            <ol>
              <li>When you use the STM32L4 board at the very first time (STM32L1 has no such issue), you might not be able to program it in Keil and receive an error of &quot;<em><strong>Target not found</strong></em>&quot; when you download the code to the board. This is because the demo program quickly puts the microprocessor into a very low power mode after a reset. There are several ways to solve it. Below are two simplest ways. The error will go away permanently.
                <ul>
                  <li><strong>Method 1</strong>: In Keil, click the icon &quot;Options for Target&quot;, follow &quot;Debug&quot; and then &quot;Settings&quot;, and change the connect from &quot;normal&quot; to &quot;with pre-reset&quot; in the dialog popped.</li>
                  <li><strong>Method 2</strong>:
                      Replace the demo program with one that does not put processor to low power mode
                    <ul>
                      <li>Download <a href="STM32L4/DISCO_L476VG_leds_buttons_DISCO_L476VG.bin">DISCO_L476VG_leds_buttons_DISCO_L476VG.bin</a></li>
                      <li>Copy it to the STM32L USB drive. When you plug the STM32L4 board to the computer, it is mounted as a USB drive named &quot;<em><strong>DIS_L476VG</strong></em>&quot;.<br>
                      </li>
                      <li>Push the &quot;reset&quot; button and reboot the board.</li>
                    </ul>
                  </li>
                  <li><img src="Keil_pre_reset.png" width="627" height="440"></li>
                </ul>
              </li>
              <li>If this problem comes back, follow the following different methods to see whether this can solve your problem
                <ul>
                  <li>Make sure all jumpers are correctly connected. </li>
                  <li>Use the STM32 ST-Link Utility to erase chip. Hold down the reset button before the USB cable is plugged in. When the reset button is released, immediately select &quot;<em><strong>Target</strong></em> -> <em><strong>Erase Chip</strong></em>&quot; in ST-Link Utility. Several attempts might be needed to get the timing correctly. </li>
                  <li>Re-install USB drive </li>
                </ul>
              </li>
            </ol>
          </li>
		  <hr>
          <li><span class="STYLE7"><a name="ProgramFailed"></a><span class="STYLE3">&quot;No ST-Link detected&quot; error</span></span>
            <ul>
              <li>First, make sure that the device driver is corrected installed. If not sure, re-install the USB device driver: go to the directory <span class="STYLE8">C:\Keil_v5\ARM\STLink\USBDriver</span> and run <span class="STYLE8">stlink_winusb_install.bat</span></li>
              <li>On STM32L4, the alternative function of pin PA.13 and PA.14 should be ST-Link SWDIO and SWCLK, respectively. If you code accidently changes the mode or the alternative function of these two pins, you can no longer program the STM3L4 kit anymore.
                <ul>
                  <li>Solution: Install <a href="http://www.st.com/en/development-tools/stsw-link004.html">ST-Link Utitity</a> and follow the following steps  to erase your bad code stored on the board.</li>
                  <li><strong>Step 1</strong>: Open ST-Link Utility, click menu &quot;<em><strong>Target</strong></em>&quot; , click &quot;<em><strong>Settings</strong></em>&quot;<br>
                    <img src="STLink_Utility_Setting.png" width="842" height="615"></li>
                  <li>Step 2: Select &quot;<em><strong>Connect Under Set</strong></em>&quot; as the connection mode<br>
                    <img src="STLink_Utility_Setting_Connect_Under_Reset.png" width="842" height="615"></li>
                  <li>Step 3: Click &quot;<em><strong>Target</strong></em>&quot; and &quot;<em><strong>Connect</strong></em>&quot;, and then click &quot;<em><strong>Target</strong></em>&quot; and &quot;<em><strong>Erase Chip</strong></em>&quot;! <br>
                    <img src="STLink_Utility.png" width="842" height="615"></li>
                  <li>Step 4: Click &quot;<em><strong>Target</strong></em>&quot; and &quot;<em><strong>Disconnect</strong></em>&quot; </li>
                </ul>
              </li>
            </ul>
          </li>
		  <hr>
          <li> <span class="STYLE7"><a name="FlashFailed"></a><span class="STYLE3"> Error: Flash Download failed </span></span>		  
            <ul>
              <li>When you program the board, the following error might show up<br>
              <img src="Keil_Flash_Download_Failed.png" width="350" height="173"></li>
              <li>Solution: </li>
              <li>Step 1: Click the icon &quot;<em><strong>Options for Targets</strong></em>&quot;, go to the Debug page, click &quot;<em><strong>Settings</strong></em>&quot; <br>
              <img src="Keil_Settings_Debug.png" width="643" height="478"></li>
              <li>Step 2: Go to the &quot;<em><strong>Flash Download</strong></em>&quot; page and make sure &quot;<em><strong>STM32L4xx 1MB Flash</strong></em>&quot; is present. If not, add it.<br>
              <img src="Keil_Settings_Debug_Flash.png" width="678" height="469"> </li>
            </ul>
          </li>
		  <hr>
          <li> <span class="STYLE7"><a name="ConnectionFailed"></a><span class="STYLE3"> ST-Link connection error </span></span>		  
            <ul>
              <li>When you program the board, Keil might report &quot;ST-Link connection error&quot;<br>
              <img src="Keil_STLink_Connection_Error.png" width="256" height="176"></li>
              <li>Solution: Make sure that your board is not connected by any other software. For example, your ST-Link Utility is not connecting to your board. No other Keil is debugging the board.</li>
            </ul>
          </li>
		  <hr>
             <li><span class="STYLE7" name="NoTarget"><a name="NoTarget"></a><span class="STYLE3"> "No target connected"</span></span>
               <ul>
                 <li>When you program the board, Keil might report &quot;No target connected&quot;. <br>
                 <img src="Keil_no_target_connected.png" width="231" height="174"></li>
                 <li>Solution: (1) Make sure the USB cable is well connected. (2) Make sure that all jumpers on the board are on the correct positions. See the user manual &quot;<a href="STM32L4/Discovery kit with STM32L476VG MCU.pdf">Discovery kit with STM32L476VG MCU</a>&quot; for the settings of jumpers. </li>
               </ul>
             </li>
          <hr>
          <li> <span class="STYLE7"><a name="L6314W"></a><span class="STYLE3">Warning: L6314W: No section matches pattern *(InRoot$$Sections) </span></span>
            <ul>
              <li>When you compile your assembly project, you will receive  an warning message of L6314W. This warning is genearted because my startup_xxx.s file directly calls the __main subroutine. You can ignore this warning message, or suppress it by following these steps: (1) click &quot;options for target&quot; icon, (2) go the Linker page, (3) add &quot;<strong>--diag_suppress=L6314W</strong>&quot; to the Misc controls box. <br>
              <img src="Keil_linker_L6314W.png" width="646" height="480"></li>
            </ul>
          </li>
		  <hr> 
		  <li> <span class="STYLE7"><a name="BootRam"></a><span class="STYLE3">Boot from RAM, instead of Flash</span></span>
		    <ul>
		      <li>Here is the simplest way: use <a href="STM32L4/STM32 STLink Utility - how to.pdf">STLink Utility</a></li>
	        </ul>
		  </li>
		  		  <hr> 
		  <li> <span class="STYLE7"><a name="SymbolError"></a><span class="STYLE3">Error:  L6218E: Undefined symbol Image$$ER_IROM1$$RO$$Base (referred from  startup_stm32l476xx.o)</span></span><ul>
		      <p>When you build a new project from the beginning without using my templates, you might have the following error if you use my modified <a href="http://web.eece.maine.edu/~zhu/book/STM32L4/startup_stm32l476xx.s">startup_stm32l476xx.s</a>:	          </p>
		      <blockquote>
		        <p>.\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$ER_IROM1$$RO$$Base (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$ER_IROM1$$RO$$Length (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$ER_IROM1$$RO$$Limit (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$ER_IROM1$$RW$$Base (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$ER_IROM1$$RW$$Length (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$ER_IROM1$$RW$$Limit (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$ER_IROM1$$ZI$$Base (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol Image$$ER_IROM1$$ZI$$Length  (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$ER_IROM1$$ZI$$Limit (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$Base (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$Length (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$Limit (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$RO$$Base (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$RO$$Length (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$RW$$Base (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$RW$$Length (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$RW$$Limit (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$ZI$$Base (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol Image$$RW_IRAM1$$ZI$$Length  (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Image$$RW_IRAM1$$ZI$$Limit (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Load$$ER_IROM1$$Base (referred from startup_stm32l476xx.o).<br>
		          .\Objects\test.axf: Error: L6218E: Undefined symbol  Load$$RW_IRAM1$$Base (referred from startup_stm32l476xx.o).</p>
	            </blockquote>
		      </ul>
		    <blockquote>
		      <p><strong>Solution</strong>: Click &quot;Options for Target&quot; icon, and go the Linker page, and select &quot;Use Memory Layout from Target Dialog&quot;, as shown below</p>
		      <p><img src="Symbol_Error_Solution.png" width="640" height="472"> </p>
		      <p>&nbsp;</p>
		    </blockquote>
		  </li>
		 <hr> 
		  <li> <span class="STYLE7"><a name="PRESERVE8"></a><span class="STYLE3">Error: L6238E: Invalid call from '~PRES8' function to 'REQ8' function</span></span>
		    <ul>
		      <li>By default, a function implemented in assembly does not require code to preserve 8-byte alignment for stack allocation. </li>
	          <li>However, a C function does require that. </li>
		      <li>Therefore, when an assembly function calls a C function, the error L6238E occurs if the assembly function uses push or pop function. </li>
            </ul>
		    <blockquote>
		      <p><strong>Solution</strong>: At the beginning of the assembly file, you can add the <span class="STYLE9">PRESERVE8</span> directive to ensure that the stack is aligned with eight-byte boundary. For example:</p>
		      <pre>    PRESERVE8      </pre>
		      <pre>    ...      </pre>  
		      <pre>    AREA myAssembly, CODE, READONLY</pre>
		      <p>&nbsp; </p>
		    </blockquote>
		  </li>		  
          </ul>	  </td>
    </tr>
  </tbody>
</table>
<p>&nbsp;</p>
</body></html>