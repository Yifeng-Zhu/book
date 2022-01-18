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
    <div align="left"><a href="index.php">Home</a> <strong>/ </strong> <a href="intro.php">Introduction </a>  <strong>/ </strong><a href="adoptions.php">Adoptions </a> <strong>/ </strong><a href="lecture.php">Instructor Site </a><strong> / </strong><a href="lab.php">Sample Labs </a><strong> / </strong> <a href="kit.php">Lab Kit</a> <strong> / </strong> <a href="tutorials.php"> Tutorials</a><strong> / </strong><a href="faq.php">FAQ</a></div>
    </div>
    <div align="left"><font face="Georgia, Times New Roman, Times, serif"><a href="calendar/index.php"> </a></font></div>  </td>
</tr>
</table>
<table width="100%" border="0" align="center">
  <tbody>
    <tr bgcolor="#ffffff">
      <td height="27"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="132"><p><strong>Table of Contents</strong>  (<a href="table_of_contents_V2.pdf">pdf</a>) <br>
        Table of Contents<br>
        <strong>Chapter 1.	See a Program Running	1</strong><br>
        1.1	Translate a C Program into a Machine Program	1<br>
        1.2	Load a Machine Program into Memory	4<br>
        1.2.1	Harvard Architecture and Von Neumann Architecture	4<br>
        1.2.2	Creating Runtime Memory Image	7<br>
        1.3	Registers	11<br>
        1.3.1	Reusing Registers to Improve Performance	12<br>
        1.3.2	Processor Registers	13<br>
        1.4	Executing a Machine Program	17<br>
        1.4.1	Loading a Program	18<br>
        1.4.2	Starting the Execution	19<br>
        1.4.3	Program Completion	23<br>
        1.5	Exercises	24<br>
        <strong>Chapter 2.	Data Representation	27</strong><br>
        2.1	Bit, Byte, Halfword, Word, and Double-word	27<br>
        2.2	Binary, Octal, Decimal, and Hexadecimal Numbers	29<br>
        2.3	Unsigned Integers	30<br>
        2.4	Signed Integers	31<br>
        2.4.1	Sign-and-Magnitude	33<br>
        2.4.2	One&rsquo;s Complement	34<br>
        2.4.3	Two&rsquo;s Complement	35<br>
        2.4.4	Carry Flag for Unsigned Addition or Subtraction	36<br>
        2.4.5	Overflow Flag for Signed Addition or Subtraction	39<br>
        2.4.5.1	Interpreting the Carry and Overflow Flags	42<br>
        2.4.5.2	Two&rsquo;s Complement Simplifies Hardware Implementation	45<br>
        2.5	Character String	49<br>
        2.6	Exercises	53<br>
        <strong>Chapter 3.	ARM Instruction Set Architecture	55</strong><br>
        3.1	ARM Assembly Instruction Sets	55<br>
        3.2	ARM Cortex-M Organization	58<br>
        3.3	Going from C to Assembly	60<br>
        3.4	Assembly Instruction Format	63<br>
        3.5	Anatomy of an Assembly Program	65<br>
        3.6	Assembly Directives	69<br>
        3.7	Exercises	73<br>
        <strong>Chapter 4.	Arithmetic and Logic	75</strong><br>
        4.1	Program Status Register	75<br>
        4.2	Updating Program Status Flags	77<br>
        4.3	Shift and Rotate	78<br>
        4.4	Arithmetic Instructions	80<br>
        4.5	Barrel Shifter	83<br>
        4.6	Bitwise Logic Operations	84<br>
        4.7	Order of Bits and Bytes	89<br>
        4.8	Sign and Zero Extension	90<br>
        4.9	Data Comparison	91<br>
        4.10	Data Movement between Registers	92<br>
        4.11	Bit Field Extract	93<br>
        4.12	Exercises	94<br>
        <strong>Chapter 5.	Load and Store	97</strong><br>
        5.1	Load Constant into Registers	97<br>
        5.1.1	Data Movement Instruction MOV and MVN	97<br>
        5.1.2	Pseudo Instruction LDR and ADR	98<br>
        5.1.3	Comparison of LDR, ADR, and MOV	99<br>
        5.2	Big Endian and Little Endian	100<br>
        5.3	Accessing Data in Memory	101<br>
        5.4	Memory Addressing	101<br>
        5.4.1	Pre-index, Post-index, and Pre-index with Update	101<br>
        5.4.2	Load and Store Instructions	103<br>
        5.4.3	PC-relative Addressing	104<br>
        5.4.4	Example of Accessing an Array	105<br>
        5.5	Loading and Storing Multiple Registers	106<br>
        5.6	Exercises	108<br>
        <strong>Chapter 6.	Branch and Conditional Execution	111</strong><br>
        6.1	Condition Testing	111<br>
        6.2	Branch Instructions	114<br>
        6.3	Conditional Execution	117<br>
        6.4	If-then Statement	118<br>
        6.5	If-then-else Statement	121<br>
        6.6	For Loop	122<br>
        6.7	While Loop	123<br>
        6.8	Do While Loop	124<br>
        6.9	Continue Statement	125<br>
        6.10	Break Statement	126<br>
        6.11	Switch Statement	126<br>
        6.12	Exercises	129<br>
        <strong>Chapter 7.	Structured Programming	133</strong><br>
        7.1	Basic Control Structures	133<br>
        7.2	Register Reuse	138<br>
        7.3	Example of Factorial Numbers	141<br>
        7.4	Example of Counting Ones in a Word	142<br>
        7.5	Example of Finding the Maximum of an Array	144<br>
        7.6	Example of Counting Digits	146<br>
        7.7	Example of Parity Bit	147<br>
        7.8	Example of Perfect Numbers	149<br>
        7.9	Example of Armstrong Numbers	151<br>
        7.10	Example of Palindrome String	152<br>
        7.11	Example of Converting String to Integer (atoi)	154<br>
        7.12	Example of Binary Search	155<br>
        7.13	Example of Bubble Sort	157<br>
        7.14	Exercises	159<br>
        <strong>Chapter 8.	Subroutines	161</strong><br>
        8.1	Calling a Subroutine	162<br>
        8.2	Stack	164<br>
        8.3	Implementation of Stack via STM and LDM	165<br>
        8.4	Preserving Runtime Environment via Stack	166<br>
        8.5	Passing Arguments to Subroutine via Registers	169<br>
        8.5.1	Pass a Variable by Value and by Reference	170<br>
        8.5.2	Example of Passing by Value	173<br>
        8.5.3	Write a Subroutine in Different Files	175<br>
        8.5.4	Example of Passing by Reference	176<br>
        8.5.5	Example of Greatest Common Divisor	177<br>
        8.5.6	Example of Concatenating Two Strings	179<br>
        8.5.7	Example of Comparing Two Strings	180<br>
        8.5.8	Example of Inserting an Integer into a Sorted Array	181<br>
        8.5.9	Example of Converting Integer to String (itoa)	182<br>
        8.5.10	Example of Matrix Transpose	184<br>
        8.5.11	Example of Removing a Character from a String	186<br>
        8.5.12	Example of Finding Unique Numbers in an Array	187<br>
        8.6	Passing Arguments through Stack	190<br>
        8.7	Recursive Functions	192<br>
        8.7.1	Example of Factorial Numbers	194<br>
        8.7.2	Example of Reversing a String	196<br>
        8.7.3	Example of String Permutation	197<br>
        8.8	Exercises	199<br>
        <strong>Chapter 9.	64-bit Data Processing	203</strong><br>
        9.1	64-bit Addition	203<br>
        9.2	64-bit Subtraction	204<br>
        9.3	64-bit Counting Leading Zeros	205<br>
        9.4	64-bit Sign Extension	205<br>
        9.5	64-bit Logic Shift Left	206<br>
        9.6	64-bit Logic Shift Right	207<br>
        9.7	64-bit Multiplication	208<br>
        9.8	64-bit Unsigned Division	209<br>
        9.9	64-bit Signed Division	211<br>
        9.10	Exercises	213<br>
        <strong>Chapter 10.	Mixing C and Assembly	215</strong><br>
        10.1	Data Types and Access	216<br>
        10.1.1	Signed or Unsigned Integers	216<br>
        10.1.2	Data Alignment	217<br>
        10.1.3	Data Structure Padding	219<br>
        10.2	Special Variables	222<br>
        10.2.1	Static Variables	222<br>
        10.2.2	Volatile Variables	226<br>
        10.3	Inline Assembly	228<br>
        10.3.1	Assembly Functions in a C Program	228<br>
        10.3.2	Inline Assembly Instructions in a C Program	229<br>
        10.4	Calling Assembly Subroutines from a C Program	230<br>
        10.4.1	Example of Calling an Assembly Subroutine	230<br>
        10.4.2	Example of Accessing C Variables in Assembly	231<br>
        10.5	Calling C Functions from Assembly Programs	232<br>
        10.5.1	Example of Calling a C Function	232<br>
        10.5.2	Example of Accessing Assembly Data in a C Program	233<br>
        10.6	Exercises	234<br>
        <strong>Chapter 11.	Interrupt	237</strong><br>
        11.1	Introduction to Interrupt	237<br>
        11.2	Interrupt Service Routine (ISR)	238<br>
        11.3	Nested Vectored Interrupt Controller (NVIC)	242<br>
        11.3.1	Enable and Disable Peripheral Interrupts	243<br>
        11.3.2	Interrupt Priority	245<br>
        11.3.3	Global Interrupt Enable and Disable	249<br>
        11.4	System Timer	250<br>
        11.5	External Interrupt	258<br>
        11.6	Software Interrupt	261<br>
        11.7	Exercises	262<br>
        <strong>Chapter 12.	Fixed-point and Floating-point Arithmetic	265</strong><br>
        12.1	Fixed-point Arithmetic	266<br>
        12.1.1	Unsigned Fixed-point Representation	267<br>
        12.1.2	Signed Fixed-point Representation	268<br>
        12.1.3	Converting to Fixed-point Format	269<br>
        12.1.4	Fixed-point Range and Resolution Tradeoff	270<br>
        12.1.5	Fixed-point Addition and Subtraction	271<br>
        12.1.6	Fixed-point Multiplication	273<br>
        12.1.7	Fixed-point Division	274<br>
        12.2	Floating-point Arithmetic	275<br>
        12.2.1	Floating-point Representation	275<br>
        12.2.2	Special Values	280<br>
        12.2.3	Overflow and Underflow	281<br>
        12.2.4	Subnormal Numbers	282<br>
        12.2.5	Tradeoff between Numeric Range and Resolution	283<br>
        12.2.6	Rounding Rules	285<br>
        12.3	Software-based Floating-point Operations	288<br>
        12.3.1	Floating-point Addition	289<br>
        12.3.2	Floating-point Multiplication	293<br>
        12.4	Hardware-based Floating-point Operations	295<br>
        12.4.1	FPU Registers	295<br>
        12.4.1.1	Floating-point General-purpose Registers (S0-S31, or D0-D15)	295<br>
        12.4.1.2	Coprocessor Access Control Register (CPACR)	297<br>
        12.4.1.3	Floating-point Status and Control Register (FPSCR)	298<br>
        12.4.1.4	Floating-point Context Address Register (FPCAR)	300<br>
        12.4.1.5	Floating-point Context Control Register (FPCCR)	301<br>
        12.4.2	Load and Store Floating-point Numbers	302<br>
        12.4.3	Copy Floating-point Numbers	303<br>
        12.4.4	Copy and Set the Status and Control Register	303<br>
        12.4.5	Single-precision Arithmetic Operations	304<br>
        12.4.6	Single-precision comparisons	305<br>
        12.4.7	Precision Conversion	306<br>
        12.4.8	FPU Exception and Exception handling	308<br>
        12.4.9	Example Assembly Programs	310<br>
        12.4.9.1	Look up a Float Array	310<br>
        12.4.9.2	Sine Function of Argument in Radians	311<br>
        12.5	Exercises	313<br>
        <strong>Chapter 13.	Instruction Encoding and Decoding	317</strong><br>
        13.1	Tradeoff between Code Density and Performance	317<br>
        13.2	Dividing Bit Streams into 16- or 32-bit Instructions	318<br>
        13.3	Encoding 16-bit Thumb Instructions	320<br>
        13.4	Encoding 32-bit Instructions	321<br>
        13.5	Calculating Target Memory Address	322<br>
        13.6	Instruction Decoding Example 1	324<br>
        13.7	Instruction Decoding Example 2	328<br>
        13.8	Exercises	333<br>
        <strong>Chapter 14.	General Purpose I/O (GPIO)	335</strong><br>
        14.1	Introduction to General Purpose I/O (GPIO)	335<br>
        14.2	GPIO Input Modes: Pull Up and Pull Down	336<br>
        14.3	GPIO Input: Schmitt Trigger	337<br>
        14.4	GPIO Output Modes: Push-Pull and Open-Drain	340<br>
        14.4.1	GPIO Push-Pull Output	340<br>
        14.4.2	GPIO Open-Drain Output	341<br>
        14.5	GPIO Output Speed: Slew Rate	343<br>
        14.6	Lighting up an LED	344<br>
        14.7	Push Button	349<br>
        14.8	Keypad Scan	353<br>
        14.9	Exercises	357<br>
        <strong>Chapter 15.	General-purpose Timers	359</strong><br>
        15.1	Timer Organization and Counting Modes	359<br>
        15.2	Compare Output	362<br>
        15.2.1	Setting Output Mode	363<br>
        15.2.2	Example of Toggling LED	365<br>
        15.3	PWM Output	367<br>
        15.3.1	PWM Output Events	370<br>
        15.3.2	PWM Programming Flowchart	371<br>
        15.4	Input Capture	375<br>
        15.4.1	Input Capture Timer Diagram	377<br>
        15.4.2	Configuring Input Capture	378<br>
        15.4.3	Interfacing to Ultrasonic Distance Sensor	382<br>
        15.5	Exercises	387<br>
        <strong>Chapter 16.	Stepper Motor Control	389</strong><br>
        16.1	Bipolar and Unipolar Stepper Motor	389<br>
        16.2	Step Angle	391<br>
        16.3	Wave Stepping	392<br>
        16.4	Full Stepping	393<br>
        16.5	Half Stepping	395<br>
        16.6	Micro-stepping	397<br>
        16.7	Driving Stepper Motor	399<br>
        16.8	Exercises	400<br>
        <strong>Chapter 17.	Liquid-crystal Display (LCD)	401</strong><br>
        17.1	Static Drive	402<br>
        17.2	Multiplexed Drive	403<br>
        17.3	STM32L Internal LCD Driver	407<br>
        17.3.1	Basic Introduction	407<br>
        17.3.2	Generic LCD Driver to Display Strings	411<br>
        17.4	Interfacing with External Character LCD Controllers	416<br>
        17.4.1	External Connection Diagram	416<br>
        17.4.2	Internal Font Encoding	419<br>
        17.4.3	Sending Commands and Data to LCD	420<br>
        17.4.4	Programming Fonts	423<br>
        17.5	Exercises	424<br>
        <strong>Chapter 18.	Real-time Clock (RTC)	425</strong><br>
        18.1	UNIX Epoch Time	425<br>
        18.2	RTC Frequency Setting	426<br>
        18.3	Binary Coded Decimal (BCD) Encoding	427<br>
        18.4	RTC Initialization	428<br>
        18.5	RTC Alarm	431<br>
        18.6	Exercises	434<br>
        <strong>Chapter 19.	Direct Memory Access (DMA)	435</strong><br>
        19.1	DMA Bus Matrix	435<br>
        19.2	Programming DMA	438<br>
        19.3	Exercises	442<br>
        <strong>Chapter 20.	Analog-to-Digital Converter (ADC)	443</strong><br>
        20.1	ADC Architecture	443<br>
        20.1.1	Digital Quantization	444<br>
        20.1.2	Sampling and Hold	446<br>
        20.2	ADC Sampling Error	447<br>
        20.3	ADC Conversion Modes	449<br>
        20.4	ADC Data Alignment	451<br>
        20.5	ADC Triggers	452<br>
        20.6	Measuring the Input Voltage	455<br>
        20.7	ADC Configuration Flowchart	455<br>
        20.8	ADC with DMA	460<br>
        20.9	Exercises	462<br>
        <strong>Chapter 21.	Digital-to-Analog Converter (DAC)	463</strong><br>
        21.1	DAC Architecture	463<br>
        21.2	DAC on STM32L Processors	464<br>
        21.3	Conversion Trigger	466<br>
        21.4	Buffered Output	467<br>
        21.5	Generating a Sinusoidal Wave via Table Lookup	468<br>
        21.6	Using Timer as a Trigger to DAC	472<br>
        21.7	Musical Synthesizing	476<br>
        21.7.1	Musical Pitch	476<br>
        21.7.2	Musical Duration	477<br>
        21.7.3	Amplitude Modulation of Tones	477<br>
        21.8	Exercises	482<br>
        <strong>Chapter 22.	Serial Communication Protocols	483</strong><br>
        22.1	Universal Asynchronous Receiver and Transmitter	483<br>
        22.1.1	Communication Frame	484<br>
        22.1.2	Baud Rate	485<br>
        22.1.3	Example Program Code in C	488<br>
        22.1.4	Serial Communication to Bluetooth Module	492<br>
        22.2	Inter-Integrated Circuit (I2C)	495<br>
        22.2.1	Interfacing Serial Digital Thermal Sensors via I2C	499<br>
        22.2.2	I2C Clock Control	501<br>
        22.2.3	I2C Maximum Rising Time	502<br>
        22.2.4	Sending Data to I2C Slave	503<br>
        22.2.5	Receiving Data from I2C Slave	504<br>
        22.2.6	Example Program Code in C	507<br>
        22.3	Serial Peripheral Interface Bus (SPI)	513<br>
        22.3.1	Data Exchange	514<br>
        22.3.2	Clock Configuration	516<br>
        22.3.3	Example Program Code in C	517<br>
        22.4	Universal Serial Bus (USB)	521<br>
        22.4.1	USB Bus Layer	522<br>
        22.4.2	USB Device Layer	524<br>
        22.4.3	USB Function Layer	527<br>
        22.4.3.1	USB Descriptors	527<br>
        22.4.3.2	Endpoint-Oriented Communication	530<br>
        22.4.3.3	USB Enumeration	531<br>
        22.4.4	USB Class Layer	535<br>
        22.4.5	Human Interface Device (HID)	536<br>
        22.5	Exercises	541<br>
        <strong>Chapter 23.	Multitasking	543</strong><br>
        23.1	Cortex-M Processor Mode and Privilege	543<br>
        23.2	Supervisor Call (SVC)	545<br>
        23.3	CPU Scheduling	548<br>
        23.4	Exercises	558<br>
        <strong>Chapter 24.	Digital Signal Processing (DSP)	559</strong><br>
        24.1	Fixed-point and Floating-point DSP	559<br>
        24.2	Fixed-point Data Types in DSP	560<br>
        24.3	Saturation	561<br>
        24.4	Arithmetic Instructions	563<br>
        24.4.1	Parallel 8-bit Add and Subtract	565<br>
        24.4.2	Parallel 16-bit Add and Subtract	569<br>
        24.4.3	32-bit Add and Subtract	571<br>
        24.4.4	Sum of Absolute Difference	572<br>
        24.4.5	Extension and Add	573<br>
        24.4.6	Add and Subtract Halfwords with Exchange	575<br>
        24.4.7	16-bit and 32-bit Multiplication	577<br>
        24.4.8	16-bit Multiply and Accumulate with 64-bit Result	580<br>
        24.4.9	16-bit Multiply and Accumulate with 32-bit Result	583<br>
        24.4.10	16&times;32 Multiply and Accumulate with 32-bit Result	585<br>
        24.4.11	32&times;32 Multiply and Accumulate with 32-bit Result	585<br>
        24.4.12	Unsigned Long Multiply with Accumulate Accumulate	586<br>
        24.5	Packing Halfwords into a Word	588<br>
        24.6	Signed and Unsigned Extension	589<br>
        24.7	GE Flags	590<br>
        24.8	Byte Selection Instruction	591<br>
        24.9	Basic DSP Functions	592<br>
        24.9.1	Vector Negate	592<br>
        24.9.2	Vector Absolution Value	593<br>
        24.9.3	Vector Offset with Saturation	595<br>
        24.9.4	Vector Shift Left with Saturation	597<br>
        24.9.5	Vector Mean	598<br>
        24.9.6	Vector Pairwise Multiplication	600<br>
        24.9.7	Vector Dot Product	602<br>
        24.9.8	Vector Min and Max	603<br>
        24.10	Exercises	604<br>
        <strong>Appendix A: Cortex-M3/M4 Instructions	607<br>
        Appendix B: Floating-point Instructions (Optional on Cortex-M4 and Cortex-M7)	609<br>
        Appendix C: DSP Instructions on Cortex-M4 and Cortex-M7	611<br>
        Appendix D: Cortex-M0/M0+/M1 Instructions	615<br>
        Appendix E: Cortex-M3 16-bit Thumb-2 Instruction Encoding	617<br>
        Appendix F: Cortex-M3 32-bit Thumb-2 Instruction Encoding	619<br>
        Appendix G: HID Codes of a Keyboard	626<br>
        Bibliography	628<br>
        Index	633</strong></p>        
        <p></p></td>
    </tr>
  </tbody>
</table>
<p>&nbsp;</p>
</body></html>
