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
.STYLE4 {color: #990000}
.STYLE5 {
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
    <div align="left"><a href="index.php">Home</a> <strong>/ </strong> <a href="intro.php">Introduction </a> <strong>/ </strong><a href="adoptions.php">Adoptions </a> <strong>/ </strong><a href="lecture.php">Instructor Site </a><strong> / </strong><a href="lab.php">Sample Labs </a><strong> / </strong> <span class="STYLE5">Lab Kit</span>  <strong> / </strong> <a href="tutorials.php"> Tutorials </a><strong> / </strong><a href="faq.php">FAQ</a></div>
    </div>
    <div align="left"><font face="Georgia, Times New Roman, Times, serif"><a href="calendar/index.php"> </a></font></div>  </td>
</tr>
</tbody>
</table>
<table width="100%" border="0" align="center">
  <tbody>
    <tr bgcolor="#ffffff">
      <td height="40" colspan="2"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33" colspan="2"><p>The following <em><strong>lab-in-a-box</strong></em> kits are recommended for labs associated with this book. These kits are breadboard friendly. Example project codes are shown <a href="lab.php">here</a>.  </p>
        <ul>
          <li> <a href="http://www.st.com/en/evaluation-tools/nucleo-l476rg.html">Nucleo-64 development board with STM32L476RG (NUCLEO-L476RG)</a>
            <ul>
              <li> $18 from <a href="http://www.mouser.com/access/?pn=nucleo-l476rg&sn=st">Mouser</a> (world wide) and <a href="http://www.digikey.com/scripts/DkSearch/dksus.dll?WT.z_cid=sp_497_0928_buynow&Enterprise=44&lang=en&Vendor=497&mpart=NUCLEO-L476RG">Digikey</a> (world wide).</li>
              <li>On-board ST-Link debugger/programmer</li>
              <li>One user LED, one user  button, one resut button</li>
              <li><strong><em>Currently back ordered, expected in stock on Mouser 2/14/2022</em></strong></li>
            </ul>
          </li>
          <li><a href="https://www.st.com/en/evaluation-tools/nucleo-f401re.html" target="_blank"> Nucleo-64 development board with STM32F401RE (NUCLEO-F401RE)</a>
            <ul>
              <li>$14 from <a href="http://www.mouser.com/access/?pn=NUCLEO-F401RE&sn=st">Mouser</a> and <a href="http://www.digikey.com/scripts/DkSearch/dksus.dll?WT.z_cid=sp_497_0928_buynow&Enterprise=44&lang=en&Vendor=497&mpart=NUCLEO-F401RE">Digikey</a></li>
              <li>On-board ST-Link debugger/programmer</li>
              <li>One user LED, one user  button, one resut button</li>
            </ul>
          </li>
        </ul></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33" colspan="2"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td width="33%" height="33"><img src="STM32L4/NUCLEO-L476RG-4.png" width="430" height="551"></td>
      <td width="67%" height="33"><img src="STM32L4/NUCLEO-L476RG-pins.png" width="638" height="638"></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33" colspan="2"><hr></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33" colspan="2"><span class="STYLE1">Other Lab Parts: </span></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="33" colspan="2"><table width="1295" border="0">
        <tr>
          <td width="116" align="center"><img src="Parts/usb_cable.jpg" width="90" height="103"></td>
          <td width="431">One USB cable (A-Male to Mini-B) (<a href="http://www.amazon.com/s/ref=nb_sb_noss?url=search-alias%3Daps&field-keywords=A-Male+to+Mini-B+USB+Cable&rh=i%3Aaps%2Ck%3AA-Male+to+Mini-B+USB+Cable" target="_new">amazon</a>), ~$5 <br>
For programming/debugging the board <br>
Purchase in volume (<a href="https://www.amazon.com/Mini-Cable-Black-Type-Mini-B/dp/B00LSRZBQM/ref=pd_sbs_328_5/130-1873174-0678424?_encoding=UTF8&amp;pd_rd_i=B00LSRZBQM&amp;pd_rd_r=83c63185-471c-4472-baa4-f43c2814b08b&amp;pd_rd_w=MdUPn&amp;pd_rd_wg=5Naru&amp;pf_rd_p=5873ae95-9063-4a23-9b7e-eafa738c2269&amp;pf_rd_r=3545W8W2SDCA9QZFD31Z&amp;refRID=3545W8W2SDCA9QZFD31Z&amp;th=1" target="_new">amazon</a>): $20 for 10 pcs</td>
          <td width="117" align="center"><img src="Parts/SSD1306_OLED.png" width="107" height="107"></td>
          <td width="537"><p>I2C SSD1306 OLED Display (<a href="https://www.amazon.com/Dorhea-Display-SSD1306-Self-Luminous-Raspberry/dp/B07FK8GB8T/ref=sr_1_5?dchild=1&keywords=SSD1306%2BOLED&qid=1609373955&s=industrial&sr=1-5&th=1" target="_new">amazon</a>), ~$7<br>
          	(Optional) Can be used in all labs. If you not have it, you can use ST-Link virtual com to display information on your computer. <br>
            Purchase in volume (<a href="https://www.amazon.com/Dorhea-Display-SSD1306-Self-Luminous-Raspberry/dp/B07WPCPM5H/ref=sr_1_5?dchild=1&keywords=SSD1306+OLED&qid=1609373955&s=industrial&sr=1-5" target="_new">amazon</a>), $24 for 6 pcs 
          </td>
        </tr>
        <tr>
          <td align="center"><img src="Parts/wires.jpg" width="80" height="80"></td>
          <td>Breadboard jumper wires, mixed of F/F and M/F, such as (<a href="http://www.amazon.com/Jumper-female-200mm-length-50pcs/dp/B00AC4NQYG/ref=sr_1_9?s=pc&ie=UTF8&qid=1455769146&sr=1-9&keywords=breadboard+jumper+wires"  target="_new">amazon</a>)</td>
          <td align="center"><img src="Parts/Breadboard.jpg" width="107" height="91"></td>
          <td>Solderless breadboard (<a href="https://www.amazon.com/AkoMatial-Solderless-Breadboard-Protoboard-Prototyping/dp/B0834ZR2VT/ref=sr_1_15?dchild=1&keywords=Solderless+Breadboard&qid=1609374818&sr=8-15" target="_new">amazon</a>), $6 for 2 pcs<br>
          Purchase in volume (<a href="https://www.amazon.com/DEYUE-breadboard-Set-Prototype-Board/dp/B07LFD4LT6/ref=sr_1_4?dchild=1&keywords=Solderless+Breadboard&qid=1609374596&sr=8-4" target="_new">amazon</a>), $9 for 6 pcs</td>
        </tr>
        <tr>
          <td><img src="Parts/keypad.jpg" width="116" height="106"></td>
          <td>One 4 x 4 matrix keypad (<a href="http://www.amazon.com/s/ref=nb_sb_noss?url=search-alias%3Daps&field-keywords=Matrix+Array+Keypad&rh=i%3Aaps%2Ck%3AMatrix+Array+Keypad" target="_new">amazon</a>), ~$5 <br>
Used in the lab of GPIO input and outputs <br>
Purchase in volume (<a href="https://www.amazon.com/Diymore-Matrix-Membrane-Keyboard-Arduino/dp/B01KJLOBYK/ref=sr_1_4?s=electronics&amp;ie=UTF8&amp;qid=1515430434&amp;sr=1-4&amp;keywords=10PCS+keypads" target="_new">amazon</a>), $12 for 10 pcs </td>
          <td><img src="Parts/motor.jpg" width="116" height="115"></td>
          <td>One 28BYJ-48  5v stepper motor + ULN2003 driver board (<a href="http://www.amazon.com/s/ref=nb_sb_noss?url=search-alias%3Daps&field-keywords=28BYJ-48+stepper+motor+ULN2003&rh=i%3Aaps%2Ck%3A28BYJ-48+stepper+motor+ULN2003" target="_new">amazon</a>), ~$3<br>
Used in the lab of GPIO output (full/half stepping) or Timer's PWM output (micro-stepping) <br>
Purchase in volume (<a href="https://www.amazon.com/LAFVIN-28BYJ-48-ULN2003-Stepper-Arduino/dp/B076KDFSGT/ref=sr_1_2?s=electronics&amp;ie=UTF8&amp;qid=1515430531&amp;sr=1-2&amp;keywords=28BYJ-48+10pcs" target="_new">amazon</a>), $12.5 for 5 pcs</td>
        </tr>
        <tr>
          <td><img src="Parts/sg90_servo.png" width="116" height="115"></td>
          <td>SG90 Micro Servo Motor (<a href="https://www.amazon.com/s?k=sg90+servo&ref=nb_sb_noss_1" target="_new">amazon</a>), ~$3 <br>
Used in the lab of timer PWM output <br>
Purchase in volume (<a href="https://www.amazon.com/Smraza-Helicopter-Airplane-Control-Arduino/dp/B07L2SF3R4/ref=sr_1_1_sspa?dchild=1&keywords=sg90+servo&qid=1608927047&sr=8-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUExM1U1T1NNWk5IVDBJJmVuY3J5cHRlZElkPUEwNTExOTY0MUJUOUtPMFFLVlIwNSZlbmNyeXB0ZWRBZElkPUEwMjA3MzA3R1FIMVdOSE1VREU0JndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==" target="_new">amazon</a>), $18.59 for 10 pcs </td>
          <td><img src="Parts/ultrasonic.jpg" width="117" height="102"></td>
          <td>One HC-SR04 ultrasonic distance measuring sensor (<a href="http://www.amazon.com/s/ref=nb_sb_noss?url=search-alias%3Daps&field-keywords=HC-SR04+Ultrasonic+Distance+Measuring+Sensor&rh=i%3Aaps%2Ck%3AHC-SR04+Ultrasonic+Distance+Measuring+Sensor" target="_new">amazon</a>), ~$5 <br>
Used in the lab of timer input capture <br>
Purchase in volume (<a href="https://www.amazon.com/WMYCONGCONG-HC-SR04-Ultrasonic-Distance-Measuring/dp/B07JJHCVRG/ref=sr_1_7?dchild=1&keywords=HC-SR04+Ultrasonic+Distance+Measuring+Sensor&qid=1608927132&sr=8-7" target="_new">amazon</a>), $13.99 for 10 pcs </td>
        </tr>
        <tr>
          <td align="center"><img src="Parts/TC74-TO-220-5.png" width="104" height="83"></td>
          <td>Potentiometer (<a href="https://www.digikey.com/en/products/detail/microchip-technology/TC74A0-3-3VAT/442720?s=N4IgTCBcDaICoGEDsAWAggBgLQGYB0OAamnFgHIAiIAugL5A" target="_new">digikey</a>) <br>
Used in the lab of I2C communication<br>
Digikey Number: TC74A0-3.3VAT-ND, Quantity: 1, Price: $1.37</td>
          <td align="center"><img src="Parts/3352E SERIES_sml.jpg" width="104" height="83"></td>
          <td>Potentiometer (<a href="http://www.digikey.com/product-detail/en/3352E-1-103LF/3352E-103LF-ND/1088309" target="_new">digikey</a>) <br>
Used in the lab of ADC <br>
Digikey Number: 3352E-103LF-ND, Quantity: 1, Price: $1.53 </td>
        </tr>
        <tr>
          <td align="center"><img src="Parts/QED123_sml.jpg" width="77" height="75"></td>
          <td>Infrared light emitting diode (<a href="http://www.digikey.com/product-detail/en/QED123/QED123-ND/187398" target="_new">Digikey</a>) <br>
Used in the lab of ADC infrared distance sensor (optional)<br>
Digikey Number: QED123-ND, Quantity: 1, Price: $0.53</td>
          <td align="center"><img src="Parts/QSD122,QSD123,QSD124_sml.jpg" width="75" height="78"></td>
          <td>Infrared phototransistor (<a href="http://www.digikey.com/product-search/en?KeyWords=QSD124-ND" target="_new">Digikey</a>) <br>
Used in the lab of ADC infrared distance sensor (optional)<br>
Digikey Number: QED124-ND, Quantity: 1, Price: $0.51</td>
        </tr>
        <tr>
          <td align="center"><img src="Parts/2N3904.jpg" width="75" height="76"></td>
          <td>2N3904 NPNnpn transistor (<a href="http://www.digikey.com/product-detail/en/2N3904BU/2N3904FS-ND/1413" target="_new">digikey</a>), <br>
Used in the lab of ADC infrared distance sensor  (optional)<br>
Digikey Number: 2N3904FS-ND, Quantity: 1, Price: $0.18</td>
          <td align="center"><img src="Parts/SJ1-3553NG.png" width="75" height="76"></td>
          <td>Stereo Jack 3pin 3.5mm  (<a href="https://www.digikey.com/en/products/detail/cui-devices/SJ1-3553NG/738710?s=N4IgTCBcDaIMIAUCMBaAzAVg2gcgcRRwBEQBdAXyA" target="_new">digikey</a>), <br>
Used in the lab of DAC music<br>
Digikey Number: CP1-3553NG-ND, Quantity: 1, Price: $0.88</td>
        </tr>
        <tr>
          <td align="center"><img src="Parts/PKM22EPPH4001.jpg" width="106" height="97"></td>
          <td>Murata Electronics North America PKM22EPPH4001-B0 Buzzer (<a href="http://www.digikey.com/product-detail/en/murata-electronics-north-america/PKM22EPPH4001-B0/490-4692-ND/1219323" target="_new">digikey</a>)<br>
Used for the lab of PWM music (optional)<br>
Digikey Number 490-4692-ND, Quantity: 1, Price: $1.25</td>
          <td valign="middle"><img src="Parts/2_2KOhm.jpg" width="95" height="80"></td>
          <td>Through Hole 2.2K Ohm Resistor (<a href="http://www.digikey.com/product-detail/en/CFR-25JB-52-2K2/2.2KQBK-ND/666" target="_new">digikey</a>), <br>
Used with 4x4 keypad as pull-up resistors <br>
Digikey Number: 2.2KQBK-ND, Quantity: 4, Total price: $0.10 </td>
        </tr>
        <tr>
          <td align="center"><img src="Parts/5_6KOhm.jpg" width="92" height="73"></td>
          <td>Through Hole 5.6K Ohm Resistor (<a href="http://www.digikey.com/product-search/en?keywords=5.6KQBK-ND" target="_new">digikey</a>), <br>
Digikey Number: 5.6KQBK-ND, Quantity: 1, Total price: $0.10 </td>
          <td align="center"><img src="Parts/100Ohm.jpg" width="84" height="67"></td>
          <td>Through Hole 100 Ohm Resistor (<a href="http://www.digikey.com/product-detail/en/yageo/CFR-25JR-52-100R/100QTR-ND/11950" target="_new">digikey</a>), <br>
Digikey Number: 100QBK-ND, Quantity: 1, Total price: $0.08 </td>
        </tr>
        <tr>
          <td align="center"><img src="Parts/150Ohm.jpg" width="84" height="67"></td>
          <td>Through Hole 150 Ohm Resistor (<a href="http://www.digikey.com/product-search/en?keywords=150DVCT-ND" target="_new">digikey</a>), <br>
Digikey Number: 150QBK-ND, Quantity: 1, Total price: $0.16 </td>
          <td>&nbsp;</td>
          <td>&nbsp;</td>
        </tr>
      </table></td>
    </tr>
    <tr bgcolor="#ffffff">
      <td height="30" colspan="2"><hr></td>
    </tr>
  </tbody>
</table>
<p>&nbsp;</p>
</body></html>
