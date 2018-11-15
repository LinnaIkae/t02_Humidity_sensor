EESchema Schematic File Version 4
LIBS:hwpboard-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Soil Humidity Sensor"
Date "2018-10-25"
Rev ""
Comp "Team 02"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Barrel_Jack_Switch J1
U 1 1 5B990E83
P 950 1250
F 0 "J1" H 1005 1567 50  0000 C CNN
F 1 "Barrel_Jack_Switch" H 1005 1476 50  0000 C CNN
F 2 "HWP:BarrelJack_Horizontal_CustomHWP" H 1000 1210 50  0001 C CNN
F 3 "~" H 1000 1210 50  0001 C CNN
	1    950  1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5B991004
P 1350 1550
F 0 "#PWR01" H 1350 1300 50  0001 C CNN
F 1 "GND" H 1355 1377 50  0000 C CNN
F 2 "" H 1350 1550 50  0001 C CNN
F 3 "" H 1350 1550 50  0001 C CNN
	1    1350 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 1350 1350 1450
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 5BAB792F
P 10250 10750
F 0 "J2" H 10330 10792 50  0000 L CNN
F 1 "Conn_01x01" H 10330 10701 50  0000 L CNN
F 2 "HWP:MountingHole_4.1mm_M4_ISO14580_Pad" H 10250 10750 50  0001 C CNN
F 3 "~" H 10250 10750 50  0001 C CNN
	1    10250 10750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 5BAB7A7C
P 11100 10750
F 0 "J4" H 11180 10792 50  0000 L CNN
F 1 "Conn_01x01" H 11180 10701 50  0000 L CNN
F 2 "HWP:MountingHole_4.1mm_M4_ISO14580_Pad" H 11100 10750 50  0001 C CNN
F 3 "~" H 11100 10750 50  0001 C CNN
	1    11100 10750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 5BAB7AA2
P 10250 10950
F 0 "J3" H 10330 10992 50  0000 L CNN
F 1 "Conn_01x01" H 10330 10901 50  0000 L CNN
F 2 "HWP:MountingHole_4.1mm_M4_ISO14580_Pad" H 10250 10950 50  0001 C CNN
F 3 "~" H 10250 10950 50  0001 C CNN
	1    10250 10950
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J5
U 1 1 5BAB7AE2
P 11100 10950
F 0 "J5" H 11180 10992 50  0000 L CNN
F 1 "Conn_01x01" H 11180 10901 50  0000 L CNN
F 2 "HWP:MountingHole_4.1mm_M4_ISO14580_Pad" H 11100 10950 50  0001 C CNN
F 3 "~" H 11100 10950 50  0001 C CNN
	1    11100 10950
	1    0    0    -1  
$EndComp
Text Notes 10500 10550 0    50   ~ 0
Mounting Holes
Wire Notes Line
	9950 10450 9950 11100
Wire Notes Line
	9950 11100 11700 11100
Wire Notes Line
	11700 11100 11700 10450
Wire Notes Line
	11700 10450 9950 10450
Wire Wire Line
	1350 1350 1250 1350
NoConn ~ 10050 10750
NoConn ~ 10050 10950
NoConn ~ 10900 10750
NoConn ~ 10900 10950
NoConn ~ 1250 1250
$Comp
L Display_Character:RC1602A LCD1
U 1 1 5BD1FB1D
P 8700 5750
F 0 "LCD1" H 8700 6628 50  0000 C CNN
F 1 "RC1602A" H 8700 6537 50  0000 C CNN
F 2 "HWP:WC1602A_no_mount" H 8800 4950 50  0001 C CNN
F 3 "http://www.raystar-optronics.com/down.php?ProID=18" H 8800 5650 50  0001 C CNN
	1    8700 5750
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x05_Male MoistureSensor1
U 1 1 5BD1FE66
P 8100 7250
F 0 "MoistureSensor1" H 8073 7180 50  0000 R CNN
F 1 "Conn_01x05_Male" H 8073 7271 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8100 7250 50  0001 C CNN
F 3 "~" H 8100 7250 50  0001 C CNN
	1    8100 7250
	-1   0    0    1   
$EndComp
$Comp
L HWP:NUCLEO64 NUCBRD1
U 1 1 5BD1FF66
P 5050 3700
F 0 "NUCBRD1" H 5000 1564 50  0000 C CNN
F 1 "NUCLEO64" H 5000 1473 50  0000 C CNN
F 2 "HWP:ST_Morpho_Connector_STLink" H 4950 4100 50  0001 C CNN
F 3 "" H 5050 4200 50  0001 C CNN
	1    5050 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 7450 7650 7450
Wire Wire Line
	7900 7050 7650 7050
$Comp
L Device:Rotary_Encoder_Switch RotaryEncoder1
U 1 1 5BD20ADA
P 9900 3250
F 0 "RotaryEncoder1" H 9900 3617 50  0000 C CNN
F 1 "Rotary_Encoder_Switch" H 9900 3526 50  0000 C CNN
F 2 "HWP:RotaryEncoder_Alps_EC12E-Switch_Vertical_H20mm_CircularMountingHoles" H 9750 3410 50  0001 C CNN
F 3 "~" H 9900 3510 50  0001 C CNN
	1    9900 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 5750 4400 5800
$Comp
L power:+5V #PWR09
U 1 1 5BD21A18
P 8700 4800
F 0 "#PWR09" H 8700 4650 50  0001 C CNN
F 1 "+5V" H 8715 4973 50  0000 C CNN
F 2 "" H 8700 4800 50  0001 C CNN
F 3 "" H 8700 4800 50  0001 C CNN
	1    8700 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5BD21A7D
P 8850 6500
F 0 "#PWR010" H 8850 6250 50  0001 C CNN
F 1 "GND" H 8855 6327 50  0000 C CNN
F 2 "" H 8850 6500 50  0001 C CNN
F 3 "" H 8850 6500 50  0001 C CNN
	1    8850 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5BD21AFB
P 8250 5350
F 0 "#PWR08" H 8250 5100 50  0001 C CNN
F 1 "GND" H 8255 5177 50  0000 C CNN
F 2 "" H 8250 5350 50  0001 C CNN
F 3 "" H 8250 5350 50  0001 C CNN
	1    8250 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2650 6650 2650
Wire Wire Line
	6100 2750 6650 2750
Wire Wire Line
	8700 4800 8700 4950
$Comp
L power:GND #PWR014
U 1 1 5BD2289C
P 10350 6450
F 0 "#PWR014" H 10350 6200 50  0001 C CNN
F 1 "GND" H 10355 6277 50  0000 C CNN
F 2 "" H 10350 6450 50  0001 C CNN
F 3 "" H 10350 6450 50  0001 C CNN
	1    10350 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 5550 9300 5550
Wire Wire Line
	8300 5350 8250 5350
Wire Wire Line
	8300 5250 8050 5250
Wire Wire Line
	8300 5550 8050 5550
Wire Wire Line
	8300 5450 8050 5450
Wire Wire Line
	8300 5750 8050 5750
Wire Wire Line
	8300 5850 8050 5850
Wire Wire Line
	8300 5950 8050 5950
Wire Wire Line
	8300 6050 8050 6050
Wire Wire Line
	8050 6150 8300 6150
Wire Wire Line
	8300 6250 8050 6250
Wire Wire Line
	3900 4450 3400 4450
Wire Wire Line
	3900 4650 3400 4650
Wire Wire Line
	3900 4750 3400 4750
Wire Wire Line
	6100 3050 6650 3050
Wire Wire Line
	7650 7450 7650 7600
Wire Wire Line
	7400 7350 7900 7350
$Comp
L Switch:SW_Push PushButton1
U 1 1 5BD347D1
P 9950 2550
F 0 "PushButton1" H 9950 2835 50  0000 C CNN
F 1 "SW_Push" H 9950 2744 50  0000 C CNN
F 2 "HWP:SW_PUSH_6mm" H 9950 2750 50  0001 C CNN
F 3 "" H 9950 2750 50  0001 C CNN
	1    9950 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2550 9750 2550
Wire Wire Line
	4700 1750 4700 1500
Text GLabel 10250 2550 2    50   Input ~ 0
btn1
Text GLabel 6600 4450 2    50   Input ~ 0
btn1
Text GLabel 6650 4950 2    50   Input ~ 0
rotary0
Text GLabel 6650 3050 2    50   Input ~ 0
rotary1
Text GLabel 6600 4350 2    50   Input ~ 0
btn0
Text GLabel 9000 3150 0    50   Input ~ 0
rotary0
Text GLabel 9000 3350 0    50   Input ~ 0
rotary1
Wire Wire Line
	10200 3150 10550 3150
Text GLabel 10550 3150 2    50   Input ~ 0
btn0
$Comp
L power:GND #PWR05
U 1 1 5BD40D21
P 5150 7650
F 0 "#PWR05" H 5150 7400 50  0001 C CNN
F 1 "GND" H 5155 7477 50  0000 C CNN
F 2 "" H 5150 7650 50  0001 C CNN
F 3 "" H 5150 7650 50  0001 C CNN
	1    5150 7650
	1    0    0    -1  
$EndComp
Text GLabel 4450 7400 0    50   Input ~ 0
PWM_LCD
Text GLabel 6700 4750 2    50   Input ~ 0
PWM_LCD
Text GLabel 6650 2650 2    50   Input ~ 0
DAC0
Text GLabel 9300 5550 2    50   Input ~ 0
DAC0
Text GLabel 5450 7150 2    50   Input ~ 0
brightness
Text GLabel 9350 5950 2    50   Input ~ 0
brightness
Wire Wire Line
	8300 5650 8050 5650
Text GLabel 8050 5550 0    50   Input ~ 0
d0
Text GLabel 8050 5650 0    50   Input ~ 0
d1
Text GLabel 8050 5750 0    50   Input ~ 0
d2
Text GLabel 8050 5850 0    50   Input ~ 0
d3
Text GLabel 8050 5950 0    50   Input ~ 0
d4
Text GLabel 8050 6050 0    50   Input ~ 0
d5
Text GLabel 8050 6150 0    50   Input ~ 0
d6
Text GLabel 8050 6250 0    50   Input ~ 0
d7
Text GLabel 3400 4450 0    50   Input ~ 0
d0
Text GLabel 6700 4850 2    50   Input ~ 0
d1
Text GLabel 3400 4350 0    50   Input ~ 0
d2
Text GLabel 6650 2750 2    50   Input ~ 0
d3
Text GLabel 6650 2850 2    50   Input ~ 0
d4
Text GLabel 6650 2950 2    50   Input ~ 0
d5
Text GLabel 6750 5150 2    50   Input ~ 0
d6
Text GLabel 6700 4550 2    50   Input ~ 0
d7
Text GLabel 8050 5250 0    50   Input ~ 0
RS
Text GLabel 3400 4650 0    50   Input ~ 0
RS
Text GLabel 8050 5450 0    50   Input ~ 0
E
Text GLabel 3400 4750 0    50   Input ~ 0
E
$Comp
L power:+3.3V #PWR03
U 1 1 5BD624F4
P 4700 1500
F 0 "#PWR03" H 4700 1350 50  0001 C CNN
F 1 "+3.3V" H 4715 1673 50  0000 C CNN
F 2 "" H 4700 1500 50  0001 C CNN
F 3 "" H 4700 1500 50  0001 C CNN
	1    4700 1500
	1    0    0    -1  
$EndComp
Text GLabel 7400 7150 0    50   Input ~ 0
humid_out
Text GLabel 7400 7250 0    50   Input ~ 0
temp_out
Text GLabel 7400 7350 0    50   Input ~ 0
PWM_sensor
Text GLabel 3350 3850 0    50   Input ~ 0
humid_out
Text GLabel 3350 3950 0    50   Input ~ 0
temp_out
Text GLabel 6700 4650 2    50   Input ~ 0
PWM_sensor
Wire Wire Line
	6100 2850 6650 2850
NoConn ~ 3900 2250
NoConn ~ 3900 2350
NoConn ~ 4600 1750
NoConn ~ 4900 1750
NoConn ~ 5000 1750
NoConn ~ 5100 1750
NoConn ~ 5500 1750
NoConn ~ 6100 2250
NoConn ~ 6100 2350
NoConn ~ 6100 2450
NoConn ~ 6100 2550
NoConn ~ 6100 3650
NoConn ~ 6100 3750
NoConn ~ 6100 3950
NoConn ~ 6100 4150
NoConn ~ 6100 4250
NoConn ~ 6100 4350
NoConn ~ 6100 5050
$Comp
L power:GND #PWR02
U 1 1 5BD94206
P 4400 6000
F 0 "#PWR02" H 4400 5750 50  0001 C CNN
F 1 "GND" H 4405 5827 50  0000 C CNN
F 2 "" H 4400 6000 50  0001 C CNN
F 3 "" H 4400 6000 50  0001 C CNN
	1    4400 6000
	1    0    0    -1  
$EndComp
Connection ~ 4400 5800
Wire Wire Line
	4400 5800 4400 6000
Wire Wire Line
	4600 5800 4600 5750
Wire Wire Line
	4600 5800 4700 5800
Wire Wire Line
	4700 5800 4700 5750
Connection ~ 4600 5800
Wire Wire Line
	4700 5800 4800 5800
Wire Wire Line
	4800 5800 4800 5750
Connection ~ 4700 5800
Wire Wire Line
	4800 5800 4900 5800
Connection ~ 4800 5800
Wire Wire Line
	5500 5750 5500 5950
NoConn ~ 6100 5450
NoConn ~ 3900 3350
NoConn ~ 3900 3450
NoConn ~ 3900 3600
NoConn ~ 3900 3700
NoConn ~ 3900 4050
NoConn ~ 3900 4150
NoConn ~ 3900 4250
NoConn ~ 3900 4850
NoConn ~ 3900 5050
NoConn ~ 3900 5150
NoConn ~ 3900 5250
NoConn ~ 3900 5350
Wire Wire Line
	8700 6450 8700 6500
Wire Wire Line
	8700 6500 8850 6500
Wire Wire Line
	4900 5750 4900 5800
Wire Wire Line
	4400 5800 4500 5800
Wire Wire Line
	4500 5800 4500 5750
Connection ~ 4500 5800
Wire Wire Line
	4500 5800 4600 5800
Wire Wire Line
	10150 2550 10250 2550
$Comp
L power:GND #PWR015
U 1 1 5BDE33F0
P 10550 3500
F 0 "#PWR015" H 10550 3250 50  0001 C CNN
F 1 "GND" H 10555 3327 50  0000 C CNN
F 2 "" H 10550 3500 50  0001 C CNN
F 3 "" H 10550 3500 50  0001 C CNN
	1    10550 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 3350 10550 3350
Wire Wire Line
	9000 3150 9600 3150
$Comp
L Device:C C1
U 1 1 5BEABE70
P 2000 1300
F 0 "C1" H 2115 1346 50  0000 L CNN
F 1 "100n" H 2115 1255 50  0000 L CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2038 1150 50  0001 C CNN
F 3 "~" H 2000 1300 50  0001 C CNN
	1    2000 1300
	-1   0    0    1   
$EndComp
Wire Wire Line
	8700 4950 10350 4950
Connection ~ 8700 4950
Wire Wire Line
	8700 4950 8700 5050
$Comp
L Device:C C2
U 1 1 5BEB900A
P 10350 5600
F 0 "C2" H 10465 5646 50  0000 L CNN
F 1 "100n" H 10465 5555 50  0000 L CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10388 5450 50  0001 C CNN
F 3 "~" H 10350 5600 50  0001 C CNN
	1    10350 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 4950 10350 5450
Wire Wire Line
	3350 3850 3900 3850
Wire Wire Line
	3900 3950 3350 3950
Wire Wire Line
	7900 7150 7400 7150
Wire Wire Line
	7400 7250 7900 7250
$Comp
L power:+5V #PWR04
U 1 1 5BEE2D16
P 4800 1250
F 0 "#PWR04" H 4800 1100 50  0001 C CNN
F 1 "+5V" H 4815 1423 50  0000 C CNN
F 2 "" H 4800 1250 50  0001 C CNN
F 3 "" H 4800 1250 50  0001 C CNN
	1    4800 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 1250 4800 1750
Connection ~ 1350 1450
Wire Wire Line
	1350 1450 1350 1550
NoConn ~ 4400 1750
Wire Wire Line
	4500 1750 4500 1150
Wire Wire Line
	1250 1150 1450 1150
Wire Wire Line
	10350 5750 10350 6450
Wire Wire Line
	9100 5950 9350 5950
$Comp
L power:+5V #PWR013
U 1 1 5BF0051B
P 9950 6050
F 0 "#PWR013" H 9950 5900 50  0001 C CNN
F 1 "+5V" H 9965 6223 50  0000 C CNN
F 2 "" H 9950 6050 50  0001 C CNN
F 3 "" H 9950 6050 50  0001 C CNN
	1    9950 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 6050 9950 6050
Wire Wire Line
	2450 1150 2700 1150
$Comp
L Power_Protection:ZEN056V115A24LS Dz1
U 1 1 5BF234A2
P 2400 1250
F 0 "Dz1" H 2400 1617 50  0000 C CNN
F 1 "ZEN056V115A24LS" H 2400 1526 50  0000 C CNN
F 2 "Diode_SMD:Littelfuse_PolyZen-LS" H 2400 900 50  0001 C CNN
F 3 "http://m.littelfuse.com/~/media/electronics/datasheets/polyzen_devices/littelfuse_polyzen_standard_polyzen_catalog_datasheet.pdf.pdf" H 2400 1650 50  0001 C CNN
	1    2400 1250
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSS138 Q1
U 1 1 5BF23B14
P 5050 7400
F 0 "Q1" H 5255 7446 50  0000 L CNN
F 1 "BSS138" H 5255 7355 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5250 7325 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 5050 7400 50  0001 L CNN
	1    5050 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 7150 5450 7150
Wire Wire Line
	5150 7150 5150 7200
Wire Wire Line
	5150 7600 5150 7650
Wire Wire Line
	4450 7400 4850 7400
$Comp
L power:GND #PWR012
U 1 1 5BF35285
P 9450 3500
F 0 "#PWR012" H 9450 3250 50  0001 C CNN
F 1 "GND" H 9455 3327 50  0000 C CNN
F 2 "" H 9450 3500 50  0001 C CNN
F 3 "" H 9450 3500 50  0001 C CNN
	1    9450 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5BF3531E
P 9400 2550
F 0 "#PWR011" H 9400 2300 50  0001 C CNN
F 1 "GND" H 9405 2377 50  0000 C CNN
F 2 "" H 9400 2550 50  0001 C CNN
F 3 "" H 9400 2550 50  0001 C CNN
	1    9400 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 3500 9450 3250
Wire Wire Line
	9450 3250 9600 3250
Wire Wire Line
	9000 3350 9600 3350
Wire Wire Line
	10550 3500 10550 3350
Text Notes 4550 7000 0    50   ~ 0
PWM driver for the LCD backlight
Text Notes 7400 6800 0    50   ~ 0
Sensor connector
Text Notes 8950 4700 0    50   ~ 0
LCD module
Text Notes 9600 2100 0    50   ~ 0
Control buttons
Wire Wire Line
	3400 4350 3900 4350
Wire Wire Line
	6100 2950 6650 2950
Wire Wire Line
	6700 4850 6100 4850
Wire Wire Line
	6700 4750 6100 4750
Wire Wire Line
	6700 4650 6100 4650
Wire Wire Line
	6100 4450 6600 4450
Wire Wire Line
	6600 4350 6100 4350
Wire Wire Line
	6100 4950 6650 4950
NoConn ~ 6100 3350
NoConn ~ 6100 3450
Wire Wire Line
	6700 4550 6100 4550
Wire Wire Line
	6750 5150 6100 5150
NoConn ~ 6100 5250
NoConn ~ 6100 5350
NoConn ~ 6100 4050
NoConn ~ 6100 3550
NoConn ~ 6100 3250
NoConn ~ 6100 3150
NoConn ~ 3900 4550
$Comp
L power:GND #PWR07
U 1 1 5BF8501F
P 7650 7600
F 0 "#PWR07" H 7650 7350 50  0001 C CNN
F 1 "GND" H 7655 7427 50  0000 C CNN
F 2 "" H 7650 7600 50  0001 C CNN
F 3 "" H 7650 7600 50  0001 C CNN
	1    7650 7600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5BF85046
P 5500 5950
F 0 "#PWR06" H 5500 5700 50  0001 C CNN
F 1 "GND" H 5505 5777 50  0000 C CNN
F 2 "" H 5500 5950 50  0001 C CNN
F 3 "" H 5500 5950 50  0001 C CNN
	1    5500 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4950 3400 4950
Text GLabel 3400 4950 0    50   Input ~ 0
Sensor_enable
Text GLabel 7400 6950 0    50   Input ~ 0
Sensor_enable
Wire Wire Line
	7400 6950 7650 6950
Wire Wire Line
	7650 6950 7650 7050
$Comp
L Device:CP C3
U 1 1 5BEDB755
P 1450 1300
F 0 "C3" H 1568 1346 50  0000 L CNN
F 1 "100u" H 1568 1255 50  0000 L CNN
F 2 "HWP:CP_Radial_D7.5mm_P2.50mm" H 1488 1150 50  0001 C CNN
F 3 "~" H 1450 1300 50  0001 C CNN
	1    1450 1300
	1    0    0    -1  
$EndComp
Connection ~ 1450 1150
Connection ~ 1450 1450
Wire Wire Line
	1450 1450 1350 1450
Connection ~ 2700 1150
Wire Wire Line
	2700 1150 4500 1150
Connection ~ 2000 1450
Wire Wire Line
	1450 1450 2000 1450
Wire Wire Line
	1450 1150 2000 1150
Connection ~ 2000 1150
Wire Wire Line
	2000 1150 2100 1150
Wire Wire Line
	2000 1450 2400 1450
$EndSCHEMATC
