EESchema Schematic File Version 4
LIBS:belt-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:USB_B_Micro J2
U 1 1 5E2137E5
P 1300 4800
F 0 "J2" H 1357 5267 50  0000 C CNN
F 1 "USB_B_Micro" H 1357 5176 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1450 4750 50  0001 C CNN
F 3 "~" H 1450 4750 50  0001 C CNN
	1    1300 4800
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J4
U 1 1 5E214297
P 1300 6600
F 0 "J4" H 1357 7067 50  0000 C CNN
F 1 "USB_B_Micro" H 1357 6976 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1450 6550 50  0001 C CNN
F 3 "~" H 1450 6550 50  0001 C CNN
	1    1300 6600
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J3
U 1 1 5E213AB5
P 1300 5700
F 0 "J3" H 1357 6167 50  0000 C CNN
F 1 "USB_B_Micro" H 1357 6076 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1450 5650 50  0001 C CNN
F 3 "~" H 1450 5650 50  0001 C CNN
	1    1300 5700
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J1
U 1 1 5E224B3A
P 1300 3900
F 0 "J1" H 1357 4367 50  0000 C CNN
F 1 "USB_B_Micro" H 1357 4276 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1450 3850 50  0001 C CNN
F 3 "~" H 1450 3850 50  0001 C CNN
	1    1300 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5E224B41
P 1600 4300
F 0 "#PWR0104" H 1600 4050 50  0001 C CNN
F 1 "GND" V 1605 4172 50  0000 R CNN
F 2 "" H 1600 4300 50  0001 C CNN
F 3 "" H 1600 4300 50  0001 C CNN
	1    1600 4300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 5E28EBE0
P 2750 3950
F 0 "R1" H 2818 3996 50  0000 L CNN
F 1 "10K" H 2818 3905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2750 3950 50  0001 C CNN
F 3 "~" H 2750 3950 50  0001 C CNN
	1    2750 3950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0105
U 1 1 5E291789
P 2750 3650
F 0 "#PWR0105" H 2750 3500 50  0001 C CNN
F 1 "+3.3V" H 2765 3823 50  0000 C CNN
F 2 "" H 2750 3650 50  0001 C CNN
F 3 "" H 2750 3650 50  0001 C CNN
	1    2750 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 3700 3100 3700
Wire Wire Line
	3100 3700 3100 3950
Wire Wire Line
	2750 3700 2750 3650
Connection ~ 2750 3700
Wire Wire Line
	2750 3700 2750 3850
Wire Wire Line
	2750 4050 2750 4250
Wire Wire Line
	2750 4250 2900 4250
Connection ~ 2750 4250
Wire Wire Line
	2450 4250 2750 4250
$Comp
L Transistor_FET:BSS138 Q2
U 1 1 5E33557E
P 3100 5350
F 0 "Q2" V 3351 5350 50  0000 C CNN
F 1 "BSS138" V 3442 5350 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3300 5275 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 3100 5350 50  0001 L CNN
	1    3100 5350
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R3
U 1 1 5E335584
P 2750 5150
F 0 "R3" H 2818 5196 50  0000 L CNN
F 1 "10K" H 2818 5105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2750 5150 50  0001 C CNN
F 3 "~" H 2750 5150 50  0001 C CNN
	1    2750 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R4
U 1 1 5E33558A
P 3400 5150
F 0 "R4" H 3468 5196 50  0000 L CNN
F 1 "10K" H 3468 5105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3400 5150 50  0001 C CNN
F 3 "~" H 3400 5150 50  0001 C CNN
	1    3400 5150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0113
U 1 1 5E335590
P 2750 4850
F 0 "#PWR0113" H 2750 4700 50  0001 C CNN
F 1 "+3.3V" H 2765 5023 50  0000 C CNN
F 2 "" H 2750 4850 50  0001 C CNN
F 3 "" H 2750 4850 50  0001 C CNN
	1    2750 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 4900 3100 4900
Wire Wire Line
	3100 4900 3100 5150
Wire Wire Line
	2750 4900 2750 4850
Connection ~ 2750 4900
Wire Wire Line
	2750 4900 2750 5050
Wire Wire Line
	2750 5250 2750 5450
Wire Wire Line
	2750 5450 2900 5450
Wire Wire Line
	3400 5250 3400 5450
Wire Wire Line
	3400 5450 3300 5450
$Comp
L Connector_Generic:Conn_01x03 PANEL2
U 1 1 5E3355AD
P 3950 5450
F 0 "PANEL2" H 4030 5492 50  0000 L CNN
F 1 "Conn_01x03" H 4030 5401 50  0000 L CNN
F 2 "Pad_SMD:Pad_SMD_01x03" H 3950 5450 50  0001 C CNN
F 3 "~" H 3950 5450 50  0001 C CNN
	1    3950 5450
	1    0    0    -1  
$EndComp
Connection ~ 2750 5450
Wire Wire Line
	3400 5050 3400 4850
$Comp
L teensy:Teensy4.0 U1
U 1 1 5E36AD29
P 5300 2050
F 0 "U1" V 5254 3528 50  0000 L CNN
F 1 "Teensy4.0" V 5345 3528 50  0000 L CNN
F 2 "teensy:Teensy40" H 4900 2250 50  0001 C CNN
F 3 "" H 4900 2250 50  0001 C CNN
	1    5300 2050
	0    -1   -1   0   
$EndComp
Text Label 5950 3150 3    50   ~ 0
DATA1
Text Label 6050 3150 3    50   ~ 0
DATA2
Text Label 2450 4250 2    50   ~ 0
DATA1
Text Label 2450 5450 2    50   ~ 0
DATA2
$Comp
L power:VDD #PWR0117
U 1 1 5E3A700D
P 3400 3650
F 0 "#PWR0117" H 3400 3500 50  0001 C CNN
F 1 "VDD" H 3417 3823 50  0000 C CNN
F 2 "" H 3400 3650 50  0001 C CNN
F 3 "" H 3400 3650 50  0001 C CNN
	1    3400 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 3850 3400 3650
$Comp
L Connector_Generic:Conn_01x03 PANEL1
U 1 1 5E319A24
P 3950 4250
F 0 "PANEL1" H 4030 4292 50  0000 L CNN
F 1 "Conn_01x03" H 4030 4201 50  0000 L CNN
F 2 "Pad_SMD:Pad_SMD_01x03" H 3950 4250 50  0001 C CNN
F 3 "~" H 3950 4250 50  0001 C CNN
	1    3950 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 4250 3400 4250
Wire Wire Line
	3400 4250 3300 4250
Connection ~ 3400 4250
Wire Wire Line
	3400 4050 3400 4250
$Comp
L Device:R_Small_US R2
U 1 1 5E290E89
P 3400 3950
F 0 "R2" H 3468 3996 50  0000 L CNN
F 1 "10K" H 3468 3905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3400 3950 50  0001 C CNN
F 3 "~" H 3400 3950 50  0001 C CNN
	1    3400 3950
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSS138 Q1
U 1 1 5E28BCEA
P 3100 4150
F 0 "Q1" V 3351 4150 50  0000 C CNN
F 1 "BSS138" V 3442 4150 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3300 4075 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 3100 4150 50  0001 L CNN
	1    3100 4150
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR0118
U 1 1 5E3A8EBA
P 3400 4850
F 0 "#PWR0118" H 3400 4700 50  0001 C CNN
F 1 "VDD" H 3417 5023 50  0000 C CNN
F 2 "" H 3400 4850 50  0001 C CNN
F 3 "" H 3400 4850 50  0001 C CNN
	1    3400 4850
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 5E3ADC97
P 1500 2300
F 0 "SW2" H 1500 2585 50  0000 C CNN
F 1 "SW_Push" H 1500 2494 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 1500 2500 50  0001 C CNN
F 3 "~" H 1500 2500 50  0001 C CNN
	1    1500 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5E3B1841
P 1300 2300
F 0 "#PWR0121" H 1300 2050 50  0001 C CNN
F 1 "GND" V 1305 2172 50  0000 R CNN
F 2 "" H 1300 2300 50  0001 C CNN
F 3 "" H 1300 2300 50  0001 C CNN
	1    1300 2300
	0    1    1    0   
$EndComp
Text Label 1700 2300 0    50   ~ 0
BUTTON
Text Label 4050 3150 3    50   ~ 0
BUTTON
$Comp
L power:GND #PWR0122
U 1 1 5E3B56F0
P 1650 1050
F 0 "#PWR0122" H 1650 800 50  0001 C CNN
F 1 "GND" V 1655 922 50  0000 R CNN
F 2 "" H 1650 1050 50  0001 C CNN
F 3 "" H 1650 1050 50  0001 C CNN
	1    1650 1050
	0    -1   -1   0   
$EndComp
Text Label 6150 3150 3    50   ~ 0
THUMBDIAL2
Text Label 1500 900  0    50   ~ 0
THUMBDIAL1
$Comp
L power:VDD #PWR0124
U 1 1 5E3BBE47
P 2450 1900
F 0 "#PWR0124" H 2450 1750 50  0001 C CNN
F 1 "VDD" H 2468 2073 50  0000 C CNN
F 2 "" H 2450 1900 50  0001 C CNN
F 3 "" H 2450 1900 50  0001 C CNN
	1    2450 1900
	-1   0    0    1   
$EndComp
NoConn ~ 2550 1900
$Comp
L power:GND #PWR0125
U 1 1 5E3BCF43
P 2650 1900
F 0 "#PWR0125" H 2650 1650 50  0001 C CNN
F 1 "GND" H 2655 1727 50  0000 C CNN
F 2 "" H 2650 1900 50  0001 C CNN
F 3 "" H 2650 1900 50  0001 C CNN
	1    2650 1900
	1    0    0    -1  
$EndComp
Text Label 2750 1900 3    50   ~ 0
MOTION_SDA
Text Label 2850 1900 3    50   ~ 0
MOTION_SCL
NoConn ~ 2950 1900
Text Label 6350 3150 3    50   ~ 0
MOTION_SDA
Text Label 6450 3150 3    50   ~ 0
MOTION_SCL
$Comp
L power:VDD #PWR02
U 1 1 5E3C0376
P 3750 4150
F 0 "#PWR02" H 3750 4000 50  0001 C CNN
F 1 "VDD" H 3767 4323 50  0000 C CNN
F 2 "" H 3750 4150 50  0001 C CNN
F 3 "" H 3750 4150 50  0001 C CNN
	1    3750 4150
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR03
U 1 1 5E3C0A47
P 3750 5350
F 0 "#PWR03" H 3750 5200 50  0001 C CNN
F 1 "VDD" H 3767 5523 50  0000 C CNN
F 2 "" H 3750 5350 50  0001 C CNN
F 3 "" H 3750 5350 50  0001 C CNN
	1    3750 5350
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR05
U 1 1 5E3C116C
P 6250 950
F 0 "#PWR05" H 6250 800 50  0001 C CNN
F 1 "VDD" H 6267 1123 50  0000 C CNN
F 2 "" H 6250 950 50  0001 C CNN
F 3 "" H 6250 950 50  0001 C CNN
	1    6250 950 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5E3C24CE
P 6350 950
F 0 "#PWR06" H 6350 700 50  0001 C CNN
F 1 "GND" H 6355 777 50  0000 C CNN
F 2 "" H 6350 950 50  0001 C CNN
F 3 "" H 6350 950 50  0001 C CNN
	1    6350 950 
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5E3C2C77
P 5550 3150
F 0 "#PWR04" H 5550 2900 50  0001 C CNN
F 1 "GND" H 5555 2977 50  0000 C CNN
F 2 "" H 5550 3150 50  0001 C CNN
F 3 "" H 5550 3150 50  0001 C CNN
	1    5550 3150
	1    0    0    -1  
$EndComp
NoConn ~ 1200 7000
NoConn ~ 1600 6800
NoConn ~ 1600 6700
NoConn ~ 1600 6600
NoConn ~ 1200 6100
NoConn ~ 1600 5900
NoConn ~ 1600 5800
NoConn ~ 1600 5700
NoConn ~ 1200 5200
NoConn ~ 1600 5000
NoConn ~ 1600 4900
NoConn ~ 1600 4800
NoConn ~ 1200 4300
NoConn ~ 1600 4100
NoConn ~ 1600 4000
NoConn ~ 1600 3900
$Comp
L power:GND #PWR01
U 1 1 5E3C845E
P 3950 3150
F 0 "#PWR01" H 3950 2900 50  0001 C CNN
F 1 "GND" H 3955 2977 50  0000 C CNN
F 2 "" H 3950 3150 50  0001 C CNN
F 3 "" H 3950 3150 50  0001 C CNN
	1    3950 3150
	1    0    0    -1  
$EndComp
NoConn ~ 3950 950 
NoConn ~ 4050 950 
NoConn ~ 4150 950 
NoConn ~ 4250 950 
NoConn ~ 4350 950 
NoConn ~ 4450 950 
NoConn ~ 4550 950 
NoConn ~ 4650 950 
NoConn ~ 4750 950 
NoConn ~ 4850 950 
NoConn ~ 4950 950 
NoConn ~ 5050 950 
NoConn ~ 5150 950 
NoConn ~ 5250 950 
NoConn ~ 5350 950 
NoConn ~ 5450 950 
NoConn ~ 5550 950 
NoConn ~ 5650 950 
NoConn ~ 5750 950 
NoConn ~ 5850 950 
NoConn ~ 6150 950 
NoConn ~ 6250 3150
NoConn ~ 5850 3150
NoConn ~ 6550 950 
NoConn ~ 5050 3150
NoConn ~ 5750 3150
NoConn ~ 5650 3150
NoConn ~ 5350 3150
NoConn ~ 5150 3150
NoConn ~ 4950 3150
NoConn ~ 4750 3150
NoConn ~ 4650 3150
NoConn ~ 4550 3150
NoConn ~ 4450 3150
NoConn ~ 4350 3150
$Comp
L power:VBUS #PWR0101
U 1 1 5E3DB32C
P 1600 3700
F 0 "#PWR0101" H 1600 3550 50  0001 C CNN
F 1 "VBUS" V 1615 3828 50  0000 L CNN
F 2 "" H 1600 3700 50  0001 C CNN
F 3 "" H 1600 3700 50  0001 C CNN
	1    1600 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	1300 4300 1600 4300
$Comp
L power:GND #PWR0102
U 1 1 5E3DF1EA
P 1600 5200
F 0 "#PWR0102" H 1600 4950 50  0001 C CNN
F 1 "GND" V 1605 5072 50  0000 R CNN
F 2 "" H 1600 5200 50  0001 C CNN
F 3 "" H 1600 5200 50  0001 C CNN
	1    1600 5200
	0    -1   -1   0   
$EndComp
$Comp
L power:VBUS #PWR0103
U 1 1 5E3DF1F0
P 1600 4600
F 0 "#PWR0103" H 1600 4450 50  0001 C CNN
F 1 "VBUS" V 1615 4728 50  0000 L CNN
F 2 "" H 1600 4600 50  0001 C CNN
F 3 "" H 1600 4600 50  0001 C CNN
	1    1600 4600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5E3E077D
P 1600 6100
F 0 "#PWR0106" H 1600 5850 50  0001 C CNN
F 1 "GND" V 1605 5972 50  0000 R CNN
F 2 "" H 1600 6100 50  0001 C CNN
F 3 "" H 1600 6100 50  0001 C CNN
	1    1600 6100
	0    -1   -1   0   
$EndComp
$Comp
L power:VBUS #PWR0107
U 1 1 5E3E0783
P 1600 5500
F 0 "#PWR0107" H 1600 5350 50  0001 C CNN
F 1 "VBUS" V 1615 5628 50  0000 L CNN
F 2 "" H 1600 5500 50  0001 C CNN
F 3 "" H 1600 5500 50  0001 C CNN
	1    1600 5500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5E3E1056
P 1600 7000
F 0 "#PWR0108" H 1600 6750 50  0001 C CNN
F 1 "GND" V 1605 6872 50  0000 R CNN
F 2 "" H 1600 7000 50  0001 C CNN
F 3 "" H 1600 7000 50  0001 C CNN
	1    1600 7000
	0    -1   -1   0   
$EndComp
$Comp
L power:VBUS #PWR0109
U 1 1 5E3E105C
P 1600 6400
F 0 "#PWR0109" H 1600 6250 50  0001 C CNN
F 1 "VBUS" V 1615 6528 50  0000 L CNN
F 2 "" H 1600 6400 50  0001 C CNN
F 3 "" H 1600 6400 50  0001 C CNN
	1    1600 6400
	0    1    1    0   
$EndComp
Wire Wire Line
	1300 7000 1600 7000
Wire Wire Line
	1300 6100 1600 6100
Wire Wire Line
	1300 5200 1600 5200
$Comp
L power:+3.3V #PWR0115
U 1 1 5E3E4A94
P 5450 3150
F 0 "#PWR0115" H 5450 3000 50  0001 C CNN
F 1 "+3.3V" H 5465 3323 50  0000 C CNN
F 2 "" H 5450 3150 50  0001 C CNN
F 3 "" H 5450 3150 50  0001 C CNN
	1    5450 3150
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5E313BD9
P 2850 6650
F 0 "#FLG0101" H 2850 6725 50  0001 C CNN
F 1 "PWR_FLAG" V 2850 6950 50  0000 C CNN
F 2 "" H 2850 6650 50  0001 C CNN
F 3 "~" H 2850 6650 50  0001 C CNN
	1    2850 6650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5E31610B
P 2800 6650
F 0 "#PWR0119" H 2800 6400 50  0001 C CNN
F 1 "GND" V 2800 6450 50  0000 C CNN
F 2 "" H 2800 6650 50  0001 C CNN
F 3 "" H 2800 6650 50  0001 C CNN
	1    2800 6650
	0    1    1    0   
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5E3171F3
P 2850 6550
F 0 "#FLG0102" H 2850 6625 50  0001 C CNN
F 1 "PWR_FLAG" V 2850 6850 50  0000 C CNN
F 2 "" H 2850 6550 50  0001 C CNN
F 3 "~" H 2850 6550 50  0001 C CNN
	1    2850 6550
	0    1    1    0   
$EndComp
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5E31936A
P 2850 6450
F 0 "#FLG0103" H 2850 6525 50  0001 C CNN
F 1 "PWR_FLAG" V 2850 6750 50  0000 C CNN
F 2 "" H 2850 6450 50  0001 C CNN
F 3 "~" H 2850 6450 50  0001 C CNN
	1    2850 6450
	0    1    1    0   
$EndComp
$Comp
L power:VBUS #PWR0126
U 1 1 5E319FB1
P 2800 6550
F 0 "#PWR0126" H 2800 6400 50  0001 C CNN
F 1 "VBUS" V 2800 6750 50  0000 C CNN
F 2 "" H 2800 6550 50  0001 C CNN
F 3 "" H 2800 6550 50  0001 C CNN
	1    2800 6550
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0127
U 1 1 5E31AA77
P 2800 6450
F 0 "#PWR0127" H 2800 6300 50  0001 C CNN
F 1 "+3.3V" V 2800 6650 50  0000 C CNN
F 2 "" H 2800 6450 50  0001 C CNN
F 3 "" H 2800 6450 50  0001 C CNN
	1    2800 6450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2850 6650 2800 6650
Wire Wire Line
	2800 6550 2850 6550
Wire Wire Line
	2850 6450 2800 6450
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5E32837F
P 2850 6350
F 0 "#FLG0104" H 2850 6425 50  0001 C CNN
F 1 "PWR_FLAG" V 2850 6650 50  0000 C CNN
F 2 "" H 2850 6350 50  0001 C CNN
F 3 "~" H 2850 6350 50  0001 C CNN
	1    2850 6350
	0    1    1    0   
$EndComp
Wire Wire Line
	2850 6350 2800 6350
$Comp
L power:VDD #PWR0128
U 1 1 5E3290A6
P 2800 6350
F 0 "#PWR0128" H 2800 6200 50  0001 C CNN
F 1 "VDD" V 2817 6478 50  0000 L CNN
F 2 "" H 2800 6350 50  0001 C CNN
F 3 "" H 2800 6350 50  0001 C CNN
	1    2800 6350
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 PANEL_PWR1
U 1 1 5E37B90D
P 4250 4750
F 0 "PANEL_PWR1" H 4330 4742 50  0000 L CNN
F 1 "Conn_01x02" H 4330 4651 50  0000 L CNN
F 2 "Pad_SMD:Pad_SMD_01x02" H 4250 4750 50  0001 C CNN
F 3 "~" H 4250 4750 50  0001 C CNN
	1    4250 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 5450 3750 5450
Connection ~ 3400 5450
$Comp
L power:VDD #PWR0130
U 1 1 5E37EDC1
P 4050 4850
F 0 "#PWR0130" H 4050 4700 50  0001 C CNN
F 1 "VDD" V 4068 4977 50  0000 L CNN
F 2 "" H 4050 4850 50  0001 C CNN
F 3 "" H 4050 4850 50  0001 C CNN
	1    4050 4850
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 PANEL_PWR2
U 1 1 5E3831A4
P 4250 5950
F 0 "PANEL_PWR2" H 4330 5942 50  0000 L CNN
F 1 "Conn_01x02" H 4330 5851 50  0000 L CNN
F 2 "Pad_SMD:Pad_SMD_01x02" H 4250 5950 50  0001 C CNN
F 3 "~" H 4250 5950 50  0001 C CNN
	1    4250 5950
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0132
U 1 1 5E3831B0
P 4050 6050
F 0 "#PWR0132" H 4050 5900 50  0001 C CNN
F 1 "VDD" V 4068 6177 50  0000 L CNN
F 2 "" H 4050 6050 50  0001 C CNN
F 3 "" H 4050 6050 50  0001 C CNN
	1    4050 6050
	0    -1   -1   0   
$EndComp
$Comp
L Breakoutboards:Adafruit_BNO055 U2
U 1 1 5E3B751D
P 2750 1700
F 0 "U2" H 3078 1896 50  0000 L CNN
F 1 "Adafruit_BNO055" H 3078 1805 50  0000 L CNN
F 2 "BreakoutBoards:Adafruit_BNO055" H 3300 2200 50  0001 C CNN
F 3 "" H 3300 2200 50  0001 C CNN
	1    2750 1700
	1    0    0    -1  
$EndComp
NoConn ~ 2550 1200
NoConn ~ 2650 1200
NoConn ~ 2750 1200
NoConn ~ 2850 1200
Wire Wire Line
	2450 5450 2750 5450
$Comp
L power:VDD #PWR0120
U 1 1 5E3ACFA6
P 1700 2850
F 0 "#PWR0120" H 1700 2700 50  0001 C CNN
F 1 "VDD" V 1717 2978 50  0000 L CNN
F 2 "" H 1700 2850 50  0001 C CNN
F 3 "" H 1700 2850 50  0001 C CNN
	1    1700 2850
	0    1    1    0   
$EndComp
$Comp
L power:VBUS #PWR0111
U 1 1 5E3E2BC8
P 1300 2750
F 0 "#PWR0111" H 1300 2600 50  0001 C CNN
F 1 "VBUS" V 1315 2878 50  0000 L CNN
F 2 "" H 1300 2750 50  0001 C CNN
F 3 "" H 1300 2750 50  0001 C CNN
	1    1300 2750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5E49DFB8
P 3750 4350
F 0 "#PWR0112" H 3750 4100 50  0001 C CNN
F 1 "GND" H 3755 4177 50  0000 C CNN
F 2 "" H 3750 4350 50  0001 C CNN
F 3 "" H 3750 4350 50  0001 C CNN
	1    3750 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5E49E6CA
P 3750 5550
F 0 "#PWR0116" H 3750 5300 50  0001 C CNN
F 1 "GND" H 3755 5377 50  0000 C CNN
F 2 "" H 3750 5550 50  0001 C CNN
F 3 "" H 3750 5550 50  0001 C CNN
	1    3750 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 5E49EE23
P 4050 5950
F 0 "#PWR0129" H 4050 5700 50  0001 C CNN
F 1 "GND" V 4055 5822 50  0000 R CNN
F 2 "" H 4050 5950 50  0001 C CNN
F 3 "" H 4050 5950 50  0001 C CNN
	1    4050 5950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 5E4A05FE
P 4050 4750
F 0 "#PWR0131" H 4050 4500 50  0001 C CNN
F 1 "GND" V 4055 4622 50  0000 R CNN
F 2 "" H 4050 4750 50  0001 C CNN
F 3 "" H 4050 4750 50  0001 C CNN
	1    4050 4750
	0    1    1    0   
$EndComp
NoConn ~ 4250 3150
NoConn ~ 1700 2650
$Comp
L Device:R_POT P2
U 1 1 5E3E27DC
P 1500 1600
F 0 "P2" V 1300 1600 50  0000 C CNN
F 1 "10K" V 1400 1600 50  0000 C CNN
F 2 "Potentiometer_THT:PDB12-H" H 1500 1600 50  0001 L BNN
F 3 "" H 1500 1600 50  0001 L BNN
F 4 "IPC-7351B" H 1500 1600 50  0001 L BNN "Field4"
F 5 "08/19" H 1500 1600 50  0001 L BNN "Field5"
F 6 "Bourns" H 1500 1600 50  0001 L BNN "Field6"
	1    1500 1600
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 5E3E27E2
P 1650 1600
F 0 "#PWR0133" H 1650 1350 50  0001 C CNN
F 1 "GND" V 1655 1472 50  0000 R CNN
F 2 "" H 1650 1600 50  0001 C CNN
F 3 "" H 1650 1600 50  0001 C CNN
	1    1650 1600
	0    -1   -1   0   
$EndComp
Text Label 1500 1450 0    50   ~ 0
THUMBDIAL2
Text Label 6650 950  1    50   ~ 0
THUMBDIAL1
$Comp
L power:+3.3VA #PWR0135
U 1 1 5E3FAA51
P 6450 950
F 0 "#PWR0135" H 6450 800 50  0001 C CNN
F 1 "+3.3VA" H 6465 1123 50  0000 C CNN
F 2 "" H 6450 950 50  0001 C CNN
F 3 "" H 6450 950 50  0001 C CNN
	1    6450 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0110
U 1 1 5E3FC6B5
P 1350 1050
F 0 "#PWR0110" H 1350 900 50  0001 C CNN
F 1 "+3.3VA" V 1350 1150 50  0000 L CNN
F 2 "" H 1350 1050 50  0001 C CNN
F 3 "" H 1350 1050 50  0001 C CNN
	1    1350 1050
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3VA #PWR0114
U 1 1 5E401221
P 1350 1600
F 0 "#PWR0114" H 1350 1450 50  0001 C CNN
F 1 "+3.3VA" V 1350 1700 50  0000 L CNN
F 2 "" H 1350 1600 50  0001 C CNN
F 3 "" H 1350 1600 50  0001 C CNN
	1    1350 1600
	0    -1   -1   0   
$EndComp
$Comp
L Mic_I2S_SPH0645LM4H-B:SPH0645LM4H-B MK1
U 1 1 5E404A86
P 6200 5400
F 0 "MK1" H 6200 5867 50  0000 C CNN
F 1 "SPH0645LM4H-B" H 6200 5776 50  0000 C CNN
F 2 "Mic-SPH0645LM4H-B:MIC_SPH0645LM4H-B" H 6200 5400 50  0001 L BNN
F 3 "" H 6200 5400 50  0001 L BNN
F 4 "Mic Mems Digital I2s Omni -26db" H 6200 5400 50  0001 L BNN "Field4"
F 5 "SPH0645LM4H-B" H 6200 5400 50  0001 L BNN "Field5"
F 6 "SIP-7 Bourns" H 6200 5400 50  0001 L BNN "Field6"
F 7 "None" H 6200 5400 50  0001 L BNN "Field7"
F 8 "Unavailable" H 6200 5400 50  0001 L BNN "Field8"
	1    6200 5400
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R6
U 1 1 5E4063B4
P 6200 4500
F 0 "R6" V 5993 4500 50  0000 C CNN
F 1 "68" V 6084 4500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6130 4500 50  0001 C CNN
F 3 "~" H 6200 4500 50  0001 C CNN
	1    6200 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5E407182
P 6200 6350
F 0 "R5" V 5993 6350 50  0000 C CNN
F 1 "10K" V 6084 6350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6130 6350 50  0001 C CNN
F 3 "~" H 6200 6350 50  0001 C CNN
	1    6200 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 6100 6200 6200
Text Label 6300 6100 0    50   ~ 0
MIC_BCLK
Text Label 6100 6100 2    50   ~ 0
MIC_LRCLK
$Comp
L Device:C C1
U 1 1 5E40B74B
P 5650 4650
F 0 "C1" H 5765 4696 50  0000 L CNN
F 1 "1uF" H 5765 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5688 4500 50  0001 C CNN
F 3 "~" H 5650 4650 50  0001 C CNN
	1    5650 4650
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C2
U 1 1 5E40C27D
P 5650 4300
F 0 "C2" H 5765 4346 50  0000 L CNN
F 1 "220pF" H 5765 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5688 4150 50  0001 C CNN
F 3 "~" H 5650 4300 50  0001 C CNN
	1    5650 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6000 4650 5800 4650
Wire Wire Line
	5800 4300 5800 4650
Connection ~ 5800 4650
$Comp
L power:GND #PWR0123
U 1 1 5E40E9D7
P 6400 4700
F 0 "#PWR0123" H 6400 4450 50  0001 C CNN
F 1 "GND" H 6405 4527 50  0000 C CNN
F 2 "" H 6400 4700 50  0001 C CNN
F 3 "" H 6400 4700 50  0001 C CNN
	1    6400 4700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5500 4650 5500 4300
$Comp
L power:GND #PWR0134
U 1 1 5E410E8E
P 5400 4650
F 0 "#PWR0134" H 5400 4400 50  0001 C CNN
F 1 "GND" H 5405 4477 50  0000 C CNN
F 2 "" H 5400 4650 50  0001 C CNN
F 3 "" H 5400 4650 50  0001 C CNN
	1    5400 4650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0136
U 1 1 5E411765
P 6200 6500
F 0 "#PWR0136" H 6200 6250 50  0001 C CNN
F 1 "GND" V 6205 6372 50  0000 R CNN
F 2 "" H 6200 6500 50  0001 C CNN
F 3 "" H 6200 6500 50  0001 C CNN
	1    6200 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 4650 5400 4650
Connection ~ 5500 4650
NoConn ~ 5250 3150
Text Label 6550 3150 3    50   ~ 0
MIC_LRCLK
Wire Wire Line
	6000 4700 6000 4650
Wire Wire Line
	6200 4700 6200 4650
Text Label 6200 4350 1    50   ~ 0
MIC_DATA_IN
Text Label 4850 3150 3    50   ~ 0
MIC_DATA_IN
Text Label 6650 3150 3    50   ~ 0
MIC_BCLK
NoConn ~ 4150 3150
$Comp
L power:+3.3VA #PWR0137
U 1 1 5E4C39D5
P 5800 4300
F 0 "#PWR0137" H 5800 4150 50  0001 C CNN
F 1 "+3.3VA" V 5815 4428 50  0000 L CNN
F 2 "" H 5800 4300 50  0001 C CNN
F 3 "" H 5800 4300 50  0001 C CNN
	1    5800 4300
	0    1    1    0   
$EndComp
Connection ~ 5800 4300
$Comp
L Switch:SW_SPDT SW1
U 1 1 5E40F1C4
P 1500 2750
F 0 "SW1" H 1500 3035 50  0000 C CNN
F 1 "SW_SPDT" H 1500 2944 50  0000 C CNN
F 2 "Extra SMD Components:Button_Switch_SPDT_GH36P010001" H 1500 2750 50  0001 C CNN
F 3 "~" H 1500 2750 50  0001 C CNN
	1    1500 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT P1
U 1 1 5E3B3B17
P 1500 1050
F 0 "P1" V 1300 1050 50  0000 C CNN
F 1 "10K" V 1400 1050 50  0000 C CNN
F 2 "Potentiometer_THT:PDB12-H" H 1500 1050 50  0001 L BNN
F 3 "" H 1500 1050 50  0001 L BNN
F 4 "IPC-7351B" H 1500 1050 50  0001 L BNN "Field4"
F 5 "08/19" H 1500 1050 50  0001 L BNN "Field5"
F 6 "Bourns" H 1500 1050 50  0001 L BNN "Field6"
	1    1500 1050
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
