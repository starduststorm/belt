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
P 1400 4800
F 0 "J2" H 1457 5267 50  0000 C CNN
F 1 "USB_B_Micro" H 1457 5176 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1550 4750 50  0001 C CNN
F 3 "~" H 1550 4750 50  0001 C CNN
	1    1400 4800
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J4
U 1 1 5E214297
P 1400 6600
F 0 "J4" H 1457 7067 50  0000 C CNN
F 1 "USB_B_Micro" H 1457 6976 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1550 6550 50  0001 C CNN
F 3 "~" H 1550 6550 50  0001 C CNN
	1    1400 6600
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J3
U 1 1 5E213AB5
P 1400 5700
F 0 "J3" H 1457 6167 50  0000 C CNN
F 1 "USB_B_Micro" H 1457 6076 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1550 5650 50  0001 C CNN
F 3 "~" H 1550 5650 50  0001 C CNN
	1    1400 5700
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J1
U 1 1 5E224B3A
P 1400 3900
F 0 "J1" H 1457 4367 50  0000 C CNN
F 1 "USB_B_Micro" H 1457 4276 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1550 3850 50  0001 C CNN
F 3 "~" H 1550 3850 50  0001 C CNN
	1    1400 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5E224B41
P 1700 4300
F 0 "#PWR0104" H 1700 4050 50  0001 C CNN
F 1 "GND" V 1705 4172 50  0000 R CNN
F 2 "" H 1700 4300 50  0001 C CNN
F 3 "" H 1700 4300 50  0001 C CNN
	1    1700 4300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 5E28EBE0
P 3000 4100
F 0 "R1" H 3068 4146 50  0000 L CNN
F 1 "10K" H 3068 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 3000 4100 50  0001 C CNN
F 3 "~" H 3000 4100 50  0001 C CNN
	1    3000 4100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0105
U 1 1 5E291789
P 3000 3800
F 0 "#PWR0105" H 3000 3650 50  0001 C CNN
F 1 "+3.3V" H 3015 3973 50  0000 C CNN
F 2 "" H 3000 3800 50  0001 C CNN
F 3 "" H 3000 3800 50  0001 C CNN
	1    3000 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 3850 3350 3850
Wire Wire Line
	3350 3850 3350 4100
Wire Wire Line
	3000 3850 3000 3800
Connection ~ 3000 3850
Wire Wire Line
	3000 3850 3000 4000
Wire Wire Line
	3000 4200 3000 4400
Wire Wire Line
	3000 4400 3150 4400
Connection ~ 3000 4400
Wire Wire Line
	2700 4400 3000 4400
$Comp
L Transistor_FET:BSS138 Q2
U 1 1 5E33557E
P 3350 5500
F 0 "Q2" V 3601 5500 50  0000 C CNN
F 1 "BSS138" V 3692 5500 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3550 5425 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 3350 5500 50  0001 L CNN
	1    3350 5500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R3
U 1 1 5E335584
P 3000 5300
F 0 "R3" H 3068 5346 50  0000 L CNN
F 1 "10K" H 3068 5255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 3000 5300 50  0001 C CNN
F 3 "~" H 3000 5300 50  0001 C CNN
	1    3000 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R4
U 1 1 5E33558A
P 3650 5300
F 0 "R4" H 3718 5346 50  0000 L CNN
F 1 "10K" H 3718 5255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 3650 5300 50  0001 C CNN
F 3 "~" H 3650 5300 50  0001 C CNN
	1    3650 5300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0113
U 1 1 5E335590
P 3000 5000
F 0 "#PWR0113" H 3000 4850 50  0001 C CNN
F 1 "+3.3V" H 3015 5173 50  0000 C CNN
F 2 "" H 3000 5000 50  0001 C CNN
F 3 "" H 3000 5000 50  0001 C CNN
	1    3000 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 5050 3350 5050
Wire Wire Line
	3350 5050 3350 5300
Wire Wire Line
	3000 5050 3000 5000
Connection ~ 3000 5050
Wire Wire Line
	3000 5050 3000 5200
Wire Wire Line
	3000 5400 3000 5600
Wire Wire Line
	3000 5600 3150 5600
Wire Wire Line
	3650 5400 3650 5600
Wire Wire Line
	3650 5600 3550 5600
$Comp
L Connector_Generic:Conn_01x03 PANEL2
U 1 1 5E3355AD
P 4200 5600
F 0 "PANEL2" H 4280 5642 50  0000 L CNN
F 1 "Conn_01x03" H 4280 5551 50  0000 L CNN
F 2 "Pad_SMD:Pad_SMD_01x03" H 4200 5600 50  0001 C CNN
F 3 "~" H 4200 5600 50  0001 C CNN
	1    4200 5600
	1    0    0    -1  
$EndComp
Connection ~ 3000 5600
Wire Wire Line
	3650 5200 3650 5000
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
Text Label 2700 4400 2    50   ~ 0
DATA1
Text Label 2700 5600 2    50   ~ 0
DATA2
$Comp
L power:VDD #PWR0117
U 1 1 5E3A700D
P 3650 3800
F 0 "#PWR0117" H 3650 3650 50  0001 C CNN
F 1 "VDD" H 3667 3973 50  0000 C CNN
F 2 "" H 3650 3800 50  0001 C CNN
F 3 "" H 3650 3800 50  0001 C CNN
	1    3650 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 4000 3650 3800
$Comp
L Connector_Generic:Conn_01x03 PANEL1
U 1 1 5E319A24
P 4200 4400
F 0 "PANEL1" H 4280 4442 50  0000 L CNN
F 1 "Conn_01x03" H 4280 4351 50  0000 L CNN
F 2 "Pad_SMD:Pad_SMD_01x03" H 4200 4400 50  0001 C CNN
F 3 "~" H 4200 4400 50  0001 C CNN
	1    4200 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 4400 3650 4400
Wire Wire Line
	3650 4400 3550 4400
Connection ~ 3650 4400
Wire Wire Line
	3650 4200 3650 4400
$Comp
L Device:R_Small_US R2
U 1 1 5E290E89
P 3650 4100
F 0 "R2" H 3718 4146 50  0000 L CNN
F 1 "10K" H 3718 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 3650 4100 50  0001 C CNN
F 3 "~" H 3650 4100 50  0001 C CNN
	1    3650 4100
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSS138 Q1
U 1 1 5E28BCEA
P 3350 4300
F 0 "Q1" V 3601 4300 50  0000 C CNN
F 1 "BSS138" V 3692 4300 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3550 4225 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 3350 4300 50  0001 L CNN
	1    3350 4300
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR0118
U 1 1 5E3A8EBA
P 3650 5000
F 0 "#PWR0118" H 3650 4850 50  0001 C CNN
F 1 "VDD" H 3667 5173 50  0000 C CNN
F 2 "" H 3650 5000 50  0001 C CNN
F 3 "" H 3650 5000 50  0001 C CNN
	1    3650 5000
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_DPDT_x2 SW1
U 1 1 5E3A9D22
P 1500 2750
F 0 "SW1" H 1500 2985 50  0000 C CNN
F 1 "SW_DPDT" H 1500 2894 50  0000 C CNN
F 2 "engstad:slide-switch-MFS201N-9-Z" H 1500 2750 50  0001 C CNN
F 3 "~" H 1500 2750 50  0001 C CNN
	1    1500 2750
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 5E3ADC97
P 1500 2300
F 0 "SW2" H 1500 2585 50  0000 C CNN
F 1 "SW_Push" H 1500 2494 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 1500 2500 50  0001 C CNN
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
L belt-rescue:3352T-1-103LF-3352T-1-103LF P1
U 1 1 5E3B3B17
P 1500 1050
F 0 "P1" H 1500 1165 50  0000 C CNN
F 1 "3352T-1-103LF" H 1500 1256 50  0000 C CNN
F 2 "Potentiometer_THT:PDB12-H" H 1500 1050 50  0001 L BNN
F 3 "" H 1500 1050 50  0001 L BNN
F 4 "IPC-7351B" H 1500 1050 50  0001 L BNN "Field4"
F 5 "08/19" H 1500 1050 50  0001 L BNN "Field5"
F 6 "Bourns" H 1500 1050 50  0001 L BNN "Field6"
	1    1500 1050
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 5E3B56F0
P 1200 1050
F 0 "#PWR0122" H 1200 800 50  0001 C CNN
F 1 "GND" V 1205 922 50  0000 R CNN
F 2 "" H 1200 1050 50  0001 C CNN
F 3 "" H 1200 1050 50  0001 C CNN
	1    1200 1050
	0    1    1    0   
$EndComp
Text Label 6150 3150 3    50   ~ 0
THUMBDIAL2
Text Label 1500 850  0    50   ~ 0
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
P 4000 4300
F 0 "#PWR02" H 4000 4150 50  0001 C CNN
F 1 "VDD" H 4017 4473 50  0000 C CNN
F 2 "" H 4000 4300 50  0001 C CNN
F 3 "" H 4000 4300 50  0001 C CNN
	1    4000 4300
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR03
U 1 1 5E3C0A47
P 4000 5500
F 0 "#PWR03" H 4000 5350 50  0001 C CNN
F 1 "VDD" H 4017 5673 50  0000 C CNN
F 2 "" H 4000 5500 50  0001 C CNN
F 3 "" H 4000 5500 50  0001 C CNN
	1    4000 5500
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
NoConn ~ 1300 7000
NoConn ~ 1700 6800
NoConn ~ 1700 6700
NoConn ~ 1700 6600
NoConn ~ 1300 6100
NoConn ~ 1700 5900
NoConn ~ 1700 5800
NoConn ~ 1700 5700
NoConn ~ 1300 5200
NoConn ~ 1700 5000
NoConn ~ 1700 4900
NoConn ~ 1700 4800
NoConn ~ 1300 4300
NoConn ~ 1700 4100
NoConn ~ 1700 4000
NoConn ~ 1700 3900
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
P 1700 3700
F 0 "#PWR0101" H 1700 3550 50  0001 C CNN
F 1 "VBUS" V 1715 3828 50  0000 L CNN
F 2 "" H 1700 3700 50  0001 C CNN
F 3 "" H 1700 3700 50  0001 C CNN
	1    1700 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	1400 4300 1700 4300
$Comp
L power:GND #PWR0102
U 1 1 5E3DF1EA
P 1700 5200
F 0 "#PWR0102" H 1700 4950 50  0001 C CNN
F 1 "GND" V 1705 5072 50  0000 R CNN
F 2 "" H 1700 5200 50  0001 C CNN
F 3 "" H 1700 5200 50  0001 C CNN
	1    1700 5200
	0    -1   -1   0   
$EndComp
$Comp
L power:VBUS #PWR0103
U 1 1 5E3DF1F0
P 1700 4600
F 0 "#PWR0103" H 1700 4450 50  0001 C CNN
F 1 "VBUS" V 1715 4728 50  0000 L CNN
F 2 "" H 1700 4600 50  0001 C CNN
F 3 "" H 1700 4600 50  0001 C CNN
	1    1700 4600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5E3E077D
P 1700 6100
F 0 "#PWR0106" H 1700 5850 50  0001 C CNN
F 1 "GND" V 1705 5972 50  0000 R CNN
F 2 "" H 1700 6100 50  0001 C CNN
F 3 "" H 1700 6100 50  0001 C CNN
	1    1700 6100
	0    -1   -1   0   
$EndComp
$Comp
L power:VBUS #PWR0107
U 1 1 5E3E0783
P 1700 5500
F 0 "#PWR0107" H 1700 5350 50  0001 C CNN
F 1 "VBUS" V 1715 5628 50  0000 L CNN
F 2 "" H 1700 5500 50  0001 C CNN
F 3 "" H 1700 5500 50  0001 C CNN
	1    1700 5500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5E3E1056
P 1700 7000
F 0 "#PWR0108" H 1700 6750 50  0001 C CNN
F 1 "GND" V 1705 6872 50  0000 R CNN
F 2 "" H 1700 7000 50  0001 C CNN
F 3 "" H 1700 7000 50  0001 C CNN
	1    1700 7000
	0    -1   -1   0   
$EndComp
$Comp
L power:VBUS #PWR0109
U 1 1 5E3E105C
P 1700 6400
F 0 "#PWR0109" H 1700 6250 50  0001 C CNN
F 1 "VBUS" V 1715 6528 50  0000 L CNN
F 2 "" H 1700 6400 50  0001 C CNN
F 3 "" H 1700 6400 50  0001 C CNN
	1    1700 6400
	0    1    1    0   
$EndComp
Wire Wire Line
	1400 7000 1700 7000
Wire Wire Line
	1400 6100 1700 6100
Wire Wire Line
	1400 5200 1700 5200
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
P 4650 7050
F 0 "#FLG0101" H 4650 7125 50  0001 C CNN
F 1 "PWR_FLAG" V 4650 7350 50  0000 C CNN
F 2 "" H 4650 7050 50  0001 C CNN
F 3 "~" H 4650 7050 50  0001 C CNN
	1    4650 7050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5E31610B
P 4600 7050
F 0 "#PWR0119" H 4600 6800 50  0001 C CNN
F 1 "GND" V 4600 6850 50  0000 C CNN
F 2 "" H 4600 7050 50  0001 C CNN
F 3 "" H 4600 7050 50  0001 C CNN
	1    4600 7050
	0    1    1    0   
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5E3171F3
P 4650 6950
F 0 "#FLG0102" H 4650 7025 50  0001 C CNN
F 1 "PWR_FLAG" V 4650 7250 50  0000 C CNN
F 2 "" H 4650 6950 50  0001 C CNN
F 3 "~" H 4650 6950 50  0001 C CNN
	1    4650 6950
	0    1    1    0   
$EndComp
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5E31936A
P 4650 6850
F 0 "#FLG0103" H 4650 6925 50  0001 C CNN
F 1 "PWR_FLAG" V 4650 7150 50  0000 C CNN
F 2 "" H 4650 6850 50  0001 C CNN
F 3 "~" H 4650 6850 50  0001 C CNN
	1    4650 6850
	0    1    1    0   
$EndComp
$Comp
L power:VBUS #PWR0126
U 1 1 5E319FB1
P 4600 6950
F 0 "#PWR0126" H 4600 6800 50  0001 C CNN
F 1 "VBUS" V 4600 7150 50  0000 C CNN
F 2 "" H 4600 6950 50  0001 C CNN
F 3 "" H 4600 6950 50  0001 C CNN
	1    4600 6950
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0127
U 1 1 5E31AA77
P 4600 6850
F 0 "#PWR0127" H 4600 6700 50  0001 C CNN
F 1 "+3.3V" V 4600 7050 50  0000 C CNN
F 2 "" H 4600 6850 50  0001 C CNN
F 3 "" H 4600 6850 50  0001 C CNN
	1    4600 6850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4650 7050 4600 7050
Wire Wire Line
	4600 6950 4650 6950
Wire Wire Line
	4650 6850 4600 6850
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5E32837F
P 4650 6750
F 0 "#FLG0104" H 4650 6825 50  0001 C CNN
F 1 "PWR_FLAG" V 4650 7050 50  0000 C CNN
F 2 "" H 4650 6750 50  0001 C CNN
F 3 "~" H 4650 6750 50  0001 C CNN
	1    4650 6750
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 6750 4600 6750
$Comp
L power:VDD #PWR0128
U 1 1 5E3290A6
P 4600 6750
F 0 "#PWR0128" H 4600 6600 50  0001 C CNN
F 1 "VDD" V 4617 6878 50  0000 L CNN
F 2 "" H 4600 6750 50  0001 C CNN
F 3 "" H 4600 6750 50  0001 C CNN
	1    4600 6750
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 PANEL_PWR1
U 1 1 5E37B90D
P 4500 4900
F 0 "PANEL_PWR1" H 4580 4892 50  0000 L CNN
F 1 "Conn_01x02" H 4580 4801 50  0000 L CNN
F 2 "Pad_SMD:Pad_SMD_01x02" H 4500 4900 50  0001 C CNN
F 3 "~" H 4500 4900 50  0001 C CNN
	1    4500 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 5600 4000 5600
Connection ~ 3650 5600
$Comp
L power:VDD #PWR0130
U 1 1 5E37EDC1
P 4300 5000
F 0 "#PWR0130" H 4300 4850 50  0001 C CNN
F 1 "VDD" V 4318 5127 50  0000 L CNN
F 2 "" H 4300 5000 50  0001 C CNN
F 3 "" H 4300 5000 50  0001 C CNN
	1    4300 5000
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 PANEL_PWR2
U 1 1 5E3831A4
P 4500 6100
F 0 "PANEL_PWR2" H 4580 6092 50  0000 L CNN
F 1 "Conn_01x02" H 4580 6001 50  0000 L CNN
F 2 "Pad_SMD:Pad_SMD_01x02" H 4500 6100 50  0001 C CNN
F 3 "~" H 4500 6100 50  0001 C CNN
	1    4500 6100
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0132
U 1 1 5E3831B0
P 4300 6200
F 0 "#PWR0132" H 4300 6050 50  0001 C CNN
F 1 "VDD" V 4318 6327 50  0000 L CNN
F 2 "" H 4300 6200 50  0001 C CNN
F 3 "" H 4300 6200 50  0001 C CNN
	1    4300 6200
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
	2700 5600 3000 5600
$Comp
L power:VDD #PWR0120
U 1 1 5E3ACFA6
P 1300 2750
F 0 "#PWR0120" H 1300 2600 50  0001 C CNN
F 1 "VDD" V 1317 2878 50  0000 L CNN
F 2 "" H 1300 2750 50  0001 C CNN
F 3 "" H 1300 2750 50  0001 C CNN
	1    1300 2750
	0    -1   -1   0   
$EndComp
$Comp
L power:VBUS #PWR0111
U 1 1 5E3E2BC8
P 1700 2650
F 0 "#PWR0111" H 1700 2500 50  0001 C CNN
F 1 "VBUS" V 1715 2778 50  0000 L CNN
F 2 "" H 1700 2650 50  0001 C CNN
F 3 "" H 1700 2650 50  0001 C CNN
	1    1700 2650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5E49DFB8
P 4000 4500
F 0 "#PWR0112" H 4000 4250 50  0001 C CNN
F 1 "GND" H 4005 4327 50  0000 C CNN
F 2 "" H 4000 4500 50  0001 C CNN
F 3 "" H 4000 4500 50  0001 C CNN
	1    4000 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5E49E6CA
P 4000 5700
F 0 "#PWR0116" H 4000 5450 50  0001 C CNN
F 1 "GND" H 4005 5527 50  0000 C CNN
F 2 "" H 4000 5700 50  0001 C CNN
F 3 "" H 4000 5700 50  0001 C CNN
	1    4000 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 5E49EE23
P 4300 6100
F 0 "#PWR0129" H 4300 5850 50  0001 C CNN
F 1 "GND" V 4305 5972 50  0000 R CNN
F 2 "" H 4300 6100 50  0001 C CNN
F 3 "" H 4300 6100 50  0001 C CNN
	1    4300 6100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 5E4A05FE
P 4300 4900
F 0 "#PWR0131" H 4300 4650 50  0001 C CNN
F 1 "GND" V 4305 4772 50  0000 R CNN
F 2 "" H 4300 4900 50  0001 C CNN
F 3 "" H 4300 4900 50  0001 C CNN
	1    4300 4900
	0    1    1    0   
$EndComp
NoConn ~ 4250 3150
NoConn ~ 1700 2850
$Comp
L belt-rescue:3352T-1-103LF-3352T-1-103LF P2
U 1 1 5E3E27DC
P 1500 1600
F 0 "P2" H 1500 1715 50  0000 C CNN
F 1 "3352T-1-103LF" H 1500 1806 50  0000 C CNN
F 2 "Potentiometer_THT:PDB12-H" H 1500 1600 50  0001 L BNN
F 3 "" H 1500 1600 50  0001 L BNN
F 4 "IPC-7351B" H 1500 1600 50  0001 L BNN "Field4"
F 5 "08/19" H 1500 1600 50  0001 L BNN "Field5"
F 6 "Bourns" H 1500 1600 50  0001 L BNN "Field6"
	1    1500 1600
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 5E3E27E2
P 1200 1600
F 0 "#PWR0133" H 1200 1350 50  0001 C CNN
F 1 "GND" V 1205 1472 50  0000 R CNN
F 2 "" H 1200 1600 50  0001 C CNN
F 3 "" H 1200 1600 50  0001 C CNN
	1    1200 1600
	0    1    1    0   
$EndComp
Text Label 1500 1400 0    50   ~ 0
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
P 1800 1050
F 0 "#PWR0110" H 1800 900 50  0001 C CNN
F 1 "+3.3VA" V 1815 1178 50  0000 L CNN
F 2 "" H 1800 1050 50  0001 C CNN
F 3 "" H 1800 1050 50  0001 C CNN
	1    1800 1050
	0    1    1    0   
$EndComp
$Comp
L power:+3.3VA #PWR0114
U 1 1 5E401221
P 1800 1600
F 0 "#PWR0114" H 1800 1450 50  0001 C CNN
F 1 "+3.3VA" V 1815 1728 50  0000 L CNN
F 2 "" H 1800 1600 50  0001 C CNN
F 3 "" H 1800 1600 50  0001 C CNN
	1    1800 1600
	0    1    1    0   
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
F 2 "Resistor_SMD:R_0805_2012Metric" V 6130 6350 50  0001 C CNN
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
L power:+3.3VA #PWR?
U 1 1 5E4C39D5
P 5800 4300
F 0 "#PWR?" H 5800 4150 50  0001 C CNN
F 1 "+3.3VA" V 5815 4428 50  0000 L CNN
F 2 "" H 5800 4300 50  0001 C CNN
F 3 "" H 5800 4300 50  0001 C CNN
	1    5800 4300
	0    1    1    0   
$EndComp
Connection ~ 5800 4300
$EndSCHEMATC
