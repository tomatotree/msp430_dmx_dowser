EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:embedded-microcontrollers
LIBS:opendous
LIBS:SN65176B
LIBS:LD1117x33
LIBS:custom
LIBS:dips-s
LIBS:Dowser-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "DMX Dowser"
Date "2 sep 2014"
Rev "2"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L C C3
U 1 1 5250EA26
P 4750 2850
F 0 "C3" H 4750 2950 40  0000 L CNN
F 1 "0.1uF" H 4756 2765 40  0000 L CNN
F 2 "~" H 4788 2700 30  0000 C CNN
F 3 "~" H 4750 2850 60  0000 C CNN
	1    4750 2850
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5250EA64
P 5000 2850
F 0 "C4" H 5000 2950 40  0000 L CNN
F 1 "10uF" H 5006 2765 40  0000 L CNN
F 2 "~" H 5038 2700 30  0000 C CNN
F 3 "~" H 5000 2850 60  0000 C CNN
	1    5000 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5250EAA5
P 5350 3150
F 0 "#PWR01" H 5350 3150 30  0001 C CNN
F 1 "GND" H 5350 3080 30  0001 C CNN
F 2 "" H 5350 3150 60  0000 C CNN
F 3 "" H 5350 3150 60  0000 C CNN
	1    5350 3150
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5250EBB5
P 4350 4500
F 0 "R1" V 4430 4500 40  0000 C CNN
F 1 "47K" V 4357 4501 40  0000 C CNN
F 2 "~" V 4280 4500 30  0000 C CNN
F 3 "~" H 4350 4500 30  0000 C CNN
	1    4350 4500
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5250EBC8
P 4150 5100
F 0 "C2" H 4150 5200 40  0000 L CNN
F 1 "2.2uF" H 4156 5015 40  0000 L CNN
F 2 "~" H 4188 4950 30  0000 C CNN
F 3 "~" H 4150 5100 60  0000 C CNN
	1    4150 5100
	0    -1   -1   0   
$EndComp
$Comp
L JUMPER JP1
U 1 1 5251B5B6
P 7500 1350
F 0 "JP1" H 7500 1500 60  0000 C CNN
F 1 "JUMPER" H 7500 1270 40  0000 C CNN
F 2 "~" H 7500 1350 60  0000 C CNN
F 3 "~" H 7500 1350 60  0000 C CNN
	1    7500 1350
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 5251B617
P 8050 1650
F 0 "C5" H 8050 1750 40  0000 L CNN
F 1 "0.1uF" H 8056 1565 40  0000 L CNN
F 2 "~" H 8088 1500 30  0000 C CNN
F 3 "~" H 8050 1650 60  0000 C CNN
	1    8050 1650
	0    -1   -1   0   
$EndComp
$Comp
L CONN_3 J1
U 1 1 5251B764
P 1400 3150
F 0 "J1" V 1350 3150 50  0000 C CNN
F 1 "RS485_TH" V 1450 3150 40  0000 C CNN
F 2 "" H 1400 3150 60  0000 C CNN
F 3 "" H 1400 3150 60  0000 C CNN
	1    1400 3150
	-1   0    0    -1  
$EndComp
$Comp
L CONN_3 J2
U 1 1 5251B77D
P 1400 3850
F 0 "J2" V 1350 3850 50  0000 C CNN
F 1 "RS485_IN" V 1450 3850 40  0000 C CNN
F 2 "" H 1400 3850 60  0000 C CNN
F 3 "" H 1400 3850 60  0000 C CNN
	1    1400 3850
	-1   0    0    -1  
$EndComp
$Comp
L CONN_4 J3
U 1 1 5251B7BD
P 1400 4700
F 0 "J3" V 1350 4700 50  0000 C CNN
F 1 "PROGRAM" V 1450 4700 50  0000 C CNN
F 2 "" H 1400 4700 60  0000 C CNN
F 3 "" H 1400 4700 60  0000 C CNN
	1    1400 4700
	-1   0    0    -1  
$EndComp
$Comp
L CONN_3 J4
U 1 1 5251BCAC
P 9950 6150
F 0 "J4" V 9900 6150 50  0000 C CNN
F 1 "SERVO" V 10000 6150 40  0000 C CNN
F 2 "" H 9950 6150 60  0000 C CNN
F 3 "" H 9950 6150 60  0000 C CNN
	1    9950 6150
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 5251BD30
P 9400 5750
F 0 "C6" H 9400 5850 40  0000 L CNN
F 1 "0.1uF" H 9406 5665 40  0000 L CNN
F 2 "~" H 9438 5600 30  0000 C CNN
F 3 "~" H 9400 5750 60  0000 C CNN
	1    9400 5750
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR02
U 1 1 5251BD67
P 9200 6350
F 0 "#PWR02" H 9200 6350 30  0001 C CNN
F 1 "GND" H 9200 6280 30  0001 C CNN
F 2 "" H 9200 6350 60  0000 C CNN
F 3 "" H 9200 6350 60  0000 C CNN
	1    9200 6350
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 J5
U 1 1 5251DA24
P 1400 2100
F 0 "J5" V 1350 2100 40  0000 C CNN
F 1 "5VDC_IN" V 1450 2100 40  0000 C CNN
F 2 "" H 1400 2100 60  0000 C CNN
F 3 "" H 1400 2100 60  0000 C CNN
	1    1400 2100
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 5251DAD3
P 2400 4100
F 0 "#PWR03" H 2400 4100 30  0001 C CNN
F 1 "GND" H 2400 4030 30  0001 C CNN
F 2 "" H 2400 4100 60  0000 C CNN
F 3 "" H 2400 4100 60  0000 C CNN
	1    2400 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 3050 5350 3050
Wire Wire Line
	5350 3050 5350 3150
Connection ~ 5000 3050
Wire Wire Line
	6500 2650 6500 3250
Wire Wire Line
	1750 4750 4900 4750
Wire Wire Line
	4350 2650 4350 4250
Connection ~ 6500 2650
Wire Wire Line
	4350 2650 7700 2650
Wire Wire Line
	1750 4550 4150 4550
Wire Wire Line
	4150 4550 4150 4150
Wire Wire Line
	4150 4150 4350 4150
Connection ~ 4350 4150
Connection ~ 4350 4750
Wire Wire Line
	1750 4850 4900 4850
Wire Wire Line
	1750 4650 3850 4650
Connection ~ 3850 5100
$Comp
L GND #PWR04
U 1 1 5251DB3F
P 3850 5250
F 0 "#PWR04" H 3850 5250 30  0001 C CNN
F 1 "GND" H 3850 5180 30  0001 C CNN
F 2 "" H 3850 5250 60  0000 C CNN
F 3 "" H 3850 5250 60  0000 C CNN
	1    3850 5250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5251DB58
P 6500 5700
F 0 "#PWR05" H 6500 5700 30  0001 C CNN
F 1 "GND" H 6500 5630 30  0001 C CNN
F 2 "" H 6500 5700 60  0000 C CNN
F 3 "" H 6500 5700 60  0000 C CNN
	1    6500 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 4650 3850 5250
$Comp
L +5V #PWR06
U 1 1 5251DBF9
P 9600 5400
F 0 "#PWR06" H 9600 5490 20  0001 C CNN
F 1 "+5V" H 9600 5490 30  0000 C CNN
F 2 "" H 9600 5400 60  0000 C CNN
F 3 "" H 9600 5400 60  0000 C CNN
	1    9600 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 5400 9600 6050
Connection ~ 9600 5750
$Comp
L +5V #PWR07
U 1 1 5251DD64
P 7200 950
F 0 "#PWR07" H 7200 1040 20  0001 C CNN
F 1 "+5V" H 7200 1040 30  0000 C CNN
F 2 "" H 7200 950 60  0000 C CNN
F 3 "" H 7200 950 60  0000 C CNN
	1    7200 950 
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR08
U 1 1 5251DD76
P 1900 1800
F 0 "#PWR08" H 1900 1890 20  0001 C CNN
F 1 "+5V" H 1900 1890 30  0000 C CNN
F 2 "" H 1900 1800 60  0000 C CNN
F 3 "" H 1900 1800 60  0000 C CNN
	1    1900 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5251DD85
P 1900 2350
F 0 "#PWR09" H 1900 2350 30  0001 C CNN
F 1 "GND" H 1900 2280 30  0001 C CNN
F 2 "" H 1900 2350 60  0000 C CNN
F 3 "" H 1900 2350 60  0000 C CNN
	1    1900 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2000 2100 2000
Wire Wire Line
	1750 2200 2100 2200
Wire Wire Line
	1900 2200 1900 2350
$Comp
L SN65176B U2
U 1 1 5251EABC
P 3150 3800
F 0 "U2" H 3150 3700 50  0000 C CNN
F 1 "SN65176B" H 3150 3900 50  0000 C CNN
F 2 "MODULE" H 3150 3800 50  0001 C CNN
F 3 "DOCUMENTATION" H 3150 3800 50  0001 C CNN
	1    3150 3800
	-1   0    0    1   
$EndComp
$Comp
L C C1
U 1 1 5251ECF3
P 2600 3300
F 0 "C1" H 2600 3400 40  0000 L CNN
F 1 "0.1uF" H 2606 3215 40  0000 L CNN
F 2 "~" H 2638 3150 30  0000 C CNN
F 3 "~" H 2600 3300 60  0000 C CNN
	1    2600 3300
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR010
U 1 1 5251ED6A
P 2200 3450
F 0 "#PWR010" H 2200 3540 20  0001 C CNN
F 1 "+5V" H 2200 3540 30  0000 C CNN
F 2 "" H 2200 3450 60  0000 C CNN
F 3 "" H 2200 3450 60  0000 C CNN
	1    2200 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 3250 1850 3250
Wire Wire Line
	1750 3150 1900 3150
Wire Wire Line
	2400 3950 2400 4100
Wire Wire Line
	2400 3650 2400 3300
Wire Wire Line
	2200 3450 2400 3450
Connection ~ 2400 3450
Wire Wire Line
	2800 3050 2800 3300
$Comp
L GND #PWR011
U 1 1 5251F341
P 2950 3250
F 0 "#PWR011" H 2950 3250 30  0001 C CNN
F 1 "GND" H 2950 3180 30  0001 C CNN
F 2 "" H 2950 3250 60  0000 C CNN
F 3 "" H 2950 3250 60  0000 C CNN
	1    2950 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 3250 2950 3150
Wire Wire Line
	2950 3150 2800 3150
Connection ~ 2800 3150
Wire Wire Line
	4900 3950 3900 3950
Wire Wire Line
	4900 3850 3900 3850
Wire Wire Line
	3900 3750 4000 3750
Wire Wire Line
	4000 3750 4000 3850
Connection ~ 4000 3850
Wire Wire Line
	3900 3650 4650 3650
Wire Wire Line
	4650 3650 4650 4050
Wire Wire Line
	4650 4050 4900 4050
Wire Wire Line
	9200 5750 9200 6350
Wire Wire Line
	9600 6250 9200 6250
Connection ~ 9200 6250
Wire Wire Line
	6500 5600 6500 5700
Wire Wire Line
	4750 6150 9600 6150
$Comp
L LD1117X33 U3
U 1 1 5252077A
P 7700 2050
F 0 "U3" H 7700 2200 60  0000 C CNN
F 1 "LD1117X33" H 8000 1700 60  0000 C CNN
F 2 "~" H 7700 2050 60  0000 C CNN
F 3 "~" H 7700 2050 60  0000 C CNN
	1    7700 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 1350 8250 2050
Wire Wire Line
	7850 1650 7150 1650
Wire Wire Line
	7150 1650 7150 2250
$Comp
L GND #PWR012
U 1 1 5252099F
P 7150 2250
F 0 "#PWR012" H 7150 2250 30  0001 C CNN
F 1 "GND" H 7150 2180 30  0001 C CNN
F 2 "" H 7150 2250 60  0000 C CNN
F 3 "" H 7150 2250 60  0000 C CNN
	1    7150 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 950  7200 1350
Wire Wire Line
	7800 1350 8250 1350
Connection ~ 7150 2050
NoConn ~ 4900 4450
NoConn ~ 4900 4550
NoConn ~ 4900 5150
Connection ~ 8250 1650
Connection ~ 1950 3050
$Comp
L PWR_FLAG #FLG013
U 1 1 52524F4E
P 2100 1900
F 0 "#FLG013" H 2100 1995 30  0001 C CNN
F 1 "PWR_FLAG" H 2100 2080 30  0000 C CNN
F 2 "" H 2100 1900 60  0000 C CNN
F 3 "" H 2100 1900 60  0000 C CNN
	1    2100 1900
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG014
U 1 1 52524F5E
P 2100 2200
F 0 "#FLG014" H 2100 2295 30  0001 C CNN
F 1 "PWR_FLAG" H 2100 2380 30  0000 C CNN
F 2 "" H 2100 2200 60  0000 C CNN
F 3 "" H 2100 2200 60  0000 C CNN
	1    2100 2200
	1    0    0    1   
$EndComp
Connection ~ 1900 2200
Wire Wire Line
	1900 2000 1900 1800
Wire Wire Line
	2100 2000 2100 1900
Connection ~ 1900 2000
$Comp
L PWR_FLAG #FLG015
U 1 1 525250A7
P 8050 1350
F 0 "#FLG015" H 8050 1445 30  0001 C CNN
F 1 "PWR_FLAG" H 8050 1530 30  0000 C CNN
F 2 "" H 8050 1350 60  0000 C CNN
F 3 "" H 8050 1350 60  0000 C CNN
	1    8050 1350
	1    0    0    -1  
$EndComp
Connection ~ 8050 1350
$Comp
L MSP430G2553IN20 U1
U 1 1 5250E375
P 5200 3550
F 0 "U1" H 7650 3650 60  0000 C CNN
F 1 "MSP430G2553IN20" H 6500 2250 60  0000 C CNN
F 2 "~" H 5200 3550 60  0000 C CNN
F 3 "~" H 5200 3550 60  0000 C CNN
	1    5200 3550
	1    0    0    -1  
$EndComp
Connection ~ 4750 2650
Connection ~ 5000 2650
Wire Wire Line
	4900 5050 4750 5050
Wire Wire Line
	4750 5050 4750 6150
$Comp
L DIPS_09 SW1
U 1 1 532A2F17
P 8700 4800
F 0 "SW1" V 8200 4800 60  0000 C CNN
F 1 "DIPS_09" V 9200 4800 60  0000 C CNN
F 2 "" H 8700 4800 60  0000 C CNN
F 3 "" H 8700 4800 60  0000 C CNN
	1    8700 4800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 5100 3950 5100
Wire Wire Line
	4350 5100 4350 4750
$Comp
L GND #PWR016
U 1 1 532B4DF7
P 8900 5500
F 0 "#PWR016" H 8900 5500 30  0001 C CNN
F 1 "GND" H 8900 5430 30  0001 C CNN
F 2 "" H 8900 5500 60  0000 C CNN
F 3 "" H 8900 5500 60  0000 C CNN
	1    8900 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 4400 8900 5500
Connection ~ 8900 4400
Connection ~ 8900 4500
Connection ~ 8900 4600
Connection ~ 8900 4700
Connection ~ 8900 4800
Connection ~ 8900 4900
Connection ~ 8900 5000
Connection ~ 8900 5100
Connection ~ 8900 5200
$Comp
L R R2
U 1 1 54055986
P 2550 3050
F 0 "R2" V 2630 3050 40  0000 C CNN
F 1 "100" V 2557 3051 40  0000 C CNN
F 2 "~" V 2480 3050 30  0000 C CNN
F 3 "~" H 2550 3050 30  0000 C CNN
	1    2550 3050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 3050 2300 3050
Wire Wire Line
	2400 3850 1750 3850
Wire Wire Line
	1900 3150 1900 3850
Connection ~ 1900 3850
Wire Wire Line
	1750 3950 2200 3950
Wire Wire Line
	2200 3950 2200 3750
Wire Wire Line
	2200 3750 2400 3750
Wire Wire Line
	1850 3250 1850 3950
Connection ~ 1850 3950
Wire Wire Line
	1950 3050 1950 3750
Wire Wire Line
	1950 3750 1750 3750
$EndSCHEMATC