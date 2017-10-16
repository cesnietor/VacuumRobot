# VacuumRobot
![alt text](images/intro.jpg "DIY VacuumRobot")

# Created by César Nieto
DIY Vacuum Robot project
An open source project designed to get into the robotics and programming (it is not needed to know how to program).
The full instructions on how to build the robot are on Instructables: 
  https://www.instructables.com/id/Build-Your-Own-Vacuum-Robot/ 


This is my first Vacuum Robot, which it's main purpose is to allow anyone to have a cleaning robot without paying so much money, to learn how they work, to build a nice robot that you can modify, update and program as much as you want, and of course to vacuum all that annoying fluff.

This project is intended to be as easy to build as possible since all the elements and parts are easy to find on Digikey, eBay, Amazon, etc.

The whole chassis was designed in Solidworks so that it could be 3d printed.

Currently it uses an Arduino Uno (if you don't like it too much you can easily change it for another micro controller, I decided to use this since my objective is that anyone could actually build it), micro-metal motors, fan propeller, infrared sensors and respective driver modules.

Another one bites the dust!

Instructions
=====


Step 1: Materials
-----
So, first I will define all the materials that I used and later I will suggest other options with a similar behavior.

**Controllers:**

* 1 x Arduino Uno Board (or similar) ([DigiKey](http://www.digikey.com/product-detail/en/arduino/A000066/1050-1024-ND/2784006))
* 1 x IRF520 MOS FET Driver Module ([Aliexpress](https://es.aliexpress.com/store/product/0-24VTop-Mosfet-Button-IRF520-MOS-Driver-Module-For-Arduino-MCU-ARM-Raspberry-pie/1185416_32275524365.html?spm=2114.04010208.3.19.g1nCuK&ws_ab_test=searchweb0_0,searchweb201602_3_10000560_10000073_10000561_10000074_10000175_10000507_10000401_10000505_10000068_10000063_10099_10000156_10096_10000569_10000097_10000094_10000090_10000091_10000147_10000144_10084_10000150_10083_10080_10000153_10082_10081_10110_10111_10112_10113_10000535_10114_10000534_10000089_10000086_10000083_10000135_10000080_10078_10079_10073_10000140_10070_10122_10123_10126_10124_10000546_10065_10068_10501_10000132_10000033_10503_10000030_10000126_10000026_10000129_10000023_10000123_432_10060_10062_10056_10055_10054_302_10059_10000120_10000020_10000117_10000013_10103_10102_10000016_10000114_10000111_10052_10053_10050_10107_10051_10106_10000621_10000384_10000101_10000100_10000579_10000104_10000045_10000578_10000108_10000612_10000613_10000390_10000042_10000592_10000039_10000587_10000036_10000389_10000187-10503_10501,searchweb201603_2,afswitch_1,ppcSwitch_4,single_sort_1_default&btsid=23d4dbb7-b390-4b6d-8de8-456e740343d8&algo_expid=c1fe14ed-efb6-4d8b-8aa1-2ef2a833c598-5&algo_pvid=c1fe14ed-efb6-4d8b-8aa1-2ef2a833c598))
* 1 x H-bridge L298 Dual Motor Driver ([Aliexpress](https://es.aliexpress.com/store/product/Dual-H-Bridge-DC-Stepper-Motor-Drive-Controller-Board-Module-L298N/712084_32273246252.html?spm=2114.04010208.3.10.hlghTt&ws_ab_test=searchweb0_0,searchweb201602_3_10000560_10000073_10000561_10000074_10000175_10000507_10000401_10000505_10000068_10000063_10099_10000156_10096_10000569_10000097_10000094_10000090_10000091_10000147_10000144_10084_10000150_10083_10080_10000153_10082_10081_10110_10111_10112_10113_10000535_10114_10000534_10000089_10000086_10000083_10000135_10000080_10078_10079_10073_10000140_10070_10122_10123_10126_10124_10000546_10065_10068_10501_10000132_10000033_10503_10000030_10000126_10000026_10000129_10000023_10000123_432_10060_10062_10056_10055_10054_302_10059_10000120_10000020_10000117_10000013_10103_10102_10000016_10000114_10000111_10052_10053_10050_10107_10051_10106_10000621_10000384_10000101_10000100_10000579_10000104_10000045_10000578_10000108_10000612_10000613_10000390_10000042_10000592_10000039_10000587_10000036_10000389_10000187-10503_10501,searchweb201603_2,afswitch_1,ppcSwitch_4,single_sort_1_default&btsid=41cc97c3-c03d-4f3a-9b5a-d7c7f16f9c39&algo_expid=c5bd03f1-c8e0-4179-81da-6a2f469cb3b0-4&algo_pvid=c5bd03f1-c8e0-4179-81da-6a2f469cb3b0))

**Actuators:**

* 2 x Micro Metal Gearmotor HP 6V 298:1 ([DigiKey](http://www.digikey.com/product-detail/en/sparkfun-electronics/ROB-12285/1568-1161-ND/5673747))
* 1 x Micro Metal Gearmotor Bracket Pair ([Pololu](https://www.pololu.com/product/1086))
* 1 x Wheel 42×19mm Pair ([DigiKey](http://www.digikey.com/product-detail/en/sparkfun-electronics/ROB-08899/ROB-08899-ND/5814304))
* 1 x Fan Blower AVC BA10033B12G 12V or similar (BCB1012UH Neato's motor) ([Ebay](http://www.ebay.com/itm/1pc-AVC-BA10033B12G-fan-9733-12V-4-5A-4pin-M3529-QL-/182281112508?_trksid=p2349526.m2548.l4275), [NeatoOption](http://www.ebay.com/itm/Neato-Vacuum-Fan-and-motor-Impeller-xv-Series-xv-11-xv-14-xv-15-xv-12-xv-21-/201157367856?hash=item2ed5e9d830:g:VHwAAOSw-jhT~I0i))

**Sensors:**

* 2 x Sharp Distance Sensor GP2Y0A41SK0F (4 - 30cm) ([DigiKey](http://www.digikey.com/product-detail/en/sharp-microelectronics/GP2Y0A41SK0F/425-2819-ND/3884447))

**Power:**

* 1 x ZIPPY Compact 1300mAh 3S 25C Lipo Pack ([HobbyKing](https://hobbyking.com/en_us/zippy-compact-1300mah-3s-25c-lipo-pack.html))
* 1 x LiPo Battery Charger 3s ([Amazon-Charger](https://www.amazon.com/Turnigy-Compact-Lipo-Charger-100-240v/dp/B00XU4ZR06/ref=sr_1_1?s=toys-and-games&ie=UTF8&qid=1488313025&sr=1-1&keywords=turnigy+lipo+charger+3s+2s))
* 1 x 1k Ohm resistor
* 1 x 2k Ohm small potentiometer

**3d Printing:**

* 3D printer with a minimum printing size of 21 L x 21 W cm .
* PLA Fillament or similar.
* If you don't have, you can print your file on [3DHubs](https://www.3dhubs.com/3dprint).

**Other materials:**

* 20 x M3 bolts with (3mm diameter)
* 20 x M3 nuts
* 2 x #8-32 x 2 IN bolts with nuts and washer.
* 1 x Vaccum bag filter (cloth type)
* 1 x Ball Caster with 3/4″ Plastic or Metal Ball ([Pololu](https://www.pololu.com/product/955))
* 2 pushbuttons
* 1 x On/Off Switch

**Tools:**

* Screw driver
* Soldering Iron
* Pliers
* Scissors
* Cable (3m)



Awesome people share!


 
