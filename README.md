![V1 to V2 Block Manager pic](https://user-images.githubusercontent.com/6006120/110375337-b2089c80-8006-11eb-9887-97105ffbceb0.jpg)
# DKblock - V1
// 20-clients
Firmware for open source DKblock battery management system. The code is written in mostly C with some C++ routines. It is written as mostly inline code for transparency and for speed of development. This firmware is designed for the Teensy 3.2 (for Pack Supervisor) and the Teensy LC (for Block Manager) boards from the super awesome PJRC at www.pjrc.com.

Hardware descriptions and video tutorials are available at https://dkblock922508958.wordpress.com

DKblock V2 is under development. 
Goals are lower cost, higher performance. Biggest improvement in is quality of RF communications. Per engineering good practice, we use a WAS/IS chart.
WAS:              IS:
RF comms 2.4Ghz   RF comms are 915Mhz for longer range, lower errors
20 clients        100's of clients
$100 per node     $25 per node (all SMT except 2 parts)
Teensy LC         TI CC1310 radio for node,CC1352p-2 for collector 

All DKblock software and hardware is released as open source hardware (OSHW) as defined by the OSHWA: https://www.oshwa.org/definition/ and under the JSON license defined at https://www.json.org/license.html
