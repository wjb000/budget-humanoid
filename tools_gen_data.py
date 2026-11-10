import json
B=lambda cat,part,unit,q,basis,src=None,href=None:dict(cat=cat,part=part,unit=unit,q=q,basis=basis,src=src,href=href)
L,S,E="Listed on page","Search listing","Estimate"
BOM=[
B("Actuators","Feetech STS3215 12 V serial-bus servo (30 kg·cm, 55 g): all joints except hip pitch and knee",19.9,15,L,"DFRobot","https://www.dfrobot.com/product-2962.html"),
B("Actuators","Feetech STS3250 12 V servo (50 kg·cm, 74.5 g): hip pitch and knee, 2 per leg",64.99,4,L,"BABSCO","https://shop.babsco.com/feetech-sts3250-50kg-heavy-duty-smart-servo"),
B("Compute","Raspberry Pi 5, 2 GB (vision, voice, high-level)",77.5,1,L,"Raspberry Pi","https://www.raspberrypi.com/products/raspberry-pi-5/"),
B("Compute","Seeed XIAO ESP32-S3 (real-time controller)",7.9,1,S,"Seeed Studio","https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html"),
B("Compute","microSD card 32 GB",8,1,E),
B("Compute","Raspberry Pi 5 active cooler",5,1,E),
B("Sensors & audio","Waveshare Bus Servo Adapter (A), TTL/UART mode",4.99,2,S,"Waveshare","https://www.waveshare.com/product/bus-servo-adapter-a.htm"),
B("Sensors & audio","Adafruit BNO085 9-DOF IMU (fused orientation, I²C)",29.5,1,L,"Adafruit","https://www.adafruit.com/product/4754"),
B("Sensors & audio","Raspberry Pi Camera Module 3 (from $25)",25,1,L,"Raspberry Pi","https://www.raspberrypi.com/products/camera-module-3/"),
B("Sensors & audio","Pi 5 camera cable adapter",4,1,E),
B("Sensors & audio","I²S MEMS microphone (SPH0645)",6.95,1,L,"Adafruit","https://www.adafruit.com/product/3421"),
B("Sensors & audio","I²S 3 W amp (MAX98357A); out of stock at Adafruit when checked, any clone works",5.95,1,L,"Adafruit","https://www.adafruit.com/product/3006"),
B("Sensors & audio","40 mm 4 Ω speaker; out of stock at Adafruit when checked",4.95,1,L,"Adafruit","https://www.adafruit.com/product/3968"),
B("Power & safety","3S LiPo 11.1 V 2200 mAh 25C, XT60",19.99,1,L,"Monkey Hobby","https://www.monkeyhobby.com/volantexrc-2200mah-3s-11-1v-25c-lipo-battery-xt60-plug/"),
B("Power & safety","Pololu D24V50F5 5 V / 5 A buck (Pi 5 rail)",32.95,1,L,"Pololu","https://www.pololu.com/product/2851"),
B("Power & safety","E-stop: latching NC mushroom button",7.5,1,L,"RobotShop","https://www.robotshop.com/products/emergency-stop-switch"),
B("Power & safety","12 V automotive relay (30 A class) + MOSFET driver",8,1,E),
B("Power & safety","Fuse holder + 30 A blade fuse",4,1,E),
B("Power & safety","XT60 connectors, 14–16 AWG wire, terminal block",10,1,E),
B("Power & safety","LiPo low-voltage alarm",4,1,E),
B("Power & safety","3S balance charger (skip if you own one)",25,1,E),
B("Structure & misc","PLA/PETG filament, 1 kg spool (printed body is about 750 g at the estimated fill, 23 parts in 14 unique designs; printer not included)",20,1,E),
B("Structure & misc","M2 + M3 socket/pan-head screw assortment kits, 2 kits (about 100 x M2, 76 x M3 used; no bearings in this design)",20,1,E),
B("Structure & misc","M2 brass heat-set inserts, 100-pack (8 used for the front cover and battery door). COST-ADDING vs plain screws",7,1,E),
B("Structure & misc","XT60 panel-mount connector for the battery door (charge/discharge port). COST-ADDING vs a loose pigtail",1.5,1,E),
B("Structure & misc","Hook-and-loop battery straps (2) + 2.5 x 100 mm cable ties (about 30)",4,1,E),
B("Structure & misc","3-pin servo cables and extensions",12,1,E),
]
DOF=[
["Neck","Yaw","Z","19","–","STS3215"],
["Arm","Shoulder pitch","Y","13","16","STS3215"],["Arm","Elbow pitch","Y","14","17","STS3215"],["Gripper","Open/close","–","15","18","STS3215"],
["Leg","Hip yaw","Z","1","7","STS3215"],["Leg","Hip roll","X","2","8","STS3215"],["Leg","Hip pitch","Y","3","9","STS3250"],["Leg","Knee pitch","Y","4","10","STS3250"],["Leg","Ankle pitch","Y","5","11","STS3215"],["Leg","Ankle roll","X","6","12","STS3215"],
]
open('data.js','w').write("const BOM=%s;\nconst DOF=%s;\n"%tuple(json.dumps(x,ensure_ascii=False) for x in (BOM,DOF)))
