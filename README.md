# Yaoji - An AR Digital Plant on Arduino and Android
妖姬 - 基于Arduino和Android的虚拟电子植物

----------

## 1. About Yaoji
Yaoji is an open-source digital plant based on Arduino and Android. It is based on the popular idea of Augmented Reality (AR). By using the deployed sensors and Leds on Yaoji, users can interact with Yaoji and see its response on an Android phone.

![](\photos\yaoji-finished.png)

![](\photos\android-main.png)

Yaoji has five full-RGB leds, one illumination sensor, one temperature & humidity sensor and one infrared sensor. Yaoji uses an Arduino Nano 3.0 as the core controller and a wifi module as a bridge to communicate with the Android app. It was a prize-winning (third prize) project in Geekon Hackmarthon in 2013.

![](\photos\hardware-arch.jpg)


## 2. Project Directory
### /documents
descriptions, datasheets and communication protocol

### /hardware
schematics, pcb and mechanical CAD

### /photos
photos of Yaoji.

### /software
- **/arduino**: Arduino source code and libraries of the firmware (.ino).

- **/android**: user application on Android (.apk).

- **/windowns**: a Windows demo of a sound analyzer that transfer sound spectrum into LED color (Visual Studio Project).


## 3. Contributors
|Name    |Role|
|---     |----|
|YunFei  | System Design & Arduino Programming |
|leepood | Android App Development |
|Congbin Zhong | Hardware Design and Debug |