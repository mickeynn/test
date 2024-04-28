Use ArduinoRX8All.c

- RaceChrono DIY BLE (~20Hz send data)
- Read CAN Bus data
- Ask OBD2 via CAN Bus
- Read Analog Data
- Concat all measurements as two RaceChrono DIY BLE CAN messages (https://github.com/aollin/racechrono-ble-diy-device)
- Draw information on two I2C OLED displays
- Add signal through buzzer
- Add signal as Led blinking

Hardware:
- Arduino UNO R4 WiFi ([Amazon](https://www.amazon.de/-/en/Arduino-Proto-Shield-REV3-TSX00083/dp/B087BWJZ4K/ref=sr_1_3_pp?crid=2IYP5HKZUNRA5&dib=eyJ2IjoiMSJ9.gUdpLdgmcdfgRbX45-HKHb9QP7IDxjAZmzYiERFPOiw2hkGiSk6htwG1t7kz2q_Bw8LutgyHOpiFwvS87HgjFmnBpy-wnijTSl-y0QFcZGMAfiYiWEr7965Po53TUSslBQNyOV9nHp46z8Dm0SvuM7NM60DCHKAvM5eJVre84SGyg0CV2dOjXexihqDlGoPF61gVnkkErz09LadcNLxky6wBr-_CEhl7AVJL5vb9BvU.kKPWQUnycytsF2T7wLclMHd7qDL7tjcgwtvkkZJvKqI&dib_tag=se&keywords=arduino+proto+shield&qid=1714330245&sprefix=arduino+proto%2Caps%2C174&sr=8-3))
- Arduino Proto Shield ([Amazon](https://www.amazon.de/-/en/Arduino-Proto-Shield-REV3-TSX00083/dp/B087BWJZ4K/ref=sr_1_3_pp?crid=2IYP5HKZUNRA5&dib=eyJ2IjoiMSJ9.gUdpLdgmcdfgRbX45-HKHb9QP7IDxjAZmzYiERFPOiw2hkGiSk6htwG1t7kz2q_Bw8LutgyHOpiFwvS87HgjFmnBpy-wnijTSl-y0QFcZGMAfiYiWEr7965Po53TUSslBQNyOV9nHp46z8Dm0SvuM7NM60DCHKAvM5eJVre84SGyg0CV2dOjXexihqDlGoPF61gVnkkErz09LadcNLxky6wBr-_CEhl7AVJL5vb9BvU.kKPWQUnycytsF2T7wLclMHd7qDL7tjcgwtvkkZJvKqI&dib_tag=se&keywords=arduino+proto+shield&qid=1714330245&sprefix=arduino+proto%2Caps%2C174&sr=8-3))
- SN65HVD230 CAN Transmitter ([Amazon](https://www.amazon.de/SN65HVD230-CAN-Board/dp/B00KM6XMXO/ref=sr_1_1_sspa?crid=RORFOKT8IHCF&dib=eyJ2IjoiMSJ9.UK_rAtKl8t8qNu4YdQwEvyqhoEE-LRSePy_N-1Yv0_mo0gwdPavhLw21YxaPSnrzJ38jCVpji-qx9NdWP9lbyl1QG2o5GdM0CEu3OtnnKbl8GyADYRKd7UcSflaYbeRV9GOCO6AaZgLyUk_ZvEKR9vvTcS3GQIChjNuM-pPpPQtAESJBNACGEL2q15vzK0pxmps_HEmB9aBytQTrwHX5izL99iArST2afbopcu9VUbI.PbhxnwOgmOxBWNi3lUCRDUxPE_NPOn7DH1utRiMR06A&dib_tag=se&keywords=sn65hvd230&qid=1714330260&sprefix=%2Caps%2C190&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1))
- 0.96' OLED I2C Displays ([Amazon](https://www.amazon.de/-/en/0-96-Display-Self-Luminous-Compatible-Raspberry/dp/B0BWMXQF8H/ref=sr_1_3?crid=7NBPRF1JOTMI&dib=eyJ2IjoiMSJ9.8jmmTmrPsJq1Ki0bV72aTd8fKp-0InhSuuhGmbtcllhvq5ZnYz8puSUVLMLrb8MlMGcdWfXAYA7nyfiXAQIxXczD9KthIXr2_iKAvGd_-wdgMgksqZrrIZw2JcTLs2kzSf6G5wE7dRWJvRPSFgX6uTSMdeH0vdWPwXgbZxi0WdpxA4PgjsysCLxwkLO4gJjp5ITzqMIP69tFVhM3rJVcfG3xQz9MugPULeGKhMDvMqA.WIrWyysUtFUr1U6TCjeSPoO3XEWfwUIli_dLZmF2YEY&dib_tag=se&keywords=oled+0.96+i2c&qid=1714330269&sprefix=oled+0%2Caps%2C185&sr=8-3))


Useful links:
https://github.com/olikraus/u8g2/blob/master/sys/arduino/u8g2_full_buffer/UpdateArea/UpdateArea.ino
https://github.com/MagnusThome/esp32_obd2/blob/master/src/esp32_obd2.cpp
https://github.com/sandeepmistry/arduino-OBD2/blob/master/src/OBD2.h
https://github.com/arduino/ArduinoCore-renesas/tree/b5a3b9dee4c659e6ee173f5eb4143c0e021ed6c3/libraries/Arduino_CAN/src
https://github.com/DaveBlackH/MazdaRX8Arduino
https://github.com/Radivv/arduino-display-obd2-can-mazda-rx8
https://docs.arduino.cc/tutorials/uno-r4-wifi/can/

patch timeout here:
/Users/n.mikolaichuk/Library/Arduino15/packages/arduino/hardware/renesas_uno/1.1.0/libraries/Wire/Wire.cpp
```
TwoWire::TwoWire(int scl, int sda, WireAddressMode_t am /*= ADDRESS_MODE_7_BITS*/, bool prefer_sci /*= false*/) :
  scl_pin(scl),
  sda_pin(sda),
  init_ok(false),
  is_master(true),
  is_sci(false),
  address_mode(am),
  timeout(10),
```

