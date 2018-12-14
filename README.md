# BT-TrackPad-ESP32
A self-built Bluetooth-Trackpad , that uses a ESP32 and a GT911-board.  
 
  
  
This is a personal playground project to learn a thing or two, by combining a generic ESP32-board and a cheap GT911-touchscreen. 
It makes use of the "ble_hid_device_demo" from Espressif and the Goodix-Library for ESP32-idf.  
  
  
I hope to be able to add more functionality in the future.  
  
  
  
  
Current hardware prototype:

<img src="https://github.com/Staars/BT-TrackPad-ESP32/blob/master/HackPad.jpeg" align="center" />
  
  
  
# current status:
  
  - simple HID-Trackpad that reports single point events 
  - short tap for left mouse button
  
  
# planned features:
  
  - adding multi-touch (the sensor can track 5 points/fingers)
  - gestures
  - maybe adding a hardware button
  
