# OmniPicker_usb2can

[OmniPicker](https://www.zhiyuan-robot.com/DOCS/PM/X1) Gripper Control Interface in Python using a USB-CAN adapter(support transparent transmission).

### Getting Started
```bash
pip3 install pyserial
sudo chmod 777 /dev/ttyUSB0
```

### To Use
```python
from OmniPicker_usb2can import OmniPicker_Interface

interface = OmniPicker_Interface()
interface.open()
interface.close()
```