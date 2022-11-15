# serial_to_sbus
Ros node that receives controller command and sends it to the ttyUSB device through uart

## Requirement
The ros node requires the serial library, install it by
```
git clone https://github.com/wjwwood/serial.git
cd serial
make
sudo make install
```

## Configuring joystick
Use *jstest-gtk* to configure joystick mapping. Use *jscal-store* to save the configuration.
```
sudo apt-get install jstest-gtk
sudo jscal-store /dev/input/js<your-joystick-number>
```