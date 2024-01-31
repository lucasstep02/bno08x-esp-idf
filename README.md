# BNO055 Test

This is a simple test for BNO055.

It just resets the BNO055 and starts receiving Euler angles.

### Important: BNO055 doesn't seem to work on I2C 400kHz - I had to set it to 100kHz

### Running
Run the `idf.py menuconfig` and set the SCL and SDA pins for the I2C. 
Optionally, you can also set the pin for LED.

Once done you can deploy (the port may be different):

    idf.py flash monitor -p /dev/ttyACM0