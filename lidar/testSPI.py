#Program to test SPI connection between Raspberry PI and MCP3008

import time

#Import MCP3008 library
import Adafruit_MCP3008

#Use software SPI to free up GPIO pin restrictions
#Variable values equal GPIO pin number
CLK  = 18
MISO = 23
MOSI = 24
CS   = 25
#setup SPI and create object
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

#Main function will display values on CH0, CH1, and CH2.
def main():
    while True:
        x = mcp.read_adc(0)
        y = mcp.read_adc(1)
        z = mcp.read_adc(2)

        print x
        print y
        print z
        print ""

        time.sleep(2)


main()
