import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

def stop_motor(channel):
    print "stop plz"
GPIO.add_event_detect(23,GPIO.RISING,callback=stop_motor)
while (True):
    if GPIO.event_detected(23):
        print"bolt"
        break
    else:
        print"no bolt"
    
GPIO.cleanup()
