import threading, time, sys, traceback, math

com_port = "/dev/ttyS0" # example: 5 == "COM6" == "/dev/tty5"
baudrate = 115200



offset = 140
init_level = 0
index = 0
numreads = 0
bad = 0
right_dist = []
right_angles = []


lidarData = [[] for i in range(360)] #A list of 360 elements Angle, Distance , quality


def update_view( angle, data ):
    """Updates the view of a sample.

Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
"""
    global numreads
    #unpack data using the denomination used during the discussions
    x = data[0] #byte 1 of 4 <distance 7:0>``
    x1= data[1] #byte 2 of 4 <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
    x2= data[2] #byte 3 of 4 <signal strength 7:0>
    x3= data[3] #byte 4 of 4 <signal strength 15:8>
    
    angle_rad = angle * math.pi / 180.0
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)

    # bit math to extract 13 ? 14 bits of data info from data[i]
    dist_mm = x | (( x1 & 0x3f) << 8) # distance is coded on 13 bits ? 14 bits ?
    quality = x2 | (x3 << 8) # quality is on 16 bits
    lidarData[angle] = [dist_mm,quality]
    dist_x = dist_mm*c
    dist_y = dist_mm*s
    

    if x1 & 0x40:
        numreads = numreads + 1
    if x1 & 0x80:       
        numreads = numreads + 1
    else:
        numreads = numreads + 1
        if (numreads > 350):
            askewYou()
        else:
            nope = 1
        
def dfright(angle, dist_x):
    global right_dist
    global right_angles
    rad = angle * math.pi / 180.0
    d = math.cos(rad)
        #if the array isn't empty, check for duplicate angle. Duplicate angle means new rotation
    if (len(right_dist) != 0):
        for ang in right_angles:
            if (ang == angle):
                #loop reset
                right_angles = []
                right_dist = []
                right_angles.append(angle)
                right_dist.append(dist_x + d*15)
                #print"reset at %d" %angle
                break
            else:
                right_angles.append(angle)
                right_dist.append(dist_x + d*15)
                #print"added angle %d" %angle
                break
    elif (len(right_dist) == 0):
        right_dist.append(dist_x+d*15)
        right_angles.append(angle)
        #print"started at %d" %angle
    else:
        #print"should not happen"
        bad = 69


    if (len(right_dist) < 5):
        bad = 4
    else:
        a = sum(right_dist)
        b = len(right_dist)
        c = a / b
        print"dist from right is %d with %d items in list" %(c,len(right_dist))
def askewYou():
    index = 0
    x_array=[]
    y_array=[]
    x_sq_array=[]
    xy_array=[]
    global lidarData
    for i in range(255,284):
        length=len(lidarData)
        #print "attempting to access lidarData[%d][0] with length %d" % (i,length)
        angle = i * math.pi / 180.0
        x_array.append(math.cos(angle)*lidarData[i][0])
        y_array.append(math.sin(angle)*lidarData[i][0])
        xy_array.append(x_array[index]*y_array[index])
        x_sq_array.append(x_array[index]**2)
        #print "angle %d x dist %d y dist %d" % (i,x_array[index],y_array[index])
        index = index + 1
    xsum=sum(x_array)
    ysum=sum(y_array)
    xysum=sum(xy_array)
    xsqsum=sum(x_sq_array)
    n = len(x_array)
    slope = (( (n*xysum) - (xsum*ysum)) / ( (n*xsqsum) - (xsum**2)))
    #print "xsum %d ysum %d xysum %d xqsum %d n %d" % (xsum,ysum,xysum,xsqsum,n)
    print "%.4f" %slope
      

         
def checksum(data):
    """Compute and return the checksum as an int.

data -- list of 20 bytes (as ints), in the order they arrived in.
"""
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF # truncate to 15 bits
    return int( checksum )


def gui_update_speed(speed_rpm):
    label_speed.text = "RPM : " + str(speed_rpm)

def compute_speed(data):
    speed_rpm = float( data[0] | (data[1] << 8) ) / 64.0
    return speed_rpm

def read_Lidar():
    global init_level, angle, index
    
    nb_errors = 0
    while True:
        try:
            time.sleep(0.00001) # do not hog the processor power

            if init_level == 0 :
                b = ord(ser.read(1))
                # start byte
                if b == 0xFA :
                    init_level = 1
                    #print lidarData
                else:
                    init_level = 0
            elif init_level == 1:
                # position index
                b = ord(ser.read(1))
                # index is the index byte in the 90 packets, going from 0xA0 (packet 0, readings 0 to 3) to 0xF9 (packet 89, readings 356 to 359).
                if b >= 0xA0 and b <= 0xF9 :
                    index = b - 0xA0
                    init_level = 2
                elif b != 0xFA:
                    init_level = 0
            elif init_level == 2 :
                # speed is two byte little endian in 64th of an RPM
                # ord stores char values as ASCII
                b_speed = [ ord(b) for b in ser.read(2)]
                
                # data is 4 bytes long and has the format
                #`byte 0 : <distance 7:0>`
                #`byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
                #`byte 2 : <signal strength 7:0>`
                #`byte 3 : <signal strength 15:8>`
                
                b_data0 = [ ord(b) for b in ser.read(4)]
                b_data1 = [ ord(b) for b in ser.read(4)]
                b_data2 = [ ord(b) for b in ser.read(4)]
                b_data3 = [ ord(b) for b in ser.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion...
                all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

                # checksum
                b_checksum = [ ord(b) for b in ser.read(2) ]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    speed_rpm = compute_speed(b_speed)

                    
                    update_view(index * 4 + 0, b_data0)
                    update_view(index * 4 + 1, b_data1)
                    update_view(index * 4 + 2, b_data2)
                    update_view(index * 4 + 3, b_data3)
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors +=1

                    
                    # display the samples in an error state
                    update_view(index * 4 + 0, [0, 0x80, 0, 0])
                    update_view(index * 4 + 1, [0, 0x80, 0, 0])
                    update_view(index * 4 + 2, [0, 0x80, 0, 0])
                    update_view(index * 4 + 3, [0, 0x80, 0, 0])
                    
                init_level = 0 # reset and wait for the next packet
                
            else: # default, should never happen...
                init_level = 0
        except:
            traceback.print_exc(file=sys.stdout)
            
import serial
ser = serial.Serial(com_port, baudrate)
th = threading.Thread(target=read_Lidar)
th.start()
