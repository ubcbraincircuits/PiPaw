#!/usr/bin/python3

import src.encoder
from time import sleep
from sys import stdout

#Settins for Right Encoder class:
CSX = 0
CLK = 1000000
BYTEMD = 4

lever = classReadEncoder.Encoder(CSX,CLK,BYTEMD)


#start = time()
#data = []
#freq = 0

while True:
    try:
        print('Right Counter reads [%s].' % str(lever.readCounter()), end='\r', flush=True)
        sleep(0.1)
        #print str(rLever.readCounter())
        #print 'CNTRSIZE = ', str(rLever.counterSize)
        #data.append(rLever.readCounter())
        #if time()-start > 10:
            #print '60 seconds passed.'
            #print data
            #print str(len(data))
            #break
    except KeyboardInterrupt:
        lever.close()
        break

print('Cya!')
