from pyfirmata import Arduino, util
import time
import pyfirmata
import numpy

port ='COM5'
board = Arduino('COM5')

it = util.Iterator(board)
it.start()


board.analog[0].mode = pyfirmata.INPUT
board.analog[1].mode = pyfirmata.INPUT
board.digital[9].mode = pyfirmata.OUTPUT
board.digital[10].mode = pyfirmata.OUTPUT

while True:
    #(board.analog[0].read()*20)
    #x1 = float(senstemp_val)
    #tank1 = (senstemp_val*20)
    print("values of Sensor1:")
    print(board.analog[0].read()*20)
    time.sleep(0.2)
    
    print(board.analog[1].read()*20)
    time.sleep(0.2)
  

    board.digital[9].write(1)
    board.digital[10].write(0)

   
    
