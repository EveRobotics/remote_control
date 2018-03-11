#!/usr/bin/python

# Say the neutral position on the joysticlk is 512, 512
# and full left is x=0 and full right is x=1023
# Speed is controlled by the y axis.
# 1023 is full forward.

zeroThreshold = 10

def convertPosToSpeed(x, y):
    left = 128
    right = 128
    
    x -= 512
    y -= 512
    
    if abs(x) < zeroThreshold:
        x = 0
    
    if abs(y) < zeroThreshold:
        y = 0
    
    x /= 8.0 # Scale to +- 64
    y /= 8.0
    
    # Convert X, Y to left right.
    left = left + y - x
    right = right + y + x
    
    if y == 0 and x != 0:
        # Zero turning radius turn:
        left = left - x
        right = right + x
    return (int(left), int(right))

for x in range(0, 1024, 8):
    for y in range(0, 1024, 8):
        print 'X, Y:', x, y, 'Left, Right:', convertPosToSpeed(x, y)


