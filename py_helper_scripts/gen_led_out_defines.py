




def main():
    for i in range(0,16):
        print('#define PCA9685_LED{}_ON_L  ((uint8_t) (LED_CONTROL_BASE + {}))'.format(i, hex(i*4)))
        print('#define PCA9685_LED{}_ON_H  ((uint8_t) (LED_CONTROL_BASE + {}))'.format(i, hex(i*4+1))) 
        print('#define PCA9685_LED{}_OFF_L ((uint8_t) (LED_CONTROL_BASE + {}))'.format(i, hex(i*4+2)))
        print('#define PCA9685_LED{}_OFF_H ((uint8_t) (LED_CONTROL_BASE + {}))'.format(i, hex(i*4+3)))

if __name__ == '__main__':
    main()
