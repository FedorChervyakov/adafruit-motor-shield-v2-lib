
def main():
    for i in range(0,16):
        print("#define PCA9685_LED{} ((uint8_t) {})".format(i,i))

if __name__ == '__main__':
    main()
