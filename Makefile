
PROG=fiatelka

MCU=attiny13a

all: 
	avr-gcc -Wall -Os -mmcu=${MCU} -S -o ${PROG}.s ${PROG}.c
	avr-gcc -Wall -Os  -Xlinker -Map=${PROG}.map  -mmcu=${MCU} -o ${PROG}.elf ${PROG}.c
	avr-size -C ${PROG}.elf
	avr-objcopy -j .text -j .data -O binary ${PROG}.elf ${PROG}.bin
    
clean:
	rm -f *.bin *.hex *.map *.elf
    
flash:
	scp ${PROG}.bin pi@raspberrypi:~/avr
	ssh pi@raspberrypi "cd avr/attiny_pypi_flasher/ && python3 kluchomat.py --low_speed write ../${PROG}.bin"
