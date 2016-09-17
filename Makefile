
CC:=gcc

CFLAG:= -O2 -Wall

FILENAME:= control.elf

all:
	$(CC) $(CFLAG) -o $(FILENAME) main.c

clean:
	rm -rf $(FILENAME)

