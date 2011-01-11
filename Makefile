all:
	gcc -g -O2 -Wall kingrid.c -o kingrid -lfreenect -lm

debug:
	gcc -g -O0 -Wall kingrid.c -o kingrid -lfreenect -lm


