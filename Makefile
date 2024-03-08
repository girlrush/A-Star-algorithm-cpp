CC = g++
CFLAGS = -std=c++11 -pthread -Iinclude

SRC = main.cpp
OBJ = main.o

PROG = run

all : ${PROG}
${PROG} : ${OBJ}
	${CC} ${CFLAGS} -o ${PROG} ${OBJ}
	rm *.o

${OBJ} : ${SRC}
	$(CC) $(CFLAGS) -c -o $@ $^


