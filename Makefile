# Pick a C compiler: honour an explicit CC, else use gcc if present,
# else fall back to cc. Override on the command line with: make CC=clang
CC      ?= $(shell command -v gcc >/dev/null 2>&1 && echo gcc || echo cc)
CFLAGS  ?= -std=c99 -Wall -Wextra -O2

demo: demo.c miniahrs.c miniahrs.h
	$(CC) $(CFLAGS) -o demo demo.c miniahrs.c -lm

test: test.c miniahrs.c miniahrs.h
	$(CC) $(CFLAGS) -o test test.c miniahrs.c -lm

readserial: readserial.c miniahrs.c miniahrs.h
	$(CC) $(CFLAGS) -o readserial readserial.c miniahrs.c -lm

run: demo
	./demo output_2026-06-29_10-48-12.log

check: test
	./test

clean:
	rm -f demo demo.exe test test.exe readserial readserial.exe

.PHONY: run check clean
