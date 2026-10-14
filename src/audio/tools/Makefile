CC ?= clang
CFLAGS ?= -O2 -Wall -Wextra
FRAMEWORKS = -framework IOKit -framework CoreFoundation

.PHONY: all clean

all: matebook_ctl

matebook_ctl: matebook_ctl.c
	$(CC) $(CFLAGS) -o $@ $< $(FRAMEWORKS)

clean:
	rm -f matebook_ctl
