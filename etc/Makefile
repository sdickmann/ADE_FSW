include Make.rules.arm

override LDFLAGS+=-rdynamic -lproc -lsatpkt -lpolydrivers -lm -ldl -lrt -lpthread
override CFLAGS+=-Wall -pedantic -std=gnu99 -g -lpthread

SRC=PTE.c
OBJS=$(SRC:.c=.o)
EXECUTABLE=PTE
CMDS=PTE_util
INSTALL_DEST=$(BIN_PATH)
CMD_FILE=PTE.cmd.cfg

all: $(EXECUTABLE) $(CMDS)

$(EXECUTABLE): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)
	$(STRIP) $@

PTE_util: PTE_util.c
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@
	$(STRIP) $@

install: $(EXECUTABLE) $(CMDS)
	cp -f $(EXECUTABLE) $(INSTALL_DEST)
	cp -f $(CMDS) $(INSTALL_DEST)
	ln -sf PTE_util $(INSTALL_DEST)/PTE-start
	ln -sf PTE_util $(INSTALL_DEST)/PTE-safe
	ln -sf PTE_util $(INSTALL_DEST)/PTE-active
	$(STRIP) $(INSTALL_DEST)/$(EXECUTABLE)
	cp $(CMD_FILE) $(ETC_PATH)

.PHONY: clean install

clean:
	rm -rf *.o $(EXECUTABLE) $(CMDS)