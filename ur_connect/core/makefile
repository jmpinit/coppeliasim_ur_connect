MODNAME=core
MODSO=../$(MODNAME).so

LUA_CFLAGS=`pkg-config lua5.3 --cflags`
LDFLAGS=-L/Applications/coppeliaSim.app/Contents/Frameworks -llua.5.3

CCWARN= -Wall

all: $(MODSO)

install: $(MODSO)
	cp $(MODSO) ..

clean:
	rm -f $(MODNAME).o
	rm -f $(MODSO)

$(MODNAME).o: $(MODNAME).c
	$(CC) -c $(LUA_CFLAGS) -fPIC -o $@ $<

$(MODSO): $(MODNAME).o
	$(CC) -shared -fPIC -o $@ $< $(LDFLAGS)

.PHONY: all install clean

