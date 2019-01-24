MODNAME=core
MODSO=$(MODNAME).so

LUA_CFLAGS=`pkg-config lua5.1 --cflags`
LDFLAGS=`pkg-config lua5.1 --libs`
INCLUDES=$(LUA_CFLAGS)

CCOPT= -O2 -fomit-frame-pointer
CCWARN= -Wall
SOCFLAGS= -fPIC $(CCOPT) $(CCWARN) $(INCLUDES) $(CFLAGS)
SOLDFLAGS= -fPIC $(LDFLAGS)

all: $(MODSO)

install: $(MODSO)
	cp $(MODSO) ..

clean:
	rm *.so *.o

$(MODNAME).o: $(MODNAME).c
	$(CC) $(SOCFLAGS) -c -o $@ $<

$(MODSO): $(MODNAME).o
	$(CC) -shared $(SOLDFLAGS) -o $@ $<

.PHONY: all install clean

