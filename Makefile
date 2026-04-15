CC      = gcc
LDLIBS  = -lm

# ---- SA Engine shared-library extension (datapump-based) ----
#
# Requires SA_ENGINE_HOME to point to the sa.engine repo root.
# Headers used: sa_core.h, sa_datapump.h, sa_syscalls.h, sa_threads.h
# (all in $SA_ENGINE_HOME/C).
#
# Example:
#   export SA_ENGINE_HOME=/home/erik/proj/sa.engine
#   make rpi-sensehat.so
#
# NOTE: the .so drops the "sa." prefix that the source files carry —
# SA Engine's load_extension treats any dot in the extension name as
# a filename suffix and skips auto-appending ".so", which breaks
# load_extension("sa.rpi-sensehat"). Dot-free names work cleanly.

SA_INCLUDES = -I$(SA_ENGINE_HOME)/C
SA_CFLAGS   = -O2 -fPIC -DUNIX=1 -DLINUX=1 -U_FORTIFY_SOURCE
SA_LFLAGS   = -shared -rdynamic

rpi-sensehat.so: sa.rpi-sensehat.c
	$(CC) $(SA_CFLAGS) $(SA_INCLUDES) $(SA_LFLAGS) -o $@ $< $(LDLIBS)

# Copy the .so to SA_ENGINE_HOME/bin so load_extension("rpi-sensehat") finds it.
install: rpi-sensehat.so
	cp rpi-sensehat.so $(SA_ENGINE_HOME)/bin/

test: install
	$(SA_ENGINE_HOME)/bin/sa.engine -O test-sa.rpi-sensehat.osql

clean:
	rm -f rpi-sensehat.so

.PHONY: clean install test
