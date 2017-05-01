# xDevs.com module for Keithley 2510
# http://xdevs.com/guide/ni_gpib_rpi/
import sys

def print_pos(x, y, text):
    sys.stdout.write("\x1b7\x1b[%d;%df%s\x1b8" % (x, y, text))
    sys.stdout.flush()

def cursor_pos(x, y):
    sys.stdout.write("\x1b7\x1b[%d;%df\x1b8" % (x, y))
    sys.stdout.flush()

def display_clear():
    sys.stdout.write("\x1b[2J")
    sys.stdout.flush()

