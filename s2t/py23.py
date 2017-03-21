import sys

if sys.version_info >= (3, 0):
    from queue import Queue
else:
    from Queue import Queue
