import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/z1mmer/Documents/robotics/upmoon25/install/bbot'
