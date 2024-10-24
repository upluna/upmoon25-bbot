import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/upmoon25/Documents/upmoon25/upmoon25-bbot/install/bbot'
