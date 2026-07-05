import sys
p = sys.argv[1]
s = open(p, 'rb').read()
s = s.replace(b'\r', b'')
if s[:3] == b'\xef\xbb\xbf':
    s = s[3:]
open(p, 'wb').write(s)
print('cleaned %s bytes=%d' % (p, len(s)))
