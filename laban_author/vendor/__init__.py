# Upstream LabanotationSuite algorithm, vendored unmodified. See UPSTREAM.md.
#
# kp_extractor.py does "import accessory as ac", the flat import that worked
# when both files sat in LabanEditor's src/labanotation/tool directory. Putting
# this package's own directory on sys.path keeps that import working, which is
# what lets the file stay byte-identical to upstream.
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
