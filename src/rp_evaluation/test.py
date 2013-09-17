#!/usr/bin/python
from collections import defaultdict
import cohen_fleiss

foo = cohen_fleiss.ratings([1,4,2,3,3,3,2,3,2,3,1,4,4,4], [3,3,4,1,2,1,4,3,2,3,1,4,4,4], 4)

print foo