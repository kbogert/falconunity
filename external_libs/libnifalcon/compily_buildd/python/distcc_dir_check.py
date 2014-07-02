#!/usr/bin/env python2.5

'''
check whether or not DISTCC_HOSTS environment variable is set, and yell if it isn't.
'''

import os
import sys

if __name__ == '__main__':
    distccdir_path = os.path.join(os.path.abspath(sys.path[0]), "../distcc/distccdir")
    if not os.path.exists(distccdir_path):
        distcc_path = os.path.join(os.path.abspath(sys.path[0]), "../distcc/")
        print "================================================================"
        print "distccdir does not exist!"
        print "Go into build_sys/distcc and symlink local, office, or another"
        print "location directory to build_sys/distcc/distccdir"
        print "Command (replace location with local/office/etc...):"
        print "ln -s %s[location] %s" % (distcc_path, distccdir_path)
        print "================================================================"
        sys.exit(1)
    sys.exit(0)
