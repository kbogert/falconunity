#!/usr/bin/env python

import os
import sys


# The compile_driver script can go one of 3 places:
# - The python directory of the compily_buildd repo 
# - The root of a repo using compily_buildd
# - In the python directory of a repo using compily_buildd as a submodule
# So, we check for it in all of those places, using the above list as the search order also 
build_sys_paths = [".", "../compily_buildd/python/", "../../compily_buildd/python/"]

for p in build_sys_paths:
    if os.path.isdir(p):
        sys.path.append(p)

try:
    from TargetBuilder import TargetBuilder
except ImportError:
    print "Cannot find build system module, exiting..."
    sys.exit(1)


def main(argv=None):
    if argv is None:
        argv = sys.argv
    if len(argv) <= 1:
        t = TargetBuilder()
        t.help()
        sys.exit(0)
    try:
        t = TargetBuilder(argv[1])
        t.create_cmake_target()
    except TargetBuilder.UnsupportedBuildTargetError:
        t = TargetBuilder()
        t.help()
        
if __name__ == "__main__":
    sys.exit(main())

