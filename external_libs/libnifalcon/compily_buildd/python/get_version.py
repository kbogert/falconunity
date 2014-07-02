#!/usr/bin/env python

import subprocess
import re
import os
import sys
from optparse import OptionParser

def proc(cmd,cwd):
    stdout = subprocess.Popen(cmd, cwd=cwd, shell=True, stdout=subprocess.PIPE).communicate()[0]
    if stdout:
        stdout.strip()
    return stdout

def get_version(cwd):
    if not os.path.isdir(cwd):
        raise ValueError("Specified directory '%s' is not a directory" % cwd)
    head = proc('git rev-parse --verify HEAD', cwd)
    if head:
        tag = proc('git name-rev --tags HEAD', cwd)
        if re.search('^HEAD\s+(.*~[0-9]*|undefined)$', tag):
            ver = 'git-'+head[:8]
        else:
            ver = tag.split('/')[1].split('^')[0]

        if proc('git diff-index HEAD', cwd):
            ver += '-dirty'
    else:
        ver = 'UNDEF'

    #version was somehow getting a newline in it, return stripped to remove?
    return ver.strip()


version_file = """#include "version.h"

const char COMPILE_TIME[] = __DATE__ " " __TIME__;
const char VERSION[] = "%s";
"""

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-f', '--file', dest='file', action='store', default='', help='File to output version information to')
    parser.add_option('-d', '--dir', dest='dir', action='store', default='', help='Git Repo Directory to get version from')
    (options, args) = parser.parse_args()

    if not options.file or not options.dir:
        parser.print_help()
        sys.exit(1)
    f = open(options.file, "w")
    f.write(version_file % (get_version(options.dir)))
    f.close()
        
