#!/usr/bin/env python2.5

# Build platform name creation code
# Originally By 510 Systems
# http://www.510systems.com
# Modified for the Nonpolynomial Labs Build System by Kyle Machulis
# http://www.nonpolynomial.com/

'''
Enumerate available build targets, and return build target for current system
'''

from optparse import OptionParser
import os
import re, operator
import sys
import platform

COMPATIBLE_SYSTEMS = { 'linux' : ['linux', 'windows'],
                       'darwin' : ['darwin', 'windows'],
                       'windows' : ['windows'] }

SYSTEMS = [ 'linux', 'darwin', 'windows' ]

VERSIONS = { 'linux'   : [ 'debian4', 'redhat5' ],
             'darwin'  : [ '10.5' ],
             'windows' : [ 'mingw4' ], # Only support mingw cross-compiler right now
             }

MACHINES = { 'linux_.*'       : [ 'x86', 'x86-64' ],
             'linux_debian.*' : [ 'ppc' ],
             'darwin'         : [ 'x86' ],
             'windows'        : [ 'x86' ],
             }

GENERATORS = { ".*" : "Unix Makefiles"
               }

EXTRAS = { ".*_mingw4_.*"        : "-DCMAKE_TOOLCHAIN_FILE=../../build_sys/cmake/toolchains/mingw_4_toolchain.cmake",
           ".*_ppc$"             : "-DCMAKE_TOOLCHAIN_FILE=../../build_sys/cmake/toolchains/eldk_4.1_ppc-4xxFP_toolchain.cmake",
           "linux_debian.*_x86$" : "-DFORCE_32_BIT=TRUE",
          }

def all_build_targets():
    '''
    Get a listing of all build targets available
    '''
    build_targets = []
    local_system = platform.system().lower()
    for system in SYSTEMS:
        if system not in COMPATIBLE_SYSTEMS[local_system]:
            continue
        for version in VERSIONS[system]:
            for machine in reduce(operator.add, [mach for match,mach in MACHINES.items() if re.search(match, "%(system)s_%(version)s" % locals())]):
                build_targets += ["%(system)s_%(version)s_%(machine)s" % locals()]
                
    build_targets.sort()
    return build_targets

class UnsupportedBuildTargetError(Exception):
    def __init__(self, target):
        self.target = target

    def __str__(self):
        return "Target %s not supported (supported targets are %s)" % (self.target, all_build_targets())

def get_build_target():
    '''
    Get the build target string for the current machine
    '''
    def linux():
        return ''.join(platform.dist()[0:2]).split('.')[0]

    def darwin():
        return '.'.join(platform.mac_ver()[0].split('.')[0:2])

    def windows():
        return 'mingw4'
        #return  platform.win32_ver()[0]

    def machine():
        mach = platform.machine()
        
        # All 32-bit x86 machines (i386, i686, etc) will just become x86 for simplicity
        if re.search('i.86', mach):
            mach = 'x86'

        return mach

    system = platform.system().lower()
    version = locals()[system]().lower().replace('_','-')
    machine = machine().lower().replace('_','-')

    build_target = "%(system)s_%(version)s_%(machine)s" % locals()

    # Check that our build target is supported
    if not build_target in all_build_targets():
        raise UnsupportedBuildTargetError(build_target)

    return build_target

def get_build_dir(target=get_build_target()):
    if not target in all_build_targets():
        raise UnsupportedBuildTargetError(target)

    return "build_%s" % target

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-a', '--all', dest='all', action='store_true', default=False, help='Print all available build targets')
    parser.add_option('-l', '--local', dest='local', action='store_true', default=False, help='Print the local build target string')    
    parser.add_option('-c', '--check', dest='check', action='store', default='', help='Check build target string for validity')
    parser.add_option('-e', '--extra', dest='extra', action='store', default='', help='Print extra CMake arguments for target')
    parser.add_option('-b', '--build', dest='build', action='store', default='', help='Print build directory for target')
    parser.add_option('-g', '--generator', dest='generator', action='store', default='', help='Print the CMake generator for target')
    (options, args) = parser.parse_args()

    def check(target):
        #We know where the library directory is in relation to ourselves, so check for the existence
        if not os.path.isdir("../../library/usr_"+target):
            print >>sys.stderr, "Build target %s does not have matching library directory.\nPlease make sure [repo_root]/library/usr_%s exists.\nCheck 'Build / Repository Instructions' on the internal wiki for instructions." % (target,target)
            sys.exit(1)
        if not target in all_build_targets():
            print >>sys.stderr, "Build target %s invalid, valid targets are:\n\n%s" % (options.check,'\n\n'.join(all_build_targets()))
            sys.exit(1)

    if options.all:
        print ' '.join(all_build_targets())
    elif options.local:
        print get_build_target()        
    elif options.check:
        check(options.check)
    elif options.extra:
        check(options.extra)
        extras = ' '.join([extra for match,extra in EXTRAS.items() if re.search(match, options.extra)])
        print extras
    elif options.generator:
        check(options.generator)
        generator = ' '.join([generator for match,generator in GENERATORS.items() if re.search(match, options.generator)])
        print "\"%s\"" % generator
    elif options.build:
        check(options.build)
        print get_build_dir(options.build)

    sys.exit(0)
                     
