#!/usr/bin/env python

'''
Enumerate available build targets, and return build target for current system
'''

from optparse import OptionParser
import os
import re, operator
import sys
import platform

BUILD_PREFIX="build"

COMPATIBLE_SYSTEMS = { 'linux' : ['linux', 'windows'],
                       'darwin' : ['darwin', 'windows'],
                       'windows' : ['windows'] }

SYSTEMS = [ 'linux', 'darwin', 'windows' ]

VERSIONS = { 'linux'   : [ 'debian4', 'redhat5' ],
             'darwin'  : [ '10.5', '10.5-gcc42' ],
             'windows' : [ 'mingw4', 'vs9', 'nmake' ],
             }

MACHINES = { 'linux_.*'       : [ 'x86', 'x86-64' ],
             'linux_debian.*' : [ 'ppc' ],
             'darwin'         : [ 'x86' ],
             'windows'        : [ 'x86' ],
             }

GENERATORS = {
               "^windows_nmake.*" : "NMake Makefiles",
               "^windows_vs9.*" : "Visual Studio 9 2008",
               "^windows_mingw.*" : "Unix Makefiles",
               "^(?!windows_).*" : "Unix Makefiles"
               }

EXTRAS = { ".*_mingw4_.*"          : "-DCMAKE_TOOLCHAIN_FILE=../../compily_buildd/cmake/toolchains/mingw_4_toolchain.cmake",
           ".*10.5-gcc42_.*"       : "-DCMAKE_TOOLCHAIN_FILE=../../compily_buildd/cmake/toolchains/darwin_10.5_gcc42_toolchain.cmake",
           "linux_debian.*_x86_.*" : "-DFORCE_32_BIT=TRUE",
           "^(?!windows_vs9).*_release_.*"         : "-DCMAKE_BUILD_TYPE=Release",
           "^(?!windows_vs9).*_debug_.*"           : "-DCMAKE_BUILD_TYPE=Debug",
           "^(?!windows_vs9).*_relwithdebinfo_.*"  : "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
           "(windows_vs9).*_release_.*"         : "-DCMAKE_CONFIGURATION_TYPES=Release",
           "(windows_vs9).*_debug_.*"           : "-DCMAKE_CONFIGURATION_TYPES=Debug",
           "(windows_vs9).*_relwithdebinfo_.*"  : "-DCMAKE_CONFIGURATION_TYPES=RelWithDebInfo",
           ".*_shared$"            : "-DBUILD_SHARED=ON -DBUILD_STATIC=OFF",
           ".*_static$"            : "-DBUILD_SHARED=OFF -DBUILD_STATIC=ON",
           ".*_shared_.*"          : "-DBUILD_SHARED=ON -DBUILD_STATIC=OFF",
           ".*_static_.*"          : "-DBUILD_SHARED=OFF -DBUILD_STATIC=ON",
           ".*_distcc$"            : "-DUSE_DISTCC=ON -DCMAKE_COMPILER_IS_GNUCXX=ON -DCMAKE_COMPILER_IS_GNUC=ON"
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

def clean_target(target):
    return '_'.join(target.split('_')[0:3])

def get_library_dir(target=get_build_target()):
    if re.search(".*_nmake_.*", target):
        target = re.sub("_nmake_", "_vs9_", target)
    return clean_target(target)

def get_build_dir(target=get_build_target()):
    build_target = clean_target(target)

    if not build_target in all_build_targets():
        raise UnsupportedBuildTargetError(build_target)

    return "%s_%s" % (BUILD_PREFIX,target)

def get_distcc_prefix(target=get_build_target()):
    target_list = target.split('_')
    if "distcc" in target_list and "darwin" in target_list:
        distcc_dir_path = os.path.join(os.path.abspath(sys.path[0]), "../distcc/distccdir/")
        distcc_exe_path = os.path.join(os.path.abspath(sys.path[0]), "../distcc/distcc-osx/distcc")
        return "CC=\"env DISTCC_DIR=%s %s /Developer/usr/bin/i686-apple-darwin9-gcc-4.2.1\" CXX=\"env DISTCC_DIR=%s %s /Developer/usr/bin/i686-apple-darwin9-g++-4.2.1\"" % (distcc_dir_path, distcc_exe_path, distcc_dir_path, distcc_exe_path)
    return ""

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-a', '--all', dest='all', action='store_true', default=False, help='Print all available build targets')
    parser.add_option('-l', '--local', dest='local', action='store_true', default=False, help='Print the local build target string')    
    parser.add_option('-c', '--check', dest='check', action='store', default='', help='Check build target string for validity')
    parser.add_option('-e', '--extra', dest='extra', action='store', default='', help='Print extra CMake arguments for target')
    parser.add_option('-b', '--build', dest='build', action='store', default='', help='Print build directory for target')
    parser.add_option('-i', '--library', dest='library', action='store', default='', help='Print library directory for target')
    parser.add_option('-g', '--generator', dest='generator', action='store', default='', help='Print the CMake generator for target')
    parser.add_option('-d', '--distcc', dest='distcc', action='store', default='', help='Print the distcc compiler environment variables for the target')
    (options, args) = parser.parse_args()

    def check(target):
        #strip off build directives
        build_target = clean_target(target)

        if not build_target in all_build_targets():
            print >>sys.stderr, "Build target %s invalid, valid targets are:\n\n%s" % (options.check,'\n\n'.join(all_build_targets()))
            sys.exit(1)
        
        #We know where the library directory is in relation to ourselves, so check for the existence
        if not os.path.isdir("../library/usr_"+get_library_dir(target)):
            print >>sys.stderr, "Build target %s does not have matching library directory.\nPlease make sure [repo_root]/library/usr_%s exists.\nCheck 'Build / Repository Instructions' on the internal wiki for instructions." % (target,target)
            #sys.exit(1)

    if options.all:
        print ' '.join(all_build_targets())
    elif options.local:
        print get_build_target()
    elif options.library:
        print get_library_dir(options.library)
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
    elif options.distcc:
        check(options.distcc)
        print get_distcc_prefix(options.distcc)
    sys.exit(0)
                     
