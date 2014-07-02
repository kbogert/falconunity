#!/usr/bin/env python

'''
Enumerate available build targets, and return build target for current system
'''

from optparse import OptionParser
import os
import re, operator
import sys
import platform
import subprocess

class TargetBuilder:
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
    
    EXTRAS = { ".*_mingw4_.*"          : ["-DCMAKE_TOOLCHAIN_FILE=../../build_sys/cmake/toolchains/mingw_4_toolchain.cmake"],
               ".*_ppc_.*"             : ["-DCMAKE_TOOLCHAIN_FILE=../../build_sys/cmake/toolchains/eldk_4.1_ppc-4xxFP_toolchain.cmake"],
               ".*10.5-gcc42_.*"       : ["-DCMAKE_TOOLCHAIN_FILE=../../build_sys/cmake/toolchains/darwin_10.5_gcc42_toolchain.cmake"],
               "linux_debian.*_x86_.*" : ["-DFORCE_32_BIT=TRUE"],
               "^(?!windows_vs9).*_release_.*"         : ["-DCMAKE_BUILD_TYPE=Release"],
               "^(?!windows_vs9).*_debug_.*"           : ["-DCMAKE_BUILD_TYPE=Debug"],
               "^(?!windows_vs9).*_relwithdebinfo_.*"  : ["-DCMAKE_BUILD_TYPE=RelWithDebInfo"],
               "^(windows_vs9).*_release_.*"         : ["-DCMAKE_CONFIGURATION_TYPES=Release"],
               "^(windows_vs9).*_debug_.*"           : ["-DCMAKE_CONFIGURATION_TYPES=Debug"],
               "^(windows_vs9).*_relwithdebinfo_.*"  : ["-DCMAKE_CONFIGURATION_TYPES=RelWithDebInfo"],
               ".*_shared$"            : ["-DBUILD_SHARED=ON", "-DBUILD_STATIC=OFF"],
               ".*_static$"            : ["-DBUILD_SHARED=OFF", "-DBUILD_STATIC=ON"],
               ".*_shared_.*"          : ["-DBUILD_SHARED=ON", "-DBUILD_STATIC=OFF"],
               ".*_static_.*"          : ["-DBUILD_SHARED=OFF", "-DBUILD_STATIC=ON"],
               ".*_distcc$"            : ["-DUSE_DISTCC=ON", "-DCMAKE_COMPILER_IS_GNUCXX=ON", "-DCMAKE_COMPILER_IS_GNUC=ON"]
               }

    def __init__(self, target_string = ""):
        self.target = target_string
        return

    def help(self):
        print  "\nAvailable build targets:"
#	print  "\tlocal     - Build for the local architecture"
	print  "\ttests     - Run the automatic test runner"
	print  "\ttestdoc   - Generate test plan document"
#	print  "\tclean-all - Clean all build directories"
	print  ""
	print  "Additional arguments to build targets:"
	print  "\tBuild Types:" 
	print  "\t\trelease"
	print  "\t\trelwithdebinfo"
	print  "\t\tdebug"
	print  ""
	print  "\tLibrary Types"
	print  "\t\tstatic"
	print  "\t\tshared"
	print  ""
	print  "\t\tBuilt Types"
	print  "\t\tdistcc (OS X only)"
	print  
	print  "Format:"
	print  "\t[target]_[build type]_[library type]"
	print  "\tex."
	print  "\t\tdarwin_10.5_x86_release_shared_distcc"
	print  "\tex."
	print  "\t\twindows_mingw4_x86_debug_static"
	print  ""
	print  "Valid build targets:"
        for t in self.all_build_targets():
            print "\t%s" % (t)

    def all_build_targets(self):
        '''
        Get a listing of all build targets available
        '''
        build_targets = []
        local_system = platform.system().lower()
        for system in self.SYSTEMS:
            if system not in self.COMPATIBLE_SYSTEMS[local_system]:
                continue
            for version in self.VERSIONS[system]:
                for machine in reduce(operator.add, [mach for match,mach in self.MACHINES.items() if re.search(match, "%(system)s_%(version)s" % locals())]):
                    build_targets += ["%(system)s_%(version)s_%(machine)s" % locals()]
                
        build_targets.sort()
        return build_targets


    class UnsupportedBuildTargetError(Exception):
        def __init__(self, target):
            self.target = target

        def __str__(self):
            return "Target %s not supported (supported targets are %s)" % (self.target, self.all_build_targets())

    def get_build_target(self):
        '''
        Get the build target string for the current machine
        '''
        def linux():
            return ''.join(platform.dist()[0:2]).split('.')[0]

        def darwin():
            return '.'.join(platform.mac_ver()[0].split('.')[0:2])

        def windows():
            #return  platform.win32_ver()[0]
            return 'mingw4'
          

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
        if not build_target in self.all_build_targets():
            raise self.UnsupportedBuildTargetError(build_target)

        return build_target

    def clean_target(self):
        return '_'.join(self.target.split('_')[0:3])

    def get_library_dir(self):
        if re.search(".*_nmake_.*", self.target):
            target = re.sub("_nmake_", "_vs9_", self.target)
        return self.clean_target()

    def get_build_dir(self):
        build_target = self.clean_target()

        if not build_target in self.all_build_targets():
            raise self.UnsupportedBuildTargetError(build_target)

        return "%s_%s" % (self.BUILD_PREFIX,self.target)

    def get_distcc_prefix(self):
        target_list = self.target.split('_')
        if "distcc" in target_list and "darwin" in target_list:
            distcc_dir_path = os.path.join(os.path.abspath(sys.path[0]), "../build_sys/distcc/distccdir/")
            distcc_exe_path = os.path.join(os.path.abspath(sys.path[0]), "../build_sys/distcc/distcc-osx/distcc")
            return ["env", "CC=env DISTCC_DIR=%s %s /Developer/usr/bin/i686-apple-darwin9-gcc-4.2.1" % (distcc_dir_path, distcc_exe_path), "CXX=env DISTCC_DIR=%s %s /Developer/usr/bin/i686-apple-darwin9-g++-4.2.1" % (distcc_dir_path, distcc_exe_path)]
        return []

    def create_cmake_target(self):
        print "Setting up build directory for %s" % (self.target)
        cmake_build_dir = self.get_build_dir()
        if not os.path.exists (cmake_build_dir):
            os.makedirs(cmake_build_dir)
        os.chdir(cmake_build_dir)
        cmake_generator = "%s" % (' '.join([generator for match,generator in self.GENERATORS.items() if re.search(match, self.target)]))
        cmake_build_platform = self.get_library_dir()
        cmake_extras = [extra for match,extra in self.EXTRAS.items() if re.search(match, self.target)]
        print cmake_extras
        cmake_command = self.get_distcc_prefix()
        #sum flattens the list of lists we get back from extras
        cmake_command += ["cmake", "-G", cmake_generator, "-DBUILDSYS_BUILD_PLATFORM=%s" % cmake_build_platform, "-DBUILDSYS_BUILD_DIR=%s" % cmake_build_dir] + sum(cmake_extras, []) + [".."]
        cmake_line = "cd %s; %s" % (cmake_build_dir, " ".join(cmake_command))
        print cmake_line
        r = subprocess.call(cmake_command)
        return

    def check(self):
        #strip off build directives
        build_target = self.clean_target()

        if not build_target in self.all_build_targets():
            print >>sys.stderr, "Build target %s invalid, valid targets are:\n\n%s" % (self.target,'\n\n'.join(self.all_build_targets()))
            sys.exit(1)
        
        #We know where the library directory is in relation to ourselves, so check for the existence
        if not os.path.isdir("../library/usr_" + self.get_library_dir()):
            print >>sys.stderr, "Build target %s does not have matching library directory.\nPlease make sure [repo_root]/library/usr_%s exists.\nCheck 'Build / Repository Instructions' on the internal wiki for instructions." % (self.target,self.target)
            sys.exit(1)
