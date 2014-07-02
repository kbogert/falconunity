The Incredibly Compily Build
============================
Kyle Machulis <kyle@nonpolynomial.com>
Version 1.0.0, August 30, 2009

== Description ==

Compily Buildd is a compilation of python scripts and cmake functions that I've put together over a few years of building cross platform software, mainly of the driver/hardware interaction variety. It relies on cmake for all of the project generation, and python to do things like git version fetching, cmake command line creation, and other utilties that would be annoying if not impossible to do in cmake. 

== What You Probably Want To Do ==

Are you just building an NP Labs project that you got the archive for? Then you most likely want to just build and get it over with versus reading about how the underpinnings of the build system work. In the project root, just do a

--------------------------------------
mkdir build
--------------------------------------

Then

--------------------------------------
cd build
--------------------------------------

Then 

--------------------------------------
cmake ..
--------------------------------------

And if you're missing requirements, cmake will tell you.

However, if you're interested in how I do my personal development or want to use the build system to its full extent, check out the COMPILEY_BUILDD.asciidoc file for more information.

== Credits ==

Compily Buildd is maintained by Kyle Machulis
More information at Nonpolynomial Labs - http://www.nonpolynomial.com

Contributions by
Andrew Schultz

Development of Compily Buildd happens in tandem with the BuildSys system at 510 Systems (http://www.510systems.com)
