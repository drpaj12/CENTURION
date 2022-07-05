/*
Copyright (c) 2019 Peter Andrew Jamieson and Bryan Van Scoy (jamieson.peter@gmail.com)

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

This file provides some rudimentary ideas on how to use centurion

1 ----------------------------------------------------------
BEFORE you start dependencies
------------------------------------------------------------
 // Before compiling - Needed other things installed.  Note that we are compiling our Centurion on a remotely attached drive on from the virtualbox.  This means, libxml2 needed to be locally compiled, and linked.
- libxml2 in LIBS directory (NOTE - we are including libxml2.9 in github for now)
    - [can skip stepl if using included libxml] download libxml2 - for example - http://xmlsoft.org/downloads.html and go to DOWNLOADS -> xmlsoft.org/libxml2 and download LATEST_LIBXML2
    - [can skip step if using included libxml] put in LIBS directory 
    - config with your directory for example "./configure --prefix=/home/robot/SIMULATION_WORK/CENTURION/LIBS/libxml2-2.9.9/LOCAL --without-python"
    - "make" (that LOCAL dir NOTE=use your own local)
    - "make install"
    - update CMakeLists.txt with proper version in HINTS (find_library(LIBXML2 NAMES libxml2 xml2 HINTS LIBS/libxml2-2.9.9/LOCAL/lib PATH_SUFFIXES libxml2) and in libxml2 include dir ("include_directories(LIBS/libxml2-2.9.10/LOCAL/include/libxml2")

- libargparse
	- cmake -DCMAKE_BUILD_TYPE=Debug
	- make

- FIXES PETER DID in UBUNTU (Note not sure if needed) to get automake updated - "sudo apt-get autoremove automake" "sudo apt-get install automake autoconf libtool" "autoreconf -i" 

- Need - python3 for scripts

2 -----------------------------------------------------------
Basic Build of centurion
-------------------------------------------------------------
BUILDING

For Debug `cmake -DCMAKE_BUILD_TYPE=Debug`
For Release `cmake -DCMAKE_BUILD_TYPE=Release`

Then `make && make install`

To select between simulating in 2D or 3D,
modify lines 16 and 17 of `./SRC/types.h`

3 -----------------------------------------------------------
Basic Testing
-------------------------------------------------------------
// Then to test

4 -----------------------------------------------------------
Advanced Debug and Tools
-------------------------------------------------------------
// Valgrind - compile with -g (DEBUG)
1. run script in SANDBOX to start up valgrind (see the script there)
2. run gdb in another process and attach

// PRofile
- cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-pg . 
1. make
2. run program

5 -----------------------------------------------------------
How to run
-------------------------------------------------------------
To run centurion from the command line, you specify the arguments as follows:
- 



