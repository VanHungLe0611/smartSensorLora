.PHONY: all clean install debug_build release_build
default:all
all:
debug_build:
		scons -j8 debug=g3
release_build:
		scons -j8 release=1
clean:
		scons -c
rtags:
		scons -j8 debug=g3 nocomstrings=0
install:		
		scons install
