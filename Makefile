# makefile $Id: Makefile 539 2014-08-15 10:19:43Z jcan $
#
# libraryname (for shared library)
libname = aulinefollower.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include
CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
CPPFLAGS += -I/opt/vc/include/interface/vcos/pthreads
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g3 -shared -Wl,-soname,$(libname)
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g3 -O0 -Wall -pedantic -fPIC $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
# ifeq ($(OPENCV),2.1)
#   export OPENCV=
# endif
# ifeq ($(OPENCV),)
# else
#   CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
# endif
#
# Object files to produce before link
sourcescpp = ufunclinefollower.cpp control.h control.cpp uart.h uart.cpp linefollower.h linefollower.cpp
#
sourcesh = $(sourcescpp:.cpp=.h)
OBJECTS = $(sourcescpp:.cpp=.o)

# shared library file name (version 0)
shlib = $(libname).0

all: $(shlib)
# compile all - all objects depend on all used files
$(shlib): $(OBJECTS) Makefile $(sources)
	c++ -o $(shlib) $(OBJECTS) $(LDFLAGS)

.o: $(sources) $(sourcesh)
	c++ $@



.PHONY : clean install
clean :
	rm -f $(shlib) $(OBJECTS)
	-@rm -fr *~ .deps

install:
	cp $(shlib) ../../build/lib

######################################################################
#
# Automatic dependencies

DEPS_MAGIC := $(shell mkdir -p .deps)


%.o: .deps/%.d

.deps/%.d: src/%.c
	@cc $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' 

.deps/%.d: %.cpp
	@g++ $(CFLAGS) -MM $< | sed 's#^$(@F:%.d=%.o):#$@ $(@:.deps/%.d=%.o):#' > $@



######################################################################
#
# Include automatic dependencies

-include $(patsubst %.o, .deps/%.d, $(objects))
