# Copyright (C) 2009, 2010 Hans Pirnay. 
# All Rights Reserved.
# This code is published under the Eclipse Public License.
#
# Author: Hans Pirnay, 2009-12-15


#############################################################################
# Do not modify anything below unless you know what you're doing.
prefix      = /home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled
exec_prefix = ${prefix}
libdir      = ${exec_prefix}/lib
bindir      = ${exec_prefix}/bin
CYGPATH_W   = echo
SRCDIR      = .
ipoptsrcdir  = $(SRCDIR)/../../../src
senssrcdir = $(SRCDIR)/../src

CXX         = g++
CXXFLAGS    = -O3 -pipe -DNDEBUG    -DIPOPT_BUILD
LDFLAGS     = $(CXXFLAGS)  -Wl,--rpath -Wl,/home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib

# Include directories (we use the CYGPATH_W variables to allow compilation with Windows compilers)
#   we also need some Ipopt headers that are not installed, so we add $(ipoptsrcdir)/Algorithm and $(ipoptsrcdir)/LinAlg
INCL = `PKG_CONFIG_PATH=/home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib64/pkgconfig:/home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib/pkgconfig:/home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/share/pkgconfig: pkg-config --cflags ipoptamplinterface` -I`$(CYGPATH_W) $(ipoptsrcdir)/Algorithm` -I`$(CYGPATH_W) $(ipoptsrcdir)/LinAlg` -I`$(CYGPATH_W) $(senssrcdir)` $(ADDINCFLAGS)
#INCL = -I`$(CYGPATH_W) /home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/include/coin`   -I`$(CYGPATH_W) $(ipoptsrcdir)/Algorithm` -I`$(CYGPATH_W) $(ipoptsrcdir)/LinAlg` -I`$(CYGPATH_W) $(senssrcdir)` $(ADDINCFLAGS)

# Linker flags
LIBS = -lsipopt `PKG_CONFIG_PATH=/home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib64/pkgconfig:/home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib/pkgconfig:/home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/share/pkgconfig: pkg-config --libs ipoptamplinterface`
##LIBS = -link -libpath:`$(CYGPATH_W) /home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib` libsipopt.lib libipoptamplinterface.lib libipopt.lib /home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib/libcoinlapack.a /home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib/libcoinblas.a -lgfortran  -L/usr/lib/gcc/x86_64-linux-gnu/9 -L/usr/lib/gcc/x86_64-linux-gnu/9/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/9/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/9/../../.. -lgfortran -lm -lquadmath /home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib/libcoinblas.a -lgfortran  -L/usr/lib/gcc/x86_64-linux-gnu/9 -L/usr/lib/gcc/x86_64-linux-gnu/9/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/9/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/9/../../.. -lgfortran -lm -lquadmath -lm  -ldl 
#LIBS = -L/home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib -lsipopt -lipoptamplinterface -lipopt /home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib/libcoinlapack.a /home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib/libcoinblas.a -lgfortran  -L/usr/lib/gcc/x86_64-linux-gnu/9 -L/usr/lib/gcc/x86_64-linux-gnu/9/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/9/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/9/../../.. -lgfortran -lm -lquadmath /home/phillip/Phoenix/catkin_ws/src/drive_ros_config/modules/drive_ros_trajectory_tracking_control/IPOPT/compiled/lib/libcoinblas.a -lgfortran  -L/usr/lib/gcc/x86_64-linux-gnu/9 -L/usr/lib/gcc/x86_64-linux-gnu/9/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/9/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/9/../../.. -lgfortran -lm -lquadmath -lm  -ldl 

#LIBTOOL = $(SHELL) $(top_builddir)/../libtool
#CXXLD = $(CXX)
#CXXLINK = $(LIBTOOL) --tag=CXX --mode=link $(CXXLD) $(AM_CXXFLAGS) \
#	$(CXXFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@

sIPOPT_LIB = libsipopt.so

sIPOPT_LIBNAME =$(sIPOPT_LIB).0.0.0

TARGET = ipopt_sens

OBJS   = SensAmplTNLP.o 

VPATH = $(SRCDIR):$(ipoptsrcdir)/Apps/AmplSolver

all: $(OBJS)

install: $(TARGET)
	if test -d $(bindir); then : ; else mkdir $(bindir); fi
	cp $(TARGET) $(bindir)

uninstall:
	rm -f $(bindir)/$(TARGET)

$(TARGET): $(OBJS) ampl_sipopt.cpp
	$(CXX) $(LDFLAGS) $(SRCDIR)/ampl_sipopt.cpp -o $(TARGET) $(OBJS) $(INCL) $(LIBS)

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) $(INCL) -o $@ -c $^

clean:
	rm -f $(OBJS) $(TARGET)

distclean: clean
