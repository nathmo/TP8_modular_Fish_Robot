# What program(s) have to be built
PROGRAMS = ex6

# Libraries needed for the executable file
LIBS = -lwsock32

# Dependencies for the program(s) to build
# Default
# ex6: ../common/netutil.o ../common/wperror.o ../common/trkcli.o ../common/utils.o ex6.o
# 6.1
ex6: ../common/remregs.o ../common/netutil.o ../common/wperror.o ../common/robot.o ../common/trkcli.o ../common/utils.o ex61.o

# Includes the common Makefile with the various rules
include ../common/Makefile.inc
