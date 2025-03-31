# What program(s) have to be built
PROGRAMS = ex7

# Libraries needed for the executable file
LIBS = -lwsock32

# Dependencies for the program(s) to build
# Default
# ex7: ../common/netutil.o ../common/wperror.o ../common/trkcli.o ../common/utils.o ex7.o
ex7: ../common/remregs.o ../common/netutil.o ../common/wperror.o ../common/robot.o ../common/trkcli.o ../common/utils.o ex7.o

# Includes the common Makefile with the various rules
include ../common/Makefile.inc
