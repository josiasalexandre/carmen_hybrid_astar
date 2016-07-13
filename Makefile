include ../Makefile.conf

# Module name and description
MODULE_NAME = Hybrid Aster Path Finder
MODULE_COMMENT = Hybrid A* with path optimization


LINK = g++
CXXFLAGS += -Os -std=c++0x
CFLAGS += -Wall

# Application specific include directories.
#IFLAGS +=

SUBDIRS += Interface

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lgrid_mapping_interface -lmap_server_interface -llocalize_ackerman_interface -lm

# Source code files (.c, .cpp)
SOURCES = hybrid_astar_path_finder_main.cpp Interface/hybrid_astar_interface.cpp PathFinding/HybridAstarPathFinder.cpp

PUBLIC_BINARIES = hybrid_astar_path_finder_main
PUBLIC_LIBRARIES = libhybrid_astar_interface.a

TARGETS = hybrid_astar_path_finder_main libhybrid_astar_interface.a

# Public headers, linked to 'carmen/include/carmen/'.
#PUBLIC_INCLUDES =

## TODO ##

hybrid_astar_path_finder_main: hybrid_astar_path_finder_main.o lib_hybrid_astar_interface.a hybrid_astar_path_finder

lib_hybrid_astar_interface.a: Interface/hybrid_astar_interface.o

hybrid_astar_path_finder: PathFinding/HybridAstarPathFinder.o

include ../Makefile.rules
