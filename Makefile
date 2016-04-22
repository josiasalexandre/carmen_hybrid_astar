include ../Makefile.conf

# Module name and description
MODULE_NAME = Hybrid Aster Motion Planner
MODULE_COMMENT = Hybrid A* with path optimization


LINK = g++
CXXFLAGS += -std=c++0x

# Application specific include directories.
#IFLAGS +=

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lgrid_mapping_interface -lmap_server_interface -llocalize_ackerman_interface
CFLAGS += -Wall

# Source code files (.c, .cpp)
SOURCES = hybrid_astar_main.cpp

# Public headers, linked to 'carmen/include/carmen/'.
#PUBLIC_INCLUDES =

## TODO ##


include ../Makefile.rules
