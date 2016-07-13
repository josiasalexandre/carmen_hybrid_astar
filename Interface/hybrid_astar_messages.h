#ifndef HYBRID_ASTAR_MESSAGE_HPP
#define HYBRID_ASTAR_MESSAGEHPP

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    carmen_ackerman_path_point_p path;
    int path_length;
    double timestamp;
    char *host;
} carmen_hybrid_astar_path_message_t, *carmen_hybrid_astar_path_message_p;

#define CARMEN_HYBRID_ASTAR_PATH_NAME "carmen_hybrid_astar_path_name"
#define CARMEN_HYBRID_ASTAR_PATH_FMT "{<{double, double, double, double, double, double}:2>,int,double,string}"

#ifdef __cplusplus
}
#endif

#endif
