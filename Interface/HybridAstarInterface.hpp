#ifndef HYBRID_ASTAR_INTERFACE_HPP
#define HYBRID_ASTAR_INTERFACE_HPP

#include "HybridAstarMessage.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void carmen_hybrid_astar_define_path_message();

void carmen_hybrid_astar_subscribe_path_message(carmen_hybrid_astar_path_message_p message,
                                                carmen_handler_t handler,
                                                carmen_subscribe_t subscribe_how);

void carmen_hybrid_astar_publish_path_message(carmen_hybrid_astar_path_message_p message);

#endif
