#include "HybridAstarInterface.hpp"


void carmen_hybrid_astar_define_path_message()
{
    IPC_RETURN_TYPE err;

    err = IPC_defineMsg(CARMEN_HYBRID_ASTAR_PATH_NAME, IPC_VARIABLE_LENGTH, CARMEN_HYBRID_ASTAR_PATH_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_HYBRID_ASTAR_PATH_NAME);
}

void carmen_hybrid_astar_subscribe_path_message(
        carmen_hybrid_astar_path_message_p message,
        carmen_handler_t                 handler,
        carmen_subscribe_t               subscribe_how)
{
    carmen_subscribe_message((char *)CARMEN_HYBRID_ASTAR_PATH_NAME, (char *)CARMEN_HYBRID_ASTAR_PATH_FMT,
                             message, sizeof(carmen_hybrid_astar_path_message_t),
                             handler, subscribe_how);
}

void carmen_hybrid_astar_publish_path_message(carmen_hybrid_astar_path_message_p message) {

    IPC_RETURN_TYPE err;
    static int first_time = 1;

    message->timestamp = carmen_get_time();
    message->host = carmen_get_host();

    err = IPC_publishData(CARMEN_HYBRID_ASTAR_PATH_NAME, message);
    carmen_test_ipc(err, "Could not publish", CARMEN_HYBRID_ASTAR_PATH_NAME);
}
