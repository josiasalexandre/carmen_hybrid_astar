/*
 *
 *  Created on: 14/04/2016
 *      Authors: Josias Oliveira, Ranik and Luan
 */

#include <iostream>
#include <csignal>

#include "Entities/Pose2D.hpp"
#include "PathFinding/HybridAstarMotionPlanner.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

// the HybridAstarMotionPlanner publish the path message directly

// the HybridAstarMotionPlanner publish the status message directly

///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

// the HybridAstarMotionPlanner manages all the handlers by itself

static void
signal_handler(int sig)
{
    std::cout << std::endl << "Signal " << sig << " received, exiting program ..." << std::endl;
    exit(1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


// The HybridAstarMotionPlanner object reads and install all the desired parametes
// see astar::HybridAstarMotionPlanner::readAllParameters()


int
main(int argc, char **argv)
{

    // the main HybridAstar object
    astar::HybridAstarMotionPlanner astar_motion_planner(argc, argv);

    // signal
    signal(SIGINT, signal_handler);

    // Iniatilize the IPC
    carmen_ipc_initialize(argc, argv);

    carmen_param_check_version(argv[0]);

    // registering all handlers
    astar_motion_planner.registerAllHandlers();

    carmen_ipc_dispatch();

    return 0;
}
