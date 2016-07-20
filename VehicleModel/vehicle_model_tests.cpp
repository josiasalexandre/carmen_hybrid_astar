#include <iostream>

#include "VehicleModel.hpp"

double
get_phi_from_curvature(double curvature, double v, double understeer_coeficient, double distance_between_front_and_rear_axles)
{
	double phi;

	phi = atan(curvature * distance_between_front_and_rear_axles) * (1.0 + v * v * understeer_coeficient);

	return (phi);
}

double
get_curvature_from_phi(double phi, double v, double understeer_coeficient, double distance_between_front_and_rear_axles)
{	// See Torc, "ByWire XGVTM User Manual", page 42
	double curvature;

	curvature = tan(phi / (1.0 + v * v * understeer_coeficient)) / distance_between_front_and_rear_axles;

	return (curvature);
}

int main () {

	astar::VehicleModel vehicle;

	// hard coded

	vehicle.low_speed = 2.2352;

	// default turn radius, it
	// vehicle.min_turn_radius = 5.0;

	// the maximum wheel deflection
	vehicle.max_wheel_deflection = 0.48;

	// the desired steering command rate
	vehicle.steering_command_rate = 0.255;

	// the sensitivity to the understeer dynamic
	vehicle.understeer = 0.0015;

	// default turn radius, it
	vehicle.max_curvature = 0.2;

	// the max velocity
	vehicle.max_velocity = 6.94;

	// max allowed forward speed
	vehicle.max_forward_speed = 46.0;

	// max allowed backward speed
	vehicle.max_backward_speed = 20.0;

	// the car length dimension
	vehicle.length = 4.425;

	// the car width dimension
	vehicle.width = 1.6;

	vehicle.width_2 = 0.8;

	// the car safety facor
	vehicle.safety_factor = 1.0;

	// the axle distance
	vehicle.axledist = 2.625;

	// distance between rear wheels
	vehicle.rear_wheels_dist = 1.535;

	// the distance between rear car and rear wheels
	vehicle.rear_car_wheels_dist = 0.96;

	// the distance between front car and front wheels
	vehicle.front_car_wheels_dist = 0.85;

	// the maximum forward acceleration
	vehicle.max_forward_acceleration = 1.2;

	// the maximum forward deceleration
	vehicle.max_forward_deceleration = 2.7;

	// the maximum backward acceleration
	vehicle.max_backward_acceleration = 1.2;

	// the maximum backward deceleration
	vehicle.max_backward_deceleration = 2.7;

	// the desired forward acceleration
	vehicle.desired_forward_acceleration = 1.2;

	// the desired forward deceleration
	vehicle.desired_forward_deceleration = 2.7;

	// the desired backward acceleration
	vehicle.desired_backward_acceleration = 1.2;

	// the desired backward deceleration
	vehicle.desired_backward_deceleration = 2.7;

	// the desired deceleration
	vehicle.desired_deceleration = 2.7;

	// the max lateral acceleration
	vehicle.max_lateral_acceleration = 1.2;

	// the left steer
	astar::Steer steer = astar::RSTurnLeft;

	// the forward gear
	astar::Gear g = astar::ForwardGear;

	double phi = vehicle.max_wheel_deflection;
	double r = 1.0/get_curvature_from_phi(phi, vehicle.low_speed, vehicle.understeer, vehicle.axledist);
	double delta_t = 1.0/vehicle.low_speed;

	// a sample Pose:
	astar::Pose2D pose(874.5311, -512.445, 0.43);

	std::cout << "The pose: " << pose.position.x << ", " << pose.position.y << ", " << pose.orientation << "\n";
	astar::Pose2D next_pose(vehicle.NextPose(pose, steer, g, 1.0, r));
	std::cout << "The next pose based on steering and gear " << next_pose.position.x << ", " << next_pose.position.y << ", " << next_pose.orientation << "\n";

	std::cout << "The diff: " << pose.position.Distance(next_pose.position) << " and r: " << r << "\n";
	//
	std::cout << "\nDone!\n";

	// test the vehicle model configuration
	vehicle.Configure();

	std::cout << "The pose: " << pose.position.x << ", " << pose.position.y << ", " << pose.orientation << "\n";
	next_pose = vehicle.NextPose(pose, steer, g, 1.0, vehicle.min_turn_radius);
	std::cout << "The next pose steer, gear and default radius after Configure(): " << next_pose.position.x << ", " << next_pose.position.y << ", " << next_pose.orientation << "\n";

	std::cout << "The diff: " << pose.position.Distance(next_pose.position) << " and r: " << r << "\n";
	//
	std::cout << "\nDone!\n";

	std::cout << "The pose: " << pose.position.x << ", " << pose.position.y << ", " << pose.orientation << "\n";
	next_pose = vehicle.NextPose(pose, steer, g, 1.0);
	std::cout << "The next pose: " << next_pose.position.x << ", " << next_pose.position.y << ", " << next_pose.orientation << "\n";

	std::cout << "The diff: " << pose.position.Distance(next_pose.position) << " and r: " << r << "\n";
	//
	std::cout << "\nDone!\n";

	std::cout << "The pose: " << pose.position.x << ", " << pose.position.y << ", " << pose.orientation << "\n";
	next_pose = vehicle.NextPose(pose, vehicle.low_speed, phi, delta_t);
	std::cout << "The next pose: " << next_pose.position.x << ", " << next_pose.position.y << ", " << next_pose.orientation << "\n";

	std::cout << "The diff: " << pose.position.Distance(next_pose.position) << "\n";
	//
	std::cout << "\nDone!\n";

	// now the old fashioned way
	double move_x = vehicle.low_speed * delta_t * std::cos(pose.orientation);
	double move_y = vehicle.low_speed * delta_t * std::sin(pose.orientation);

	std::cout << "The pose: " << pose.position.x << ", " << pose.position.y << ", " << pose.orientation << "\n";
	next_pose.position.x  = pose.position.x +  move_x;
	next_pose.position.y  = pose.position.y +  move_y;
	next_pose.orientation = pose.orientation + vehicle.low_speed * delta_t * std::tan(phi) / vehicle.axledist;
	std::cout << "\nDone!\n";
	std::cout << "The next pose: " << next_pose.position.x << ", " << next_pose.position.y << ", " << next_pose.orientation << "\n";
	std::cout << "The diff: " << pose.position.Distance(next_pose.position) << "\n";

	return 0;
}
