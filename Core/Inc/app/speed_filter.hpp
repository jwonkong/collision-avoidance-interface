/*
 * speed_filter.hpp
 *
 *  Created on: Mar 27, 2024
 *      Author: jiwon.kong
 */
#ifndef INC_SPEED_FILTER_HPP_
#define INC_SPEED_FILTER_HPP_

#include <vector>
#include <algorithm>

struct Point {
    float x;
    float y;
};

struct Node { // T-S diagram tree node
    float x = 0; // x position
    float y = 0; // y position
    float v = 0; // velocity
    float a = 0; // acceleration
    Node* parent = nullptr; // parent node
};

struct ObjectInfo { // From camera CAN
    int classification;
    double relative_X_position;
    double relative_Y_position;
    double relative_X_velocity;
    double relative_Y_velocity;
    float size[2];
};

struct Coordinate {
    float x; // x position
    float y; // y position
    float m; // slope
    float r; // radius
};

namespace speed_filter {

//* ego path predictor
#define wheel_base 2.97 // [m]
#define track_width 5 // [m]
#define distance_from_cg_to_front_axle 1.47 // [m]
#define time_step 0.02 // [s]
#define cornering_stiffness_front 100000
#define cornering_stiffness_rear 80000
#define yaw_inertia 3954.288 // [kg m^2]
#define vehicle_mass 2108 // [kg]
#define steering_ratio 20 // : 1

//* command_selector
#define distance_object_to_line 4 // [m]
#define velocity_limit 22 // [m/s]

class CollisionAvoidanceFilter {

public:
	CollisionAvoidanceFilter(double vehicle_speed,
							 double vehicle_steering_angle,
							 double vehicle_longitudinal_acceleration,
							 double target_acceleration,
							 std::vector<ObjectInfo>& objectInfos);

	~CollisionAvoidanceFilter() {};

public:
	void ego_path_predictor();
	void object_path_predictor();
	void commend_selector();
	void run(); // Run above three function
	double getOutput(); // return output acceleration

private:
	//* v_can
	double input_speed;
	double input_steering_angle;
	double input_longitudinal_acceleration;

	//* ad_can
	double input_target_acceleration;
	double output_target_acceleration;

	//* camera
	std::vector<ObjectInfo> object_infos;
	float front_object_station;
	float front_object_velocity;

	//* T-S graph
	std::vector<Coordinate> objectList_coord;

	//* my_path
	std::vector<Point> predicted_my_path;

	class PathFinder {
	public:
	    std::vector<Node*> nodeList;
	    std::vector<std::pair<float, float>> objectList;
	    Node* ptNode = nullptr;
	    bool pathfound;

	    PathFinder();
	    ~PathFinder();

	    void Sampling(Node* sample,Node*& parent, float accel);
	    bool collision(Node* sample, Node*& parent);
	    float Filter(float cmd_acceleration, float current_acceleration, float velocity);
	    float findPath(float start_x, float start_y, float start_v, float start_a);
	    void AddLinearFunctionPoints(float a, float b, float m, float verticalHeight, float xEnd, float gap1, float gap2);
	};

	class BicycleModel {
	public:
		float x;  // X position [m]
		float y;  // Y position [m]
		float yaw;  // Yaw angle [rad]
		float vx;  // Longitudinal velocity [m/s]
		float vy;  // Lateral velocity [m/s]
		float omega;  // Yaw rate [rad/s]

		float L, Lf, dt, Cf, Cr, Iz, m, W, S;

		BicycleModel() : x(0.0), y(0.0), yaw(0.0), vx(0.0), vy(0.0), omega(0.0) {}

		float normalize_angle(float angle);
		void update_nonlinear(float vx, float delta, float L, float Lf, float dt, float m, float Cf, float Cr, float Iz);
		void update_linear(float vx, float delta, float L, float Lf, float dt, float m, float Cf, float Cr, float Iz);
	};

	void ClassificationToSize(ObjectInfo &obj); // Object classification

};

}

#endif /* INC_SPEED_FILTER_HPP_ */
