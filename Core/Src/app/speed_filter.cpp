/*
 * speed_filter.cpp
 *
 *  Created on: 2024. 3. 27.
 *      Author: jiwon.kong
 */
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include "app/speed_filter.hpp"

using namespace speed_filter;
using namespace std;

CollisionAvoidanceFilter::CollisionAvoidanceFilter(double vehicle_speed,
                                                   double vehicle_steering_angle,
                                                   double vehicle_longitudinal_acceleration,
                                                   double target_acceleration,
												   std::vector<ObjectInfo>& objectInfos)
    : input_speed(vehicle_speed),
      input_steering_angle(vehicle_steering_angle),
      input_longitudinal_acceleration(vehicle_longitudinal_acceleration),
      input_target_acceleration(target_acceleration),
	  object_infos(objectInfos) {}

void CollisionAvoidanceFilter::run() {
	ego_path_predictor();
	object_path_predictor();
	commend_selector();
}

double CollisionAvoidanceFilter::getOutput() {
	return output_target_acceleration;
}

void CollisionAvoidanceFilter::ego_path_predictor() {
	BicycleModel vehicle;

	float L = wheel_base;
	float Lf = distance_from_cg_to_front_axle;
	float dt = time_step;
	float m = vehicle_mass;
	float Cf = cornering_stiffness_front;
	float Cr = cornering_stiffness_rear;
	float Iz = yaw_inertia;
	float S = steering_ratio;

	int N = static_cast<int>((input_speed / 5 + 3) / dt);
	if (input_speed < 6) N = static_cast<int>(8 / dt);

	predicted_my_path.clear();

	Point point;
	point.x = 0.0;
	point.y = 0.0;
	predicted_my_path.push_back(point); // Push back (0, 0)

	// Apply bicycle model
	if (input_speed < 6) { // slow situation -> linear
		for (int i = 0; i < N; ++i) {
			vehicle.update_linear(input_speed, input_steering_angle*(M_PI/180)/S, L, Lf, dt, m, Cf, Cr, Iz);
			Point point;
			point.x = vehicle.x;
			point.y = vehicle.y;
			if (point.x < predicted_my_path.back().x) break;
			predicted_my_path.push_back(point);
		}
	}

	else { // fast situation -> non-linear
		for (int i = 0; i < N; ++i) {
			vehicle.update_nonlinear(input_speed, input_steering_angle*(M_PI/180)/S, L, Lf, dt, m, Cf, Cr, Iz);
			Point point;
			point.x = vehicle.x;
			point.y = vehicle.y;
			if (point.x < predicted_my_path.back().x) break;
			predicted_my_path.push_back(point);
		}
	}
}

void CollisionAvoidanceFilter::object_path_predictor() {
    float W = track_width;
    objectList_coord.clear();
    front_object_station = 0;

    for (auto& object : object_infos) {
        // Front object
        if (object.relative_X_position > 0) {
            // absolute speed
        	float object_vx = object.relative_X_velocity + input_speed;
            float object_vy = object.relative_Y_velocity;
            // Classification operation
            CollisionAvoidanceFilter::ClassificationToSize(object);

            // Estimate the object's 3-second movement in a constant-speed model and express it as a rectangle
            Point center_point;
            center_point.x = object.relative_X_position + (3/2)*object_vx + object.size[0]/2;
            center_point.y = object.relative_Y_position + (3/2)*object_vy;
            float theta = atan2(object_vy, object_vx);
            float v = sqrt(object_vx*object_vx + object_vy*object_vy);
            float rectHalfHeight = (object.size[0] / 2) + (3/2)*v;
            float rectHalfWidth = object.size[1] / 2;

            Point rotatedCenter;
            for (size_t i = 1; i < predicted_my_path.size(); ++i) {
                // Calculate object elements and lengths through rotational transformations of the predicted_my_path point
                rotatedCenter.x = cos(theta) * (predicted_my_path[i].x - center_point.x) + sin(theta) * (predicted_my_path[i].y - center_point.y);
                rotatedCenter.y = -sin(theta) * (predicted_my_path[i].x - center_point.x) + cos(theta) * (predicted_my_path[i].y - center_point.y);
                float distX = fabs(rotatedCenter.x);
                float distY = fabs(rotatedCenter.y);

                // collision detection
                if (distY <= (rectHalfWidth + W/2) && distX <= (rectHalfHeight + W/2)) {
                    // Object speed in the direction of my path internally
                    float dx = predicted_my_path[i].x - predicted_my_path[i-1].x; // collision x point
                    float dy = predicted_my_path[i].y - predicted_my_path[i-1].y; // collision y point
                    float length = sqrt(dx*dx + dy*dy); // length
                    float ux = dx / length; // x velocity unit vector
                    float uy = dy / length; // y velocity unit vector

                    // When there's a current obstacle on my path
                    if (-rectHalfHeight + object.size[0] + W/2 >= rotatedCenter.x) {
                        float s = sqrt(pow(predicted_my_path[i].x, 2) + pow(predicted_my_path[i].y, 2));
                        if (front_object_station > 0 && s > front_object_station) break;
                        front_object_station = s;
                        front_object_velocity = object_vx * ux + object_vy * uy;
                        break;

                    }
                    // When there are obstacles in the future on my path
                    else {
                        Coordinate coord;
                        float t = ((3/2)*v + rotatedCenter.x - object.size[0] - W/2) / (3*v);
                        coord.x = 3*t;
                        coord.y = sqrt(pow(predicted_my_path[i].x, 2) + pow(predicted_my_path[i].y, 2));
                        coord.m = object_vx * ux + object_vy * uy;
                        coord.r = sqrt(pow(object.size[0], 2) + pow(object.size[1], 2))/2;
                        objectList_coord.push_back(coord);
                        break;
                    }
                }
            }
        }
    }
}

void CollisionAvoidanceFilter::commend_selector() {
	PathFinder pathFinder;

	if (front_object_station > 0) { // present collision element
		pathFinder.AddLinearFunctionPoints(0,
										   front_object_station,
										   front_object_velocity,
										   1,
										   input_speed/5 + 3,
										   2*distance_object_to_line/input_speed,
										   2*distance_object_to_line);
	}

	for (unsigned int i = 0 ; i < objectList_coord.size() ; ++i) { // future collision element
		pathFinder.AddLinearFunctionPoints(objectList_coord[i].x,
										   objectList_coord[i].y,
										   objectList_coord[i].m,
										   2*objectList_coord[i].r,
										   3,
										   2*distance_object_to_line/input_speed,
										   2*distance_object_to_line);
	}

	output_target_acceleration = pathFinder.Filter(input_target_acceleration, input_longitudinal_acceleration, input_speed);
}

CollisionAvoidanceFilter::PathFinder::PathFinder() : pathfound(false) {}

CollisionAvoidanceFilter::PathFinder::~PathFinder() {
    for (Node* node : nodeList) delete node;
    nodeList.clear();
    objectList.clear();
}

void CollisionAvoidanceFilter::PathFinder::Sampling(Node* sample, Node*& parent, float accel) {
    float delta_t = (parent->v/5) * (-0.1 * accel + 0.5);
    if (delta_t < 0.4) delta_t = 0.4;
    sample->x = parent->x + delta_t;
    sample->y = parent->y + 0.5*(accel)*(delta_t)*(delta_t) + (parent->v)*delta_t;
    sample->v = parent->v + accel*delta_t;
    sample->a = accel;
    sample->parent = parent;
}


bool CollisionAvoidanceFilter::PathFinder::collision(Node* sample, Node*& parent) {
    float tmpDist;
    float tmpY;

    for (auto o : objectList) {
        if (o.first >= parent->x && o.first <= sample->x && o.second > parent->y) {
            tmpY = 0.5*(sample->a)*(o.first-parent->x)*(o.first-parent->x) + (parent->v)*(o.first-parent->x) + parent->y;
            tmpDist = abs(o.second - tmpY);
            if (tmpDist <= distance_object_to_line) {
                delete sample;
                return false;
            }
        }
    }
    return true;
}

float CollisionAvoidanceFilter::PathFinder::Filter(float cmd_acceleration, float current_acceleration, float velocity) {
    float x_new = 1.2;
    float y_new = (0.5*cmd_acceleration)*x_new*x_new + velocity*x_new;
    float v_new = velocity + cmd_acceleration*x_new;
    float Target_A;
    float tmpY;
    float tmpDist;
    bool new_check = true;
    for (auto o : objectList) {
        if (o.first <= x_new) {
            tmpY = 0.5*(cmd_acceleration)*(o.first)*(o.first) + (velocity)*(o.first);
            tmpDist = abs(o.second - tmpY);
            if (tmpDist <= distance_object_to_line) {
                new_check = false;
                break;
            }
        }
    }

    if (!new_check){
        Target_A = min(findPath(0, 0, velocity, min(cmd_acceleration, 0.0f)), 0.0f);
        return Target_A;
    }

    // velocity limit
    else if (velocity > velocity_limit) {
        Target_A = findPath(x_new, y_new, v_new, - 0.5f);
        if (Target_A != -0.5) {
            if (Target_A < -5) {
                // Start at (0, 0)
                Target_A = findPath(0, 0, velocity, min(current_acceleration - 0.5f, -0.5f));
            }
            else {
                Target_A = min(min((Target_A+3*cmd_acceleration)/4, current_acceleration - 0.5f), -0.5f);
            }
            return Target_A;
        }
        return Target_A;
    }

    else {
        Target_A = findPath(x_new, y_new, v_new, cmd_acceleration);
        if (Target_A != cmd_acceleration) {
        	if (Target_A >= 0) {
        		Target_A = Target_A;
        	}

        	else if (Target_A < -5) {
                // Start at (0, 0) consider jerk
                Target_A = findPath(0, 0, velocity, min(current_acceleration - 0.5f, (Target_A + 3*cmd_acceleration)/4));
//            	Target_A = min(min(min((Target_A+3*cmd_acceleration)/4, cmd_acceleration - 0.5f), current_acceleration - 0.5f), 0.0f);
            }

            else {
//                Target_A = min(min(min((Target_A + (x_new/0.4)*cmd_acceleration)/(x_new/0.4 + 1), cmd_acceleration - 0.5f), current_acceleration - 0.5f), 0.0f);
                Target_A = min(min((Target_A + 3*cmd_acceleration)/4, cmd_acceleration - 0.5f), current_acceleration - 0.5f);
            }
            return Target_A;
        }
    }

    return Target_A;
}

float CollisionAvoidanceFilter::PathFinder::findPath(float start_x, float start_y, float start_v, float start_a) {
    Node* head = new Node;
    head->x = start_x;
    head->y = start_y;
    head->v = start_v;
    nodeList.push_back(head);

    float width = start_v/5 + 3;
    float delta_t = 0.4;
//    if (start_x == 0) delta = 0.8;
    float iterator = start_a;
    bool result = false;

    while (pathfound == false)
    {
        Node* parent = nodeList.back();

        for (float i = iterator ; i >= -5 ; i--) {
            ptNode = new Node;
            Sampling(ptNode, parent , i);
            result = collision(ptNode, parent);

            if (result) {
                ptNode->x = ptNode->parent->x + delta_t;
                ptNode->y = 0.5*(i)*delta_t*delta_t + (ptNode->parent->v)*delta_t + ptNode->parent->y;
                ptNode->v = ptNode->parent->v + i*delta_t;
                ptNode->a = i;
                nodeList.push_back(ptNode);
                iterator = 0;

                if (nodeList.back()->x >= width || nodeList.back()->v <= 0) pathfound = true;
                break;
            }

            else if (i < -4) {
                for (int j = nodeList.size() - 1; j >= 0; j--) {
                    if (nodeList.back()->a < -4.5) {
                        delete nodeList.back();
                        nodeList.pop_back();
                    }
                    else {
                        iterator = (nodeList.back()->a) - 1;
                        delete nodeList.back();
                        nodeList.pop_back();
                        break;
                    }
                }
            }
        }
        if (nodeList.empty()) return -5.119;
    }
    return nodeList.at(1)->a;
}


void CollisionAvoidanceFilter::PathFinder::AddLinearFunctionPoints(float a, float b, float m, float verticalHeight, float xEnd, float gap1, float gap2) {
    if (gap1 > 0.2f) gap1 = 0.2f;
    if (a < 0) a = 0;
    for (float i = b; i < b + verticalHeight; i += gap2) {
        for (float x = a; x < xEnd + gap1; x += gap1) {
            float y = m * (x-a) + i;
            objectList.push_back(make_pair(x, y));
        }
    }
}

float CollisionAvoidanceFilter::BicycleModel::normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


void CollisionAvoidanceFilter::BicycleModel::update_nonlinear(float vx, float delta, float L, float Lf, float dt, float m, float Cf, float Cr, float Iz) {
    float Lr = L - Lf;  // Distance from CG to rear axle [m]
    float cos_yaw = cos(yaw);
    float sin_yaw = sin(yaw);

    // Update states
    x += vx * cos_yaw * dt - vy * sin_yaw * dt;
    y += vx * sin_yaw * dt + vy * cos_yaw * dt;
    yaw += omega * dt;
    yaw = normalize_angle(yaw);

    float Ffy = -Cf * atan2((vy + Lf * omega) / vx - delta, 1.0);
    float Fry = -Cr * atan2((vy - Lr * omega) / vx, 1.0);

    // Update velocities
    vx = vx + dt * (Ffy * sin(delta) / m);
    vy = vy + dt * ((Fry + Ffy * cos(delta)) / m - vx * omega);
    omega += dt * (Ffy * Lf * cos(delta) - Fry * Lr) / Iz;
}


void CollisionAvoidanceFilter::BicycleModel::update_linear(float vx, float delta, float L, float Lf, float dt, float m, float Cf, float Cr, float Iz) {
    float Lr = L - Lf;
    float beta = atan2((Lr / L) * tan(delta), 1.0);

    // Update state
    x += vx * cos(yaw+beta) * dt;
    y += vx * sin(yaw+beta) * dt;
    yaw += vx / L * cos(beta) * tan(delta) * dt;
    yaw = normalize_angle(yaw);
}

void CollisionAvoidanceFilter::ClassificationToSize(ObjectInfo &obj) {
    std::unordered_map<int, std::pair<float, float>> sizeMap = {
        {0, {2, 2}},   // Unknown
        {1, {5, 2}},   // Car
        {2, {8, 2.5}}, // Truck
        {3, {8, 2}},   // Motorcycle
        {4, {6, 2.5}}, // Other vehicle
        {5, {1, 1}},   // Pedestrian

    };

    auto it = sizeMap.find(obj.classification);
    if (it != sizeMap.end()) {
        obj.size[0] = it->second.first;
        obj.size[1] = it->second.second;
    } else {
        obj.size[0] = 2;
        obj.size[1] = 2;
    }
}
