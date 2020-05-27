#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

//declaring the lanes - the ego car lane currently into is lane =1 
// this is done for lane switching
int lane = 1;

// ref_speed is set to 0 to avoid extreme accelerations and jerks in the start
double ref_vel = 00.00;//mph

int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }


    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
        &map_waypoints_dx, &map_waypoints_dy]
        (uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
            uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2') {

                    auto s = hasData(data);

                    if (s != "") {
                        auto j = json::parse(s);

                        string event = j[0].get<string>();

                        if (event == "telemetry") {
                            // j[1] is the data JSON object

                            // Main car's localization Data
                            double car_x = j[1]["x"];
                            double car_y = j[1]["y"];
                            double car_s = j[1]["s"];
                            double car_d = j[1]["d"];
                            double car_yaw = j[1]["yaw"];
                            double car_speed = j[1]["speed"];

                            // Previous path data given to the Planner
                            auto previous_path_x = j[1]["previous_path_x"];
                            auto previous_path_y = j[1]["previous_path_y"];
                            // Previous path's end s and d values 
                            double end_path_s = j[1]["end_path_s"];
                            double end_path_d = j[1]["end_path_d"];

                            // Sensor Fusion Data, a list of all other cars on the same side 
                            //   of the road.
                            auto sensor_fusion = j[1]["sensor_fusion"];
                          
                          
                            //TODO CODE


                            //we will be considering the list of previous path points, so that we can have a smooth transition
                            //rememeber it will be same for x and y size
                            int prev_size = previous_path_x.size();


                            //take the previous s coordinate value of our ego car
                            if (prev_size > 0)
                            {
                                car_s = end_path_s;
                            }
                          
//--------------------------------------------------------------------------------------------------------------   
                          //To avoid collisons and do lane changes without any issues
                             //1. we need to consider sensor fusion data
                             //2. sensor fusion data has all the required information of the vehicles in ego car visible environement
                          
                             //3. sesnor fusion data format for each car
                             //["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates]
                     
                               //4. If there is car in front then out ego car has 3 options
                              //option 1 Try to move into left lane - if it is open, as it will be the fastest one
                              //option 2 If left is not open then try to move into right lane - if it is open.
                              //option 3 If no lanes are open just reduce your speed and keep going in the same lane.
                          
                          		// consider 3 flags
                          	        bool too_close = false;//to check if there is a car in path of our ego lane
                                    	bool left_open = true;// to check if the left lane is open
                                    	bool right_open = true;// to check if right lane is open


                                    // go through all the sensor fusion data - all car that are detected by sensors
                                    for(int i=0; i<sensor_fusion.size(); i++)
                                    {
                                      
                                         //first we will check if there is any car in ego car lane
                                		// take the d value of the ith car
                                		float d = sensor_fusion[i][6];

                                		//take the velocity of the car and calculate its speed
                                		double vx = sensor_fusion[i][3];
                                		double vy = sensor_fusion[i][4];
                                        	// calculate speed
                                		double check_speed = sqrt(vx*vx + vy*vy);
                                		//also check how far the car is by taking s coordinate into consideration
                                		double check_car_s = sensor_fusion[i][5];                                          
                                                                     
                                        // get the left lane - to our ego car
                                        int left_lane = lane - 1;
                                      
                                       // get the right lane- to our ego car
                                        int right_lane = lane + 1;
                                      
                                        check_car_s += double(prev_size) * 0.02 * check_speed;

                                        // check if the car is in the same lane as our ego car
                                        if( d < (2+4*lane+2) && d > (2+4*lane-2) )
                                        {
                                            //check if the car is in front of our ego car and is within 30 m in front of ego car
                                            if( (check_car_s > car_s) && (check_car_s - car_s) < 30 )
                                            {
                                              //if yes then flag turn true
                                                too_close = true;
                                            }
                                        }


                                      //Left Lane move
                                      //check 3 things first - 
                                      //1. if the lane is open and 
                                      //2. if the left lane >= 0 is not on the other side of yellow lanes
                                      //3. and if the car is in left lane
                                        if( left_open &&  left_lane >= 0 && ( d < (2+4*left_lane+2) && d > (2+4*left_lane-2) )  )
                                        { 
                                          //we need to make sure there are no cars in front or behind in the future lane of our ego carin 30m range front as well as behind

                                            double front_behind = abs(check_car_s - car_s); 
                                            if (front_behind < 30)
                                            {
                                              // if there is a car then do not move
                                                left_open = false;
                                            }
                                        }
                                      
				      //Right Lane move
                                      //check 3 things first - 
                                      //1. if the lane is open and 
                                      //2. if the right lane >= 0 is not on the other side of yellow lanes
                                      //3. and if the car is in right lane
                                        if( right_open && right_lane >= 0 && ( d < (2+4*right_lane+2) && d > (2+4*right_lane-2) )  )
                                        {
                                          //we need to make sure there are no cars in front or behind in the future lane of our ego carin 30m 												range front as well as behind

                                            double front_behind = abs(check_car_s - car_s); 
                                            if (front_behind < 30)
                                            {
                                              // if there is a car then do not move
                                                right_open = false;
                                            }
                                        }
                                    }
                          
                               //Implement our logic here
                          	  //If there is car in front then out ego car has 3 options
                              //option 1 Try to move into left lane - if it is open, as it will be the fastest one
                              //option 2 If left is not open then try to move into right lane - if it is open.
                              //option 3 If no lanes are open just reduce your speed and keep going in the same lane.
                          
                                    if(too_close) 
                                    {

                                        if (lane >0 && left_open)//check if the current lane > 0 and left lane is good to move
                                        {
                                            lane = lane - 1;// ego car lane is the on left now

                                        } else if (lane < 2 && right_open)//check if the current lane < 2 and right lane is good to move
                                        {
                                            lane = lane + 1;// ego car lane is the on right now

                                        } else
                                        { 
                                            ref_vel -= 0.224; // reduce vel at a rate of 5m/s2
                                        }
                                      
                                    }

                                    //increase the speed if open road - speed limit is set to 49.5 mph
                                    else if(ref_vel < 49.5) 
                                        ref_vel += 0.224;
//-------------------------------------------------------------------------------------------------------------------------------//
                            /**
                             * TODO: define a path made up of (x,y) points that the car will visit
                             *   sequentially every .02 seconds
                             */
                          
                          	//To generator trajectory our ego car should follow for driving on the Highway-
                          	//Inside data/highway_map.csv there is a list of waypoints that go all the way around the track. 
				//The waypoints are in the middle of the double-yellow dividing line in the center of the highway.


                            // The basic idea here is we need to create widely spaced points (x,y) waypoints far spaced, 
				// at around 30m. 60m and 90m
                            // Then use spline tool to fill in the points inside those widely spaced waypoints. 
                          	// Another option here is to use the polynomial function that was taught in class.
                          
                          	//1. Create vector to store the waypoints ptsx and ptsy

                            vector<double> ptsx;
                            vector<double> ptsy;

                            // 2. Take the car_reference x, y, yaw status
                          	//This will be referenceing where the car is at or the previous path points
                            //coming form localization data
                            double ref_x = car_x;
                            double ref_y = car_y;
                            double ref_yaw = deg2rad(car_yaw);
                          
                          
                          	//3. Tkaing advantage of previous path points. There are 2 things that could happen here-
                          	// option 1 : the previous points could be empty
                          	// option 2 : there could be points left from previous path


                            // option 1 - if previous size is almost empty, use the ego car current state as starting reference
                          	//the reference will remain the same as the previously defined ones ref_x,ref_y, ref_yaw
                            if (prev_size < 2)
                            {                           
                                 //creating a path that is tangent to the angle of the car

                                //generate x and y previous points depending on the angle of the car 
                                //that make the path tangent to the car
                                double prev_car_x = car_x - cos(car_yaw);  
                                double prev_car_y = car_y - sin(car_yaw);
                              
                                //push all the 2 of each coordinates points in the created vector
                                ptsx.push_back(prev_car_x);
                                ptsx.push_back(car_x);
                                ptsy.push_back(prev_car_y);
                                ptsy.push_back(car_y);
                            }
                            // option 2 - if we have the previous path's end points, then use them as starting reference
                          	//the reference will change per the previous point new ref_x, new ref_y, new ref_yaw
                            else
                            {
                              
                              //creating a path that is tangent to the car
                              //for this we will be using the last 2 points
                              // the reference x,y and angle now has changed to the previous point

                                ref_x = previous_path_x[prev_size - 1];
                                ref_y = previous_path_y[prev_size - 1];

                                double ref_x_prev = previous_path_x[prev_size - 2];
                                double ref_y_prev = previous_path_y[prev_size - 2];
                                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                                //push all the 2 of each coordinates points in the created vector
                                ptsx.push_back(ref_x_prev);
                                ptsx.push_back(ref_x);

                                ptsy.push_back(ref_y_prev);
                                ptsy.push_back(ref_y);
                            }

                            // 3. In frenet and to use spline library and to have plan the path we are adding evenly 30m, 60m, 90m spaced points ahead of the starting reference
                            vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                            vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                            vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                          
			   //4. push the x coordinates into the create vectors pntsx 
                            ptsx.push_back(next_wp0[0]);
                            ptsx.push_back(next_wp1[0]);
                            ptsx.push_back(next_wp2[0]);
				//4. push the y coordinates into the create vectors pntsy
                            ptsy.push_back(next_wp0[1]);
                            ptsy.push_back(next_wp1[1]);
                            ptsy.push_back(next_wp2[1]);

                            // 5. Now change all the points in the pntsx and pntsy vector form from global coordinate to our ego carcoordinates
                            for (int i = 0; i < ptsx.size(); i++)
                            {
                                // shifting the points so that the last previous point is at x,y 0,0 
                                double shift_x = ptsx[i] - ref_x;
                                double shift_y = ptsy[i] - ref_y;
                              
			// rotating the points so that the last previous point is at 0 degree angle
                                ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                                ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                            }


                            // 6. create a spline -this is used for the craeting the trajectory
                            tk::spline s;

                            // 7. set(x,y) points to the spline- the vector of pnts having 5 points each right now in pntsx and pntsy
                            s.set_points(ptsx, ptsy);

                            // 8. Now we will be filling in the weidely spaced waypoints at 30,60 and 90m and for saving those points we create two vectors as shown below - This are the points define the actual (x,y) points we will use for the planner 
                          		// the car will visit sequentially every .02 seconds
                            vector<double> next_x_vals;
                            vector<double> next_y_vals;


                            // 9. First we need to add all the remaining previous path points from last time this is for smooth transition from last run to new run- so the first few points in the path planning trajectory will be the previous path points. 
                            for (int i = 0; i < previous_path_x.size(); i++)
                            {
                                next_x_vals.push_back(previous_path_x[i]);
                                next_y_vals.push_back(previous_path_y[i]);
                            }

                            // 10. Now we need to space the points in the spline so that we travel at our desired speed 
                          	// Now we need to get the points evenly spaced between start to the 30m in future
                            double target_x = 30; 
                           // spline gives the y axis value for the x axis taht we provide.
                            double target_y = s(target_x);
                           // we can calculate the total distance 
                            double target_dist = sqrt(target_x * target_x + target_y * target_y);
                          	// the first point is at 0,0 and 0 degree angle as we did during transformation of coordinates into ego car
                          	// to keep track of new points
                            double x_add_on = 0;
                          	// no. of points N will be (d = s * N * time for moving to each point)
                            double N = target_dist / (0.02 * ref_vel / 2.24);  // divide by 2.24 to transfer from miles/h to meters/s
                          	// we can get the x spacing of each point
                            double x_step = target_x / N;
                          
                           	// 11. Now we will fill in the remaining points using spline in 30 m distance 
                          
							// The defined total number of points that we wnat is 50, out of 50 we have fed in the previous points into the vector for smooth transition
                            for (int i = 1; i <= 50 - previous_path_x.size(); i++)
                            { 
								// All the next points will be added after the 0,0 points and each point is placed x_step ahead of the last point
                                double x_point = x_add_on + x_step;
                              	// getting the y from spline library 
                                double y_point = s(x_point);
								// new last point will be recently added point
                                x_add_on = x_point;
								
                              	// transforming back to the global coordinates form car coordinates
                                double x_ref = x_point;
                                double y_ref = y_point;

                                // rotate back to world coordinate 
                                x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                                y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
								// shift back to its locaton in global coordinates
                                x_point += ref_x;
                                y_point += ref_y;
								// adding the new point to the new_x and new_y created list
                                next_x_vals.push_back(x_point);
                                next_y_vals.push_back(y_point);
                            }

                            json msgJson;

							//sending the points vector (planned trajectory) which the ego car should follow to the simulator
                            msgJson["next_x"] = next_x_vals;
                            msgJson["next_y"] = next_y_vals;

                            auto msg = "42[\"control\"," + msgJson.dump() + "]";

                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }  // end "telemetry" if
                    }
                    else {
                        // Manual driving
                        std::string msg = "42[\"manual\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }  // end websocket if
        }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
        });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char* message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    }
    else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}
