# **Path Planning Project** 

## The goal of this project are the following:
* This project main goal is to safely navigate our ego car around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
* Utilize Ego car's localization and as well as sensor fusion data if and when needed, there is also a sparse map list of waypoints around the highway.
* Ego car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too.
* Ego car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
* Ego car should be able to make one complete loop around the 6946m highway.
* Ego car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

My github repo for this project : [project code](https://github.com/amundada2310/CarND-Path-Planning-Project)

## The steps taken to achieve all the above acceptance critieria:

### Creating Trajectory or path planning for Ego car:

##### To have a smooth transition from previous path to new path it is important that we take previous data points into account.
  * The previous list of points could be empty, then we use use current ego car state as reference.
  * OR if we have previous points the utilize them as the reference points.
  
  `                         	
                            
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
                            }`

##### The basic idea here is we need to create widely spaced points (x,y) waypoints far spaced, at around 30m, 60m and 90m. Then generate new pointsa and fill the spaces.
`

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
`
 
##### Then Filling in the data point either by utilizing Spline tools library or Polynomial function.
  * I utilized Spline tool library as it is much easier to use and doesn't require alot of coding. 
  * With Spline - it provide much smoother path and makes sure our ego car is following each of the points.
  
##### Before utilizing spline it is much easier to tranform the data points and work with ego car coordinates instead of global coordinates.
  * Shifted the 5 points stored in ptsx and ptsy in such a way that the last previous path point is at 0,0 coordinates.
  * Rotated the 5 points stored in ptsx and ptsy in such a way that the last previous path point is at 0 degree angle.
`

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
 `
##### Now I will be filling in the widely spaced waypoints at 30,60 and 90m .Those are the points which defines the actual (x,y) points defining the path for our ego car. The points our ego car will visit sequentially every .02 seconds
  * For creating those points I used spline library.
  * First I added all the remaining previous path points from last run. This is for smooth transition from last run to new run- so the first few points in the path planning trajectory will be the previous path points. 
  * Then filled in the left over spots with new created points.
  
<img src = "data/the spline explained.PNG" width="700" height="400" />
      
  * Then tranformed the newly created points from ego car coordinates to the global coordinates.
  * Finally pushing the newly transformed points to the simulator.
  `
  
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
`

### Tacking the issue with collisions, lane changes, jerks and max accelerations:

##### The initial speed of car was set to 0.0 MPH and was gradually increased at all the times to avoid the sudden acceleration and jerk.
##### The maximum speed the car could reach was limited to 49.5MPH to avoid speeding.
##### For lane changing and avoid going out of the road or crossing the yellow line, the current lane in which ego car starts was set to 1.
`

                                    //declaring the lanes - the ego car lane currently into is lane =1 
                                    // this is done for lane switching
                                    int lane = 1;

                                    // ref_speed is set to 0 to avoid extreme accelerations and jerks in the start
                                    double ref_vel = 00.00;//mph
`
`

                                    //increase the speed if open road - speed limit is set to 49.5 mph
                                    else if(ref_vel < 49.5) 
                                        ref_vel += 0.224;
`
##### If there is car in front of the ego car within 30 m of distance. Then ego car has 3 options-
* option 1 Try to move to the left lane - First check if it is valid move and if it is open. This is our first choice as it the fastest lane.
* option 2 Try to move to the right lane - First check if it is valid move and if it is open. Only perform if left lane is not availabe.
* option 3 Slow down - If no lanes are open just reduce ego cars speed and keep going in the same lane. 

`

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
`

## Additional Information
##### My code was able to perform as expected, and the car was able to drive through the Highway without any issues.

* No initial jerks or maximum acceleration issues
<img src = "output photos/no initial jerks or max acceleration issues.PNG" width="700" height="400" />

* Maintaining Speed limit
<img src = "output photos/speed limit.PNG" width="700" height="400" />

* Avoid lane changing during heavy traffic and slowing down after detecting car in ego cars lane within 30m of distance in front.
<img src = "output photos/avoid lane change in traffic and slowing down.PNG" width="700" height="400" />

* Left lane Change
<img src = "output photos/left lane changing.PNG" width="700" height="400" />

* Right lane Change
<img src = "output photos/right lane changing.PNG" width="700" height="400" />
