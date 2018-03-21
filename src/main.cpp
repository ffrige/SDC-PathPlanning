#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double LANE_WIDTH = 4;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

//define control point type
typedef struct Point_Type {
	double x;
	double y;
} Point_Type;


//evaluate the Bezier curve defined by control points CtrlPoints at time t [0..1]
Point_Type EvaluateBezier(const vector<Point_Type> CtrlPoints, double t)
{
	int k,i;
	int n = 3; //order of spline (3=cubic; 4=quartic...)

	if(t<=0) return CtrlPoints[0];
	if(t>=1) return CtrlPoints[n];
	
	vector<Point_Type> Point;	
	for(i=0;i<=n;i++) Point.push_back(CtrlPoints[i]);
	
	for (k=1; k<=n; k++) {
		for (i=0; i<=n-k; i++) {
			Point[i].x = (1.0-t)*Point[i].x + t*Point[i+1].x;
			Point[i].y = (1.0-t)*Point[i].y + t*Point[i+1].y;
		}
	}
	
	return Point[0];
}


//calculate (approximate) length of Bezier curve
double BezierLength(const vector<Point_Type> CtrlPoints)
{
	vector<Point_Type> Point;
	int i;
	int k = 30; //increase number of points for more accuracy
	double Length = 0;
	
	Point.push_back(CtrlPoints[0]);
	Point.push_back(CtrlPoints[0]);
	
	//evaluate the BezierCurve at different points and then add the linear distances between them
	for(i=1;i<=k;i++) {
		Point[1] = EvaluateBezier(CtrlPoints,(double)(i)/(double)(k));
		double dx = Point[1].x - Point[0].x;
		double dy = Point[1].y - Point[0].y;
		Length += sqrt(dx*dx+dy*dy);
		Point[0] = Point[1];
	}

	return Length;
}


int main() {
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

	//start in lane 1
	int lane = 1;
	
	//start with zero speed
	double ref_vel = 0;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

						int prev_size = previous_path_x.size();
						double ref_x = car_x;
						double ref_y = car_y;
						double ref_yaw = deg2rad(car_yaw);

						if (prev_size > 1) {
							car_s = end_path_s;
							car_d = end_path_d;
							ref_x = previous_path_x[prev_size-1];
							ref_y = previous_path_y[prev_size-1];
							double prev_ref_x = previous_path_x[prev_size-2];
							double prev_ref_y = previous_path_y[prev_size-2];
							ref_yaw = atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);
						}
												
						bool too_close = false; //true if collision approaching ahead
						bool left_free = true; //false if left lane is occupied
						bool right_free = true; //false if right lane is occupied
						
						double max_speed = 0;
						double target_distance = 30.0; //meters
						
						//detect other cars
						for (int i=0;i<sensor_fusion.size();i++) {
						
							//check lane of each car
							float d = sensor_fusion[i][6];
							
							//check if our lane is free
							if (d<2+LANE_WIDTH*lane+2 && d>2+LANE_WIDTH*lane-2) {
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_car_speed = sqrt(vx*vx+vy*vy);
								double check_car_s = sensor_fusion[i][5];
								
								//predict future position of this car
								check_car_s += ((double)prev_size*check_car_speed*0.02);
								
								if (check_car_s > car_s && check_car_s < car_s+target_distance && check_car_speed < car_speed) {
									//car will collide -> slow down!
									too_close = true;
									max_speed = check_car_speed;
								}
							}
							
							//check if left lane is free
							if (d<2+LANE_WIDTH*(lane-1)+2 && d>2+LANE_WIDTH*(lane-1)-2) {
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_car_speed = sqrt(vx*vx+vy*vy);
								double check_car_s = sensor_fusion[i][5];
								
								//predict future position of this car
								check_car_s += ((double)prev_size*check_car_speed*0.02);
								
								if (check_car_s > car_s-target_distance && check_car_s < car_s+target_distance) {
									left_free = false;
								}
							}
							
							//check if right lane is free
							if (d<2+LANE_WIDTH*(lane+1)+2 && d>2+LANE_WIDTH*(lane+1)-2) {
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_car_speed = sqrt(vx*vx+vy*vy);
								double check_car_s = sensor_fusion[i][5];
								
								//predict future position of this car
								check_car_s += ((double)prev_size*check_car_speed*0.02);
								
								if (check_car_s > car_s-target_distance && check_car_s < car_s+target_distance) {
									right_free = false;
								}
							}
							
						
						}
						
						//main control logic
						if (too_close) { //collision ahead
							//look for lane change or slow down
							if (right_free && lane<2) lane+=1;
							else if (left_free && lane>0) lane-=1;
							else if (ref_vel > max_speed) ref_vel -= 0.5;
						}	else { //no collision ahead
							//speed up to limit
							if (ref_vel < 22) { //22m/s is a bit less than 50mph
								ref_vel += 0.5;
							}
							//try to stay in middle lane
							if (lane>1 && left_free) lane-=1;
							if (lane<1 && right_free) lane+=1;
						}

						
						//we create a cubic Bezier curve, starting from last planned position of car to future position after target_distance
						vector<Point_Type> CtrlPoints;
						Point_Type tmpPoint;
						vector<double> tmp_xy;						
						
						//first control point is last planned position of car (unless no planned data is available)
						tmpPoint.x = ref_x;
						tmpPoint.y = ref_y;
						CtrlPoints.push_back(tmpPoint);
						
						//second control point is at 1/3 of target_distance along current tangent
						tmpPoint.x = CtrlPoints[0].x + target_distance * 0.333 * cos(ref_yaw);
						tmpPoint.y = CtrlPoints[0].y + target_distance * 0.333 * sin(ref_yaw);						
						CtrlPoints.push_back(tmpPoint);
						
						//third control point is at 2/3 of target_distance along target tangent
						tmp_xy = getXY(car_s+target_distance*0.667,2+lane*LANE_WIDTH,map_waypoints_s,map_waypoints_x, map_waypoints_y);						
						tmpPoint.x = tmp_xy[0];
						tmpPoint.y = tmp_xy[1];
						CtrlPoints.push_back(tmpPoint);

						//fourth control point is at target_distance ahead in target lane
						tmp_xy = getXY(car_s+target_distance,2+lane*LANE_WIDTH,map_waypoints_s,map_waypoints_x, map_waypoints_y);						
						tmpPoint.x = tmp_xy[0];
						tmpPoint.y = tmp_xy[1];
						CtrlPoints.push_back(tmpPoint);
						
						//calculate length of spline
						double Path_Length = BezierLength(CtrlPoints);
						
						//calculate time that takes to run spline at given speed
						double Path_Time = Path_Length / (ref_vel*0.9); //reduced because path_length is underestimated						
						
          	//now define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

						//these are the actual points that will be passed to the simulator in X,Y coordinates
						vector<double> next_x_vals;
						vector<double> next_y_vals;
						
						
						//start with all the previously planned points (if still available)
						for (int i=0;i<prev_size;i++) {
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);						
						}
			

						//add the rest from the spline
						double t=0;
						double dt = 0.02/Path_Time; //interval between consecutive points
						for (int i=0;i<25-prev_size;i++) {

							t += dt;
							tmpPoint = EvaluateBezier(CtrlPoints,t);

							next_x_vals.push_back(tmpPoint.x);
							next_y_vals.push_back(tmpPoint.y);

						}

						
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
