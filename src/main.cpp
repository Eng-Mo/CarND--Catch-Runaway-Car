#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "Tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // Create a UKF instance
  UKF ukf;

  double target_x = 0.0;
  double target_y = 0.0;
  Point pt1,pt2,pt3, c, tp;
  c.x=0.0;
  c.y=0.0;
//  tp.x=0.0;
//  tp.y=0.0;
  long long stime=0.0;
  long long time= 0.0;
  double timetoc=0.0;
  double  dt2=0.0;



  double rad=0.0;

  bool onep=false,twop=false,threep=false,detectc=false, detectt=false;
  double heading_to_target =0.0,heading_difference=0.0,distance_difference=0.0;
  bool take=false;



  h.onMessage([&ukf,&target_x,&target_y, &stime,&time,&rad,&detectc,&detectt,&c,&pt1,&pt2,&pt3,
			   &timetoc,&onep,&twop,&threep,&dt2,
			   &heading_to_target,&heading_difference,&distance_difference,&take](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
      	
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
          double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
          double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());
          
          string lidar_measurment = j[1]["lidar_measurement"];
          
          MeasurementPackage meas_package_L;
          istringstream iss_L(lidar_measurment);
    	  long long timestamp_L;

    	  // reads first element from the current line
    	  string sensor_type_L;
    	  iss_L >> sensor_type_L;

      	  // read measurements at this timestamp
      	  meas_package_L.sensor_type_ = MeasurementPackage::LASER;
          meas_package_L.raw_measurements_ = VectorXd(2);
          float px;
      	  float py;
          iss_L >> px;
          iss_L >> py;
          meas_package_L.raw_measurements_ << px, py;
          iss_L >> timestamp_L;
          meas_package_L.timestamp_ = timestamp_L;
          if (stime ==0.0){
                  	  stime= timestamp_L;
          }
          else {time=timestamp_L;}

          
    	  ukf.ProcessMeasurement(meas_package_L);
		 
    	  string radar_measurment = j[1]["radar_measurement"];
          
          MeasurementPackage meas_package_R;
          istringstream iss_R(radar_measurment);
    	  long long timestamp_R;

    	  // reads first element from the current line
    	  string sensor_type_R;
    	  iss_R >> sensor_type_R;

      	  // read measurements at this timestamp
      	  meas_package_R.sensor_type_ = MeasurementPackage::RADAR;
          meas_package_R.raw_measurements_ = VectorXd(3);
          float ro;
      	  float theta;
      	  float ro_dot;
          iss_R >> ro;
          iss_R >> theta;
          iss_R >> ro_dot;
          meas_package_R.raw_measurements_ << ro,theta, ro_dot;
          iss_R >> timestamp_R;
          meas_package_R.timestamp_ = timestamp_R;
          if (stime ==0.0){
        	  stime= timestamp_R;
          }
          else {time=timestamp_R;}
          
    	  ukf.ProcessMeasurement(meas_package_R);

//    	  bool onep=false, twop=false, threep=false, detect=false;



    	  double dt;
    	  dt = (time-stime)/10000000.0;

    	  cout<<"dt= "<<dt<<endl;
    	  Point  tp;
    	  Tools calc;

			tp.x=0.0;
			tp.y=0.0;



//
//
//
    	  if (dt>=0.0&&dt<0.005&&onep==false){

    		  cout<<"excute if 1"<<endl;
    		  timetoc=dt;
    		  pt1.x= ukf.x_[0];
    		  pt1.y= ukf.x_[1];
//    		  onep=true;
    		  cout<< "point one "<< pt1.x<<", "<<pt1.y<<endl;
    		  target_x=0.0;
    		  target_y=0.0;
    		  onep=true;



    	  }
    	  else if (dt>.1&&dt<0.155&&twop==false){
//    		  timetoc=dt;
    		  cout<<"excute if 2"<<endl;
    		  pt2.x= ukf.x_[0];
			  pt2.y= ukf.x_[1];
			  twop=true;
			  cout<< "point two "<< pt2.x<<", "<<pt2.y<<endl;

    	  }

    	  else if (dt>.2&&threep==false&&twop==true){

    		  threep=true;
    		  cout<<"excute if 3"<<endl;
			  pt3.x= ukf.x_[0];
			  pt3.y= ukf.x_[1];
			  c= calc.calcCenter(pt1, pt2, pt3);
			  cout<<"center= "<<c.x<<", "<<c.y<<endl;
			  threep=true;

//
			  target_x= c.x;
			  target_y=c.y;
			  double d= sqrt(pow(c.radiaus,2)+pow(c.radiaus,2));
			  timetoc=(d/ukf.x_[2]);
////			  timetoc+=dt;
			  dt2=dt+timetoc;
//
//
//			  detectc=true;


			 }
    	  if (detectc==false&&threep==true&&hunter_x==0&&hunter_y==0){
    		  c= calc.calcCenter(pt1, pt2, pt3);
			  cout<<"center= "<<c.x<<", "<<c.y<<endl;
			  threep=true;

 //
			  target_x= c.x;
			  target_y=c.y;
//			  detectc=true;

    	  }

    	  else if ( detectc==true&&detectt==false ){
    	     		  cout<<"excute if 4"<<endl;
    	     		  detectt=true;
    	     		 c.radiaus+=.23;
//    	 			  double d= sqrt(pow(c.radiaus,2)+pow(c.radiaus,2));
    	 			  double t=(c.radiaus/(ukf.x_[2]));
    	 			  ukf.Prediction(t);
    	 			  cout<<"tome to target="<<t<<endl;
    	 			  double theta= (atan2((ukf.x_[1]-pt1.y),(ukf.x_[0]-pt1.x)));
    	 			  double w=theta/dt;
    	 //			  dt=.2;
    	 	  		  theta+= ukf.x_[2]*t;
    	 			  while (theta > M_PI) theta-=2.*M_PI;
    	 			  while (theta <-M_PI) theta+=2.*M_PI;

    	 			  target_x= hunter_x+c.radiaus*cos( theta);
    	 			  target_y=hunter_y+c.radiaus*sin( theta);
    	 			 cout<<"theta= "<<theta<<endl;
    	     	  }

    	  cout<<"tome to center="<<timetoc<<endl;
    	  cout<<"dt2="<<dt2<<endl;

//    	  target_x= tp.x;
//    	  target_y=tp.y;
    	  cout<<"detct center= "<<detectc<<endl;
    	  cout<<"detct target= "<<detectt<<endl;
    	  cout<<"center= "<<c.x<<", "<<c.y<<endl;
    	  cout<<"target= "<<target_x<<", "<<target_y<<endl;
		  cout<<"hunter= "<<hunter_x<<", "<<hunter_y<<endl;

		  heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
		  while (heading_to_target > M_PI) heading_to_target-=2.*M_PI;
		  while (heading_to_target <-M_PI) heading_to_target+=2.*M_PI;
		  heading_difference = (heading_to_target - hunter_heading)/3;
		  while (heading_difference > M_PI) heading_difference-=2.*M_PI;
		  while (heading_difference <-M_PI) heading_difference+=2.*M_PI;

		  distance_difference = sqrt((target_y - hunter_y)*(target_y - hunter_y) + (target_x - hunter_x)*(target_x - hunter_x));
		  double distance_to_center= distance_difference;

		  if ((hunter_x==target_x&&hunter_y==target_y)||fabs(distance_to_center)<.05)
		  {
			  distance_difference=0;
			  heading_difference=0;
//			  detectc=true;
			  take=true;

		  }
		  if (take==true){
			  detectc=true;
		  }




			  //turn towards the target

		  cout<<"heading to target= "<<heading_to_target<<endl;
		  cout<<"heading difference= "<<heading_difference<<endl;
		  cout<<"distance difference= "<<distance_difference<<endl;
		  cout<<"hunter heading= "<<hunter_heading<<endl;


          json msgJson;
          msgJson["turn"] = heading_difference;
          msgJson["dist"] = distance_difference;
          auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}























































































