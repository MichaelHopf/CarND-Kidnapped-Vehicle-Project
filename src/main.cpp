#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "particle_filter.h"
#include <time.h>

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

  //Set up parameters here
  double delta_t = 0.1; // Time elapsed between measurements [sec]
  double sensor_range = 50; // Sensor range [m]

  double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

  double gauss_norms[3] = { 1 / (2 * M_PI * sigma_landmark[0] * sigma_landmark[1]), 2 * pow(sigma_landmark[0], 2) , 2 * pow(sigma_landmark[1], 2) };

 


  // Read map data
  Map map;
  if (!read_map_data("../data/map_data.txt", map)) {
	  cout << "Error: Could not open map file" << endl;
	  return -1;
  }

  Map map_landmarks = map;
  // Set area structure
  std::array<std::array<area_dist, 150>, 350> area;
  // Construct diagram 
  // -50 < x < 300	and		-100 < y < 50	by observation of landmarks
  for (int x = 0; x < 350; x++) {
	  for (int y = 0; y < 150; y++) {
		  int real_x = x - 50;
		  int real_y = y - 100;
		  // obtain the closest landmark
		  double nearest_dist = 10000; //just a large number
		  int close_id = 0;
		  for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			  if (nearest_dist > dist(real_x, real_y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f)) {
				  nearest_dist = dist(real_x, real_y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
				  close_id = map_landmarks.landmark_list[j].id_i;
			  }
		  }
		  area_dist A;
		  A.x = real_x;
		  A.y = real_y;
		  A.no = 1;
		  A.distance.push_back(nearest_dist);
		  A.id.push_back(close_id);

		  // Check if another landmark is in distance 2*sqrt(2)*nearest_dist
		  double detour = 2 * sqrt(2) + nearest_dist;
		  for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			  double cur_dist = dist(real_x, real_y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
			  if ((cur_dist < detour) && (map_landmarks.landmark_list[j].id_i != close_id)) {
				  A.distance.push_back(cur_dist);
				  A.id.push_back(map_landmarks.landmark_list[j].id_i);
				  A.no += 1;
			  }
		  }
		  area[x][y] = A;
	  }
  }


  // Create particle filter
  ParticleFilter pf;

  pf.area = area;

  h.onMessage([&pf,&map,&delta_t,&sensor_range,&sigma_pos,&sigma_landmark,&gauss_norms](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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


          if (!pf.initialized()) {
          	// Sense noisy position data from the simulator
			double sense_x = std::stod(j[1]["sense_x"].get<std::string>());
			double sense_y = std::stod(j[1]["sense_y"].get<std::string>());
			double sense_theta = std::stod(j[1]["sense_theta"].get<std::string>());

			pf.init(sense_x, sense_y, sense_theta, sigma_pos, map, sigma_landmark, gauss_norms);
		  }
		  else {
			// Predict the vehicle's next state from previous (noiseless control) data.
		  	double previous_velocity = std::stod(j[1]["previous_velocity"].get<std::string>());
			double previous_yawrate = std::stod(j[1]["previous_yawrate"].get<std::string>());

			pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
		  }

		  // receive noisy observation data from the simulator
		  // sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
		  	vector<LandmarkObs> noisy_observations;
		  	string sense_observations_x = j[1]["sense_observations_x"];
		  	string sense_observations_y = j[1]["sense_observations_y"];

		  	std::vector<float> x_sense;
  			std::istringstream iss_x(sense_observations_x);

  			std::copy(std::istream_iterator<float>(iss_x),
        	std::istream_iterator<float>(),
        	std::back_inserter(x_sense));

        	std::vector<float> y_sense;
  			std::istringstream iss_y(sense_observations_y);

  			std::copy(std::istream_iterator<float>(iss_y),
        	std::istream_iterator<float>(),
        	std::back_inserter(y_sense));

        	for(int i = 0; i < x_sense.size(); i++)
        	{
        		LandmarkObs obs;
        		obs.x = x_sense[i];
				obs.y = y_sense[i];
				noisy_observations.push_back(obs);
        	}

		  // Update the weights and resample
		  //clock_t mytime1 = clock();
		  pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
		 // clock_t mytime2 = clock();
		  //cout << 1000.0 * (mytime2 - mytime1) / CLOCKS_PER_SEC << "\n";
		  pf.resample();
		//  clock_t mytime3 = clock();
		 // cout << 1000.0 * (mytime3 - mytime2) / CLOCKS_PER_SEC << "\n";

		  // Calculate and output the average weighted error of the particle filter over all time steps so far.
		  vector<Particle> particles = pf.particles;
		  int num_particles = particles.size();
		  double highest_weight = -1.0;
		  Particle best_particle;
		  double weight_sum = 0.0;

		  for (int i = 0; i < num_particles; ++i) {
			if (particles[i].weight > highest_weight) {
				highest_weight = particles[i].weight;
				best_particle = particles[i];
			}
			weight_sum += particles[i].weight;
		  }

		//  cout << "highest w " << highest_weight << endl;
		//  cout << "average w " << weight_sum/num_particles << endl;




          json msgJson;
          msgJson["best_particle_x"] = best_particle.x;
          msgJson["best_particle_y"] = best_particle.y;
          msgJson["best_particle_theta"] = best_particle.theta;

          //Optional message data used for debugging particle's sensing and associations
          msgJson["best_particle_associations"] = pf.getAssociations(best_particle);
          msgJson["best_particle_sense_x"] = pf.getSenseX(best_particle);
          msgJson["best_particle_sense_y"] = pf.getSenseY(best_particle);

          auto msg = "42[\"best_particle\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else {
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























































































