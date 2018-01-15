/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <array>
#include <time.h>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[], const Map &map_landmarks, double std_landmark[], double gaussians[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	num_particles = 100;

	gauss_norm = gaussians[0];
	gauss_norm_1 = gaussians[1];
	gauss_norm_2 = gaussians[2];

	for (int i = 0; i < num_particles; i++) {
		Particle part;
		part.id = i;
		part.x = dist_x(gen);
		part.y = dist_y(gen);
		part.theta = dist_theta(gen);
		part.weight = 1.0;
		particles.push_back(part);
	}
	is_initialized = true;
	

	cout << "Initialization done! \n";
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
// TODO: Add measurements to each particle and add random Gaussian noise.
// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
//  http://www.cplusplus.com/reference/random/default_random_engine/

	std::normal_distribution<double> dist_x(0, std_pos[0]);
	std::normal_distribution<double> dist_y(0, std_pos[1]);
	std::normal_distribution<double> dist_theta(0, std_pos[2]);
	 
	// Initialize particles

	double velo_yaw = velocity / yaw_rate;
	double velo_t = velocity * delta_t;
	double yaw_t = yaw_rate * delta_t;

	for (int i = 0; i < num_particles; i++) {
		if (fabs(yaw_rate) > 10E-5) {
			particles[i].x += velo_yaw * (sin(particles[i].theta + yaw_t) - sin(particles[i].theta));
			particles[i].y += velo_yaw * (cos(particles[i].theta) - cos(particles[i].theta + yaw_t));
			particles[i].theta += yaw_t;
		}
		else {
			particles[i].x += velo_t * cos(particles[i].theta);
			particles[i].y += velo_t * sin(particles[i].theta);
		}
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
	

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}



void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
	const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html


	double t_obs_x;
	double t_obs_y;
	double weight_before_exp;
	total_weight = 0;
	max_weight = -1;

	for (int i0 = 0; i0 < num_particles; i0++) {
		particles[i0].weight = 1;
		weight_before_exp = 0;
		particles[i0].associations.clear();
		particles[i0].sense_x.clear();
		particles[i0].sense_y.clear();

		for (int i = 0; i < observations.size(); i++) {

			// Transform the observations into map coordinates
			t_obs_x = particles[i0].x + cos(particles[i0].theta) * observations[i].x - sin(particles[i0].theta) * observations[i].y;
			t_obs_y = particles[i0].y + sin(particles[i0].theta) * observations[i].x + cos(particles[i0].theta) * observations[i].y;

			// Find closest Landmark via loop
		/*	float closest_x = map_landmarks.landmark_list[0].x_f;
			float closest_y = map_landmarks.landmark_list[0].y_f;
			double nearest_dist = dist(t_obs_x, t_obs_y, closest_x, closest_y);
			int close_id = map_landmarks.landmark_list[0].id_i;

			for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
				if (nearest_dist > dist(t_obs_x, t_obs_y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f)) {
					closest_x = map_landmarks.landmark_list[j].x_f;
					closest_y = map_landmarks.landmark_list[j].y_f;
					nearest_dist = dist(t_obs_x, t_obs_y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
					close_id = map_landmarks.landmark_list[j].id_i;
				}
			}*/

			// Find closest landmark via area array
			int x_fc[] = { int(floor(t_obs_x)), int(ceil(t_obs_x)) };
			int y_fc[] = { int(floor(t_obs_y)), int(ceil(t_obs_y)) };
			double cur_d = 10000;
			int cur_id = 0;

			for (int x : x_fc) {
				for (int y : y_fc) {
					area_dist A = area[x + 50][y + 100];
					for (int k = 0; k < A.no; k++) {
						if (cur_d > dist(t_obs_x, t_obs_y, map_landmarks.landmark_list[A.id[k]-1].x_f, map_landmarks.landmark_list[A.id[k]-1].y_f)) {
							cur_d = dist(t_obs_x, t_obs_y, map_landmarks.landmark_list[A.id[k] - 1].x_f, map_landmarks.landmark_list[A.id[k] - 1].y_f);
							cur_id = A.id[k];
						}
					}
				}
			}
			double x_sq = pow(t_obs_x - map_landmarks.landmark_list[cur_id - 1].x_f, 2);
			double y_sq = pow(t_obs_y - map_landmarks.landmark_list[cur_id - 1].y_f, 2);
			int close_id = cur_id;

			// Add associations
			particles[i0].associations.push_back(close_id);
			particles[i0].sense_x.push_back(t_obs_x);
			particles[i0].sense_y.push_back(t_obs_y);

			// Update weights
			//particles[i0].weight *= exp(-(x_sq / gauss_norm_1 + y_sq / gauss_norm_2));
			weight_before_exp += -(x_sq / gauss_norm_1 + y_sq / gauss_norm_2);
			}
		particles[i0].weight = pow(gauss_norm, observations.size()) * exp(weight_before_exp);
		total_weight += particles[i0].weight;
		if (particles[i0].weight > max_weight) {
			max_weight = particles[i0].weight;
		}
	}
}



void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
//	cout << "---------RESAMPLE WEIGHTS------" << "\n";


	for (int i = 0; i < num_particles; i++) {
		particles[i].weight /= total_weight;
	}
	
	max_weight /= total_weight;

	int current_index = rand() % num_particles;
	Particle part = particles[current_index];
	std::vector<Particle> new_particles;
	new_particles.clear();
	double beta = 0;
	for (int i = 0; i < num_particles; i++) {
		double r = ((double)rand() / (RAND_MAX)) *2 * max_weight;
		beta += r;
		while (part.weight < beta) {
			beta -= part.weight;
			current_index += 1;
			current_index %= num_particles ;
			part = particles[current_index];
		}
		new_particles.push_back(part);
	}
	particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
