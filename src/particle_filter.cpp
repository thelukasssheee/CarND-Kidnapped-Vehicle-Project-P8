/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <math.h>
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first
	//       position (based on estimates of
	//       x, y, theta and their uncertainties from GPS) and all weights to 1.
	//       Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method
	// 			 (and others in this file).

	// Set number of particles
	num_particles = 5;
	std::cout << "Number of particles: " << num_particles << std::endl;

	// Invoke random number generator and normal distribution
	std::default_random_engine rnd_gen;
	std::normal_distribution<double> std_x(0.0,std[0]);
	std::normal_distribution<double> std_y(0.0,std[1]);
	std::normal_distribution<double> std_theta(0.0,std[2]);

	// Create vector with particles and random uncertainty based on provided sigmas
	particles.resize(num_particles);
	for (unsigned int j = 0; j < num_particles; j++) {
		particles[j].id = j;
		particles[j].x = x + std_x(rnd_gen);
		particles[j].y = y + std_y(rnd_gen);
		particles[j].theta = theta + std_theta(rnd_gen);
		// std::cout << "x: " << particles[j].x << ", y: " << particles[j].y << std::endl;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
																double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution
	//  and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Invoke random number generator and normal distribution
	std::default_random_engine rnd_gen;
	std::normal_distribution<double> std_x(0.0,std_pos[0]);
	std::normal_distribution<double> std_y(0.0,std_pos[1]);
	std::normal_distribution<double> std_theta(0.0,std_pos[2]);

	// Predict particle position, but initialize some variables first (faster!)
	double yawd_dt = yaw_rate * delta_t;
	double v_yawd = velocity / yaw_rate;
	double theta = 0.;
	for (unsigned int j = 0; j < num_particles; j++) {
		theta = particles[j].theta;
		particles[j].x += v_yawd * ( sin(theta + yawd_dt) - sin(theta)) + std_x(rnd_gen);
		particles[j].y += v_yawd * (-cos(theta + yawd_dt) + cos(theta)) + std_y(rnd_gen);
		particles[j].theta += yawd_dt + std_theta(rnd_gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> pot_landmarks,
																		 std::vector<LandmarkObs> observ_global) {
	// TODO: Find the predicted measurement that is closest to each observed
	//   measurement and assign the observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will
	//   probably find it useful to implement this method and use it as a helper
	//	 during the updateWeights phase.
	//   POT_LANDMARKS: all landmarks within sensor range for one specific particle
	//   OBSERV_GLOBAL: actual landmark measurements from lidar within sensor range

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian
	//   distribution. You can read more about this distribution here:
	//   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your
	//   particles are located according to the MAP'S coordinate system. You
	//   will need to transform between the two systems. Keep in mind that this
	//   transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to
	//   implement (look at equation 3.33)
	//   http://planning.cs.uiuc.edu/node99.html

	double x = 0.;			// Current particle position x
	double y = 0.;			// Current particle position y
	double theta = 0.;	// Current particle yaw angle theta

	double x_f = 0.;	  // Current landmark position x
	double y_f = 0.;	  // Current landmark position y

	double x_loc = 0.;	// Current observation in local coordinates
	double y_loc = 0.;	// Current observation in local coordinates

	double w_fac = 1/(2*M_PI*std_landmark[0]*std_landmark[1]);
	std::cout << "w_fac = " << w_fac << std::endl;

	double var_x = std_landmark[0] * std_landmark[0];
	double var_y = std_landmark[1] * std_landmark[1];



	// Walk through all particles
	std::vector<LandmarkObs> pot_landmarks;
	std::vector<LandmarkObs> observ_global;
	for (unsigned int j = 0; j < num_particles; ++j) {
		x = particles[j].x;
		y = particles[j].y;
		theta = particles[j].theta;

		// Find landmarks in map, which are within sensor Range for current particle
		// Identified landmarks will be stored as potential landmarks
		// Walk through all map landmarks
		vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
		std::cout << "Landmarks: " << landmarks.size() << ", ";
		std::cout << "Observations: " << observations.size() << std::endl;
		// Reset variables for this particle before walking through landmarks
		pot_landmarks.clear();
		LandmarkObs curr_landmark;
		for (unsigned int k = 0; k < landmarks.size(); ++k) {
		// for (auto& k : map_landmarks) {
			x_f = map_landmarks.landmark_list[k].x_f;
			y_f = map_landmarks.landmark_list[k].y_f;
			// Check if current landmark is within sensor range of current particle
			if (dist(x, y, x_f, y_f) < sensor_range) {
				// Add current landmark to list of potential landmark candidates
				curr_landmark.id = map_landmarks.landmark_list[k].id_i;
				curr_landmark.x = x_f;
				curr_landmark.y = y_f;
				pot_landmarks.push_back(curr_landmark);
				// std::cout << "LM " << k << ": " << map_landmarks.landmark_list[k].x_f
				// << "," << map_landmarks.landmark_list[k].y_f << std::endl;
			}
		}

		// Convert lidar measurements of current predicted particle to
		// global coordinate system
		LandmarkObs meas;
		observ_global.clear();
		for (unsigned int k = 0; k < observations.size(); ++k) {
			x_loc = observations[k].x;
			y_loc = observations[k].y;
			meas.id = observations[k].id;
			meas.x = x_loc * cos(theta) - y_loc * sin(theta) + x;
			meas.y = y_loc * sin(theta) + y_loc * cos(theta) + y;
			observ_global.push_back(meas);
		}

		// Correlate/associate potential map landmarks and lidar observations
		// dataAssociation(pot_landmarks, observ_global);
		std::vector<double> dists_x;
		std::vector<double> dists_y;
		dists_x.resize(observ_global.size());
		dists_y.resize(observ_global.size());
		for (unsigned int k = 0; k < observ_global.size(); ++k) {
			double min_dist = dist(	observ_global[k].x, observ_global[k].y,
															pot_landmarks[0].x, pot_landmarks[0].y);
			unsigned int min_loc = 0;
			double curr_dist;
			for (unsigned int m = 1; m < pot_landmarks.size(); ++m) {
				curr_dist = dist(	observ_global[k].x, observ_global[k].y,
													pot_landmarks[m].x, pot_landmarks[m].y);
				if (curr_dist < min_dist) {
					min_dist = curr_dist;
					min_loc = m;
				}
			}
			// Save landmark ID back to observation
			observ_global[k].id = pot_landmarks[min_loc].id;
			dists_x[k] = observ_global[k].x - pot_landmarks[min_loc].x;
			dists_y[k] = observ_global[k].y - pot_landmarks[min_loc].y;
		}

		// Update weight of particle based on quality of sensor measurements
		double weights_cum = 1.;
		for (unsigned int k = 0; k < observ_global.size(); ++k) {
			weights_cum *= w_fac / exp( 0.5 *
				dists_x[k]*dists_x[k] / var_x + dists_y[k]*dists_y[k] / var_y);
		}
		weights[j] = weights_cum;
		particles[j].weight = weights_cum;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional
	//   to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle,
		const std::vector<int>& associations, const std::vector<double>& sense_x,
		const std::vector<double>& sense_y) {
    // particle: the particle to assign each listed association, and
		// association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
		return particle; // TODO: is that correct???
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
