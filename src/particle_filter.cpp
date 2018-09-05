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
	num_particles = 100;
	std::cout << "Number of particles: " << num_particles << std::endl;

	// Invoke random number generator and normal distribution
	std::default_random_engine rnd_gen;
	std::normal_distribution<double> std_x(0.0,std[0]);
	std::normal_distribution<double> std_y(0.0,std[1]);
	std::normal_distribution<double> std_theta(0.0,std[2]);

	// Initialize weights array
	weights.resize(num_particles);

	// Create vector with particles and random uncertainty based on provided sigmas
	particles.resize(num_particles);
	for (unsigned int j = 0; j < num_particles; j++) {
		particles[j].id = j;
		particles[j].x = x + std_x(rnd_gen);
		particles[j].y = y + std_y(rnd_gen);
		particles[j].theta = theta + std_theta(rnd_gen);
		particles[j].weight = 1.0;
	}

	// Set initialization flag to true
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

	// Create distributions for adding noise
	std::normal_distribution<double> std_x(0.0,std_pos[0]);
	std::normal_distribution<double> std_y(0.0,std_pos[1]);
	std::normal_distribution<double> std_theta(0.0,std_pos[2]);

	// To avoid division by zero, include two sets of equations
	// Predict particle position, but precalculate some variables first (faster!)
	double yawd_dt = yaw_rate * delta_t;
	double v_yawd = velocity / yaw_rate;
	double theta = 0.;
	double v_t = velocity * delta_t;
	for (unsigned int j = 0; j < num_particles; j++) {
		theta = particles[j].theta; 		// Read theta from particle
		if (abs(yaw_rate) < 0.00001) {
			particles[j].x += v_t * cos(theta);
			particles[j].y += v_t * sin(theta);
			// Theta stays the same, since yaw rate is nearly zero
		}
		else {
			particles[j].x += v_yawd * ( sin(theta + yawd_dt) - sin(theta)) + std_x(rnd_gen);
			particles[j].y += v_yawd * (-cos(theta + yawd_dt) + cos(theta)) + std_y(rnd_gen);
			particles[j].theta += yawd_dt + std_theta(rnd_gen);
		}
	}
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

	// Precalculate some terms of the multi-variate Gaussian distribution first
	// to save calculation time. They stay constant throughout execution.
  const double GD_a = 0.5 / ( M_PI * std_landmark[0] * std_landmark[1] );
  // The variance of the multi variate gaussian distribution stays constant
	const double var_x_2 = 2 * std_landmark[0] * std_landmark[0]; // 2x Variance in x dir
	const double var_y_2 = 2 * std_landmark[1] * std_landmark[1]; // 2x Variance in y dir

	// Read in landmarks
	vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;

  // Iterate through each particle
  for (unsigned int i = 0; i < num_particles; ++i) {

    // Create variable to calculate cumulative product of weight
    double GD_temp = 1.0;

		// Calculate distance between particle and all Landmark
		vector<double> LM_p_dist (landmarks.size());
		for (unsigned int k = 0; k < landmarks.size(); ++k) {
			LM_p_dist[k]	= dist(	particles[i].x, 	particles[i].y,
														landmarks[k].x_f, landmarks[k].y_f);
		}

    // For each observation
    for (unsigned int j = 0; j < observations.size(); ++j) {

      // Transform observation from vehicle coodinate system to map coordinates)
      double x_obs_glob, y_obs_glob;
      x_obs_glob = 	particles[i].x +
										observations[j].x * cos(particles[i].theta) -
										observations[j].y * sin(particles[i].theta);
      y_obs_glob = 	particles[i].y +
										observations[j].x * sin(particles[i].theta) +
										observations[j].y * cos(particles[i].theta);

			// Walk through all landmarks
      vector<double> LM_obs_dist (landmarks.size());
      for (unsigned int k = 0; k < landmarks.size(); ++k) {

        // Only check landmarks which are in sensor range.
        // Use nearest neighbour approach. If in range, calculate distance
				// between measurement and nearest neighbour landmark object.
        if (LM_p_dist[k] <= sensor_range) {
          LM_obs_dist[k] = dist(	x_obs_glob, 			y_obs_glob,
																	landmarks[k].x_f, landmarks[k].y_f);
        }
				else {
          // Set landmarks outside sensor range to high distance value.
          LM_obs_dist[k] = 88888.;
        }
      }

      // Associate observation point with its nearest neighbor landmark
      int min_pos = distance(LM_obs_dist.begin(),
								 min_element(LM_obs_dist.begin(),LM_obs_dist.end()));

      // Calculate weight based on multi-variate Gaussian distribution
      double x_diff = x_obs_glob - landmarks[min_pos].x_f;
      double y_diff = y_obs_glob - landmarks[min_pos].y_f;
      double GD_b = (x_diff * x_diff / var_x_2) +
										(y_diff * y_diff / var_y_2);
      GD_temp *= GD_a * exp(-1 * GD_b);

    }

    // Update particle weights with combined multi-variate Gaussian distribution
    particles[i].weight = GD_temp;
    weights[i] = GD_temp;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional
	//   to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Invoke discrete distribution generator:
	// Generates integer values dependent on provided weight vector. This allows
	// to resample particles based on their weight (higher weight leads to
	// higher proability and higher count of this particle)
	std::discrete_distribution<unsigned int> discrete_distr (weights.begin(),weights.end());
	// Generate random number generator
	std::default_random_engine generator;


	// Prepare a vector to hold the new particles
	std::vector<Particle> particles_resampled (num_particles);

	// Use discrete distribution to return particles by weight
	for (unsigned int j = 0; j < particles.size(); ++j) {
		particles_resampled[j] = particles[discrete_distr(generator)];
	}

	// Replace particles vector with resampled one
	particles = particles_resampled;
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
		return particle;
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
