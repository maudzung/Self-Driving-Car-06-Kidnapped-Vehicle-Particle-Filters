/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <cassert>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  if (is_initialized) {
    return;
  }
  num_particles = 1000;

  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);


  // particles.resize(num_particles);
  particles = std::vector<Particle>(static_cast<unsigned long>(num_particles));
  weights = std::vector<double>(static_cast<unsigned long>(num_particles), 1.0);

  for(int i=0; i < num_particles; i++){
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = weights[i];
  }

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
  double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::normal_distribution<double> dist_x(0., std_pos[0]);
  std::normal_distribution<double> dist_y(0., std_pos[1]);
  std::normal_distribution<double> dist_theta(0., std_pos[2]);

  std::default_random_engine gen;
  for(auto &p:particles){
    // if yaw_rate ~ 0. ==> do as normal
    if (fabs(yaw_rate) < 0.000001) {
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);
    } else {
      double delta_theta = yaw_rate * delta_t;
      p.x += velocity / yaw_rate * (sin(p.theta + delta_theta) - sin(p.theta));
      p.y += velocity / yaw_rate *(cos(p.theta) - cos(p.theta + delta_theta));
      p.theta += delta_theta;
    }
    // Add noise
    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
 vector<LandmarkObs>& observations) {
/**
 * Find the predicted measurement that is closest to each 
 *   observed measurement and assign the observed measurement to this 
 *   particular landmark.
 * NOTE: this method will NOT be called by the grading code. But you will 
 *   probably find it useful to implement this method and use it as a helper 
 *   during the updateWeights phase.
 */
// Initialize min_dist value to the maximum double

  for(auto &obser : observations){
    double min_dist = std::numeric_limits<double>::max();

    obser.id = -1;

    for(auto const &pred : predicted){
    // Calculate Euclidean distance between a prediction and a observation
      double distance = dist(obser.x, obser.y, pred.x, pred.y);
      if (distance <= min_dist){
        min_dist = distance;
        obser.id = pred.id;
      }
    }
  // Check whether the landmark found or not
    assert(obser.id != -1);
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
 const vector<LandmarkObs> &observations, 
 const Map &map_landmarks) {
/**
 * Update the weights of each particle using a mult-variate Gaussian 
 *   distribution. You can read more about this distribution here: 
 *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
 * NOTE: The observations are given in the VEHICLE'S coordinate system. 
 *   Your particles are located according to the MAP'S coordinate system. 
 *   You will need to transform between the two systems. Keep in mind that
 *   this transformation requires both rotation AND translation (but no scaling).
 *   The following is a good resource for the theory:
 *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
 *   and the following is a good resource for the actual equation to implement
 *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
 */

  const double std_x = std_landmark[0];
  const double std_y = std_landmark[1];
  const double std_x_square = pow(std_x, 2);
  const double std_y_square = pow(std_y, 2);
  const double nuy_norm = 1 / (2 * M_PI * std_x * std_y);


  for(unsigned int p_idx=0; p_idx < particles.size(); p_idx++){
    Particle const &p = particles[p_idx];
  // Step 1: Transform from the car coordinate the map coordinate
    vector<LandmarkObs> transformed_observations(observations.size());
    for(unsigned int i=0; i < observations.size(); i++){
      double sin_theta = sin(p.theta);
      double cos_theta = cos(p.theta);
      LandmarkObs obser = observations[i];

      transformed_observations[i].x = p.x + cos_theta * obser.x - sin_theta * obser.y;
      transformed_observations[i].y = p.y + sin_theta * obser.x + cos_theta * obser.y;
      transformed_observations[i].id = -1; // haven't know with which landmark to associate this observation yet
    }

    // Step 2: Association - Neareast neighbor
    // Choose landmark within the sensor_range

    vector<LandmarkObs> predicted;
    for(auto const &m_landmark : map_landmarks.landmark_list){
      double distance = dist(m_landmark.x_f, m_landmark.y_f, p.x, p.y);
      if (distance <= sensor_range){
        LandmarkObs lm_added = {
          .id = m_landmark.id_i,
          .x = static_cast<double>(m_landmark.x_f), // Float to double
          .y = static_cast<double>(m_landmark.y_f), // Float to double

        };
        predicted.push_back(lm_added);
      }
    }
    // Checked: predicted is not empty 
    assert(!predicted.empty());

    // associate transformed observations with predicted >> nearest neighbor
    dataAssociation(predicted, transformed_observations);

    // Step 3: Update weights
    // calculate probabilities
    double prob = 1.0;
    for(unsigned int i=0; i < transformed_observations.size(); i++){
    // for(auto &trans_obser : transformed_observations){
      LandmarkObs trans_obser = transformed_observations[i];
    // if no matching landmark then set prob = 0. for the particle
      if(trans_obser.id == -1){
        prob = 0.0;
        break;
      }
      else {
        LandmarkObs nearest_lm = {
          .id = -1,
          .x = static_cast<double> (map_landmarks.landmark_list[trans_obser.id - 1].x_f),
          .y = static_cast<double> (map_landmarks.landmark_list[trans_obser.id - 1].y_f),
        };

        double delta_x_square = pow(trans_obser.x - nearest_lm.x, 2);
        double delta_y_square = pow(trans_obser.y - nearest_lm.y, 2);

        prob *= nuy_norm * exp(-(delta_x_square / (2 * std_x_square) + delta_y_square / (2 * std_y_square)));
      }
    }

    particles[p_idx].weight = prob;

  }
  
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  for (unsigned int p_idx = 0; p_idx < particles.size(); p_idx++) {
    weights[p_idx] = particles[p_idx].weight;
  }

  std::default_random_engine gen;
  std::discrete_distribution<size_t> dist_weight(weights.begin(), weights.end());

  vector<Particle> resampled_particles(particles.size());

  for(unsigned int p_idx=0; p_idx < particles.size(); p_idx++){
    int sampling_index = dist_weight(gen);
    resampled_particles[p_idx] = particles[sampling_index];
  }

  particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
 const vector<int>& associations, 
 const vector<double>& sense_x, 
 const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}