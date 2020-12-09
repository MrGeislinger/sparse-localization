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

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::numeric_limits;
using std::uniform_real_distribution;
using std::max_element;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * - Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * - Add random Gaussian noise to each particle.
   */
  // Make sure we have enough particles implemented for state space
  num_particles = 1000;

  // Will add noise to each particle based on Gaussian & std deviation given
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Set each particle to the x,y coordinates and heading (theta)
  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    // Set each weight to 1 (all equally likely)
    p.weight = 1.0;
    // Add to list of all current particles
    particles.push_back(p);
  }

  // Initialize weights (stored separately from each particle); equally likely
   weights = vector<double>(num_particles, 1.0);
  
  // Update flag to signal initialization
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   */
  // Update the particles locations
  for (int i=0; i < num_particles; i++) {
    Particle p = particles[i];
    // Default to no change (small change) in theta
    double v_scaled = velocity;
    double x_delta_by_theta = cos(p.theta);
    double y_delta_by_theta = sin(p.theta);
    // Check that yaw_rate isn't (near) zero (no spinning in theta)
    if (fabs(yaw_rate) > 0.001) { //~0.05 degrees; small on short time scale
      // Takes into the account the acceleration (change in theta)
      v_scaled = velocity / yaw_rate;
      x_delta_by_theta = sin(p.theta + yaw_rate*delta_t) - sin(p.theta);
      y_delta_by_theta = cos(p.theta) - cos(p.theta + yaw_rate * delta_t);
    } 
    // Update after taking into account of theta
    p.x = p.x + v_scaled * x_delta_by_theta;
    p.y = p.y + v_scaled * y_delta_by_theta;
    p.theta = p.theta + yaw_rate*delta_t;
    
    // Add noise directly to position & heading (yaw)
    // NOTE: In some cases, we could add noise directly to the velocities
    std::normal_distribution<double> dist_x(p.x, std_pos[0]);
    std::normal_distribution<double> dist_y(p.y, std_pos[1]);
    std::normal_distribution<double> dist_theta(p.theta, std_pos[2]);
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs> &observations) {
  /**
   * Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  // Iterate over each observation and compare each landmark (predicted)
  double distance;
  double min_distance;
  int landmark_id;
  for (int o=0; o < observations.size(); o++) {
    LandmarkObs observation = observations[o];
    // Initialize a maximum to compare for the observation (always does the 1st)
    min_distance = numeric_limits<double>::max();
    // Iterate over each landmark to find closest
    for (int p=0; p < predicted.size(); p++) {
      LandmarkObs landmark = predicted[p];
      //Find the distance to landmark from observation
      distance = dist(observation.x, observation.y, landmark.x, landmark.y);
      // If shortest distance so far, save the id for the observation
      // Note that a tie for shortest goes to the first one found
      if (distance < min_distance) {
        min_distance = distance;
        landmark_id = landmark.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a multi-variate Gaussian 
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
  // Repeat process for each particle
  for (int i=0; i < num_particles; i++) {
    Particle p = particles[i];

    // Iterate over each landmark to ensure its within range
    vector<LandmarkObs> possible_predictions;
    for (int j=0; j < map_landmarks.landmark_list.size(); j++) {
      // Get the landmark coordinates to make life easier
      float lm_x = map_landmarks.landmark_list[j].x_f;
      float lm_y = map_landmarks.landmark_list[j].y_f;
      int lm_id = map_landmarks.landmark_list[j].id_i;

      // Only look at landmarks within the sensor range of the particle
      // TODO: Consider making more efficient by looking at just x&y (square)
      double dist_to_landmark = dist(p.x, p.y, lm_x, lm_y);
      if (dist_to_landmark < sensor_range) { 
        // Add to possible prediction
        possible_predictions.push_back(LandmarkObs{lm_id, lm_x, lm_y});
      }
    }

    // Transformed observations list to later associate with probable landmarks
    vector<LandmarkObs> observations_trans;
    for (int k=0; k < observations.size(); k++) {
      LandmarkObs obs = observations[k];
      double obs_tx = cos(p.theta)*obs.x - sin(p.theta)*obs.y + p.x;
      double obs_ty = sin(p.theta)*obs.x + cos(p.theta)*obs.y + p.y;
      observations_trans.push_back(LandmarkObs{obs.id, obs_tx, obs_ty});
    }

    // Find the associated landmark from map's coordinate system of observations
    dataAssociation(possible_predictions, observations_trans);
    
    // Find the correct prediction for observation
    double p_weight;
    for (int o=0; o < observations_trans.size(); o++) {
      LandmarkObs obs_trans = observations_trans[o];
      LandmarkObs pred;
      // Get the landmark associated with observation
      for (int j=0; j < possible_predictions.size(); j++) {
        if (possible_predictions[j].id == obs_trans.id) {
          pred = possible_predictions[j];
          break;
        }
      }

      // Update weight using PDF based on predicted landmarks
      p_weight = 1.0;
      double obs_w = 
        1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]) 
        * 
        exp( -(
          pow(pred.x - obs_trans.x,2) / (2*pow(std_landmark[0],2)) 
          + 
          pow(pred.y - obs_trans.y,2) / (2*pow(std_landmark[1],2))
        ));

      // product of this obersvation weight with total observations weight
      p_weight *= obs_w;
    }
    weights[i] = p_weight;
    particles[i].weight = p_weight;
  }

  

}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  // Only use weights with the given range
  double weight_max = *max_element(weights.begin(), weights.end());
  // Pick a random particle to start from
  int index = rand() % num_particles;
  // Keep track of where you are in the cycle
  double beta = 0.0;
  // Uniformly random value
  uniform_real_distribution<double> dist_0_1(0.0, 1.0);
  
  // Go around the wheel/cycle to get new particles
  // Weights determine the size/area to take up to be resampled
  vector<Particle> new_particles;
  for (int i=0; i < num_particles; i++) {
    // Have to move "beta" more to get to next particle
    beta += dist_0_1(gen) * 2.0 * weight_max;
    // Move around until we hit the final weight index
    while(beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }
  // Set the new particles to be the next particlees to use
  particles = new_particles;
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
  particle.associations = associations;
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