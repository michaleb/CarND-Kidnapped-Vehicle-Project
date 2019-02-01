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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  
  //random noise distributions with zero mean and std of respective variables
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(0, std[0]);
  std::normal_distribution<double> dist_y(0, std[1]);
  std::normal_distribution<double> dist_theta(0, std[2]);  

  num_particles = 70;  // TODO: Set the number of particles
  Particle particle;
    
  for (unsigned int i=0; i< num_particles; i++) {

    particle.id = i+1;
    particle.x = x + dist_x(gen);
    particle.y = y + dist_y(gen);
    particle.theta = theta + dist_theta(gen);
    particle.weight = 1;
    
    particles.push_back(particle);
  }
  is_initialized = true;
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;

  for (unsigned int i=0; i< num_particles; i++) {
  
    if (yaw_rate == 0) { 
      particles[i].x += cos(particles[i].theta) * velocity * delta_t;
      particles[i].y += sin(particles[i].theta) * velocity * delta_t;
    }
    
    else {
      particles[i].x += velocity/yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      particles[i].y += velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
    }
    particles[i].theta += yaw_rate * delta_t;

    std::normal_distribution<double> dist_px(0, std_pos[0]);
    std::normal_distribution<double> dist_py(0, std_pos[1]);
    std::normal_distribution<double> dist_ptheta(0, std_pos[2]); 

    particles[i].x += dist_px(gen);
    particles[i].y += dist_py(gen);
    particles[i].theta += dist_ptheta(gen);
  }  
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  
  for (unsigned int i=0; i < observations.size(); i++) {
    double shortest_dist = std::numeric_limits<double>::infinity();

    for (unsigned int j=0; j < predicted.size(); j++) {
      double diff_dist = fabs(predicted[j].x - observations[i].x) + fabs(predicted[j].y - observations[i].y);
      
      if (diff_dist < shortest_dist) {
        shortest_dist = diff_dist;
      //Assigning the landmark id to the "observation" closest to each in-range landmark 
        observations[i].id = predicted[j].id; 
      //Due to the non-const reference; dataAssociation function will alter
      // transformed_obs variable in calling function
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
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

  for (unsigned int i=0; i< num_particles; i++) {
    //create vector to store landmarks within range of sensor of each particle (predicted state)
    vector <LandmarkObs> in_range;

    for (unsigned int j=0; j < map_landmarks.landmark_list.size(); j++) {
      
      int landmark_id = map_landmarks.landmark_list[j].id_i;
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;

      //checking for landmark x,y coordinates within sensor range of each predicted state
      if (fabs(landmark_x - particles[i].x) <= sensor_range &&
         fabs(landmark_y - particles[i].y) <= sensor_range) {
         
         in_range.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
      }
    }
  
    vector <LandmarkObs> transformed_obs; //vector to store tranformed x,y coordinates
    
    for (unsigned int k=0; k < observations.size(); k++) {
      // transform car observation x coordinate to map x coordinate    
      double t_obs_x = particles[i].x + cos(particles[i].theta) * observations[k].x - sin(particles[i].theta) * observations[k].y;

      // transform car observation x coordinate to map y coordinate    
      double t_obs_y = particles[i].y + sin(particles[i].theta) * observations[k].x + cos(particles[i].theta) * observations[k].y;

      //Adding the [observation.id] placeholder that will be populated in the dataAssociation module 
      transformed_obs.push_back(LandmarkObs{observations[k].id, t_obs_x, t_obs_y});  
    } 
    
    dataAssociation(in_range, transformed_obs);

    //re-initialize before calculating weights of new predicted state
    particles[i].weight = 1.0;
    
    for (unsigned int k=0; k < observations.size(); k++) {
      for (unsigned int j=0; j < in_range.size(); j++) {
        
        //obtaining closest landmark/measurement coordinates for each predicted state using matching IDs
        if (transformed_obs[k].id == in_range[j].id) { 
          // calculate normalization term
          double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

          // calculate exponent
          double exponent = (pow(transformed_obs[k].x - in_range[j].x, 2) / (2 * pow(std_landmark[0], 2)))
                       + (pow(transformed_obs[k].y - in_range[j].y, 2) / (2 * pow(std_landmark[1], 2)));
            
          // calculate weight using normalization terms and exponent
          particles[i].weight *= gauss_norm * exp(-exponent);
        }
      }
    }
  }  
}

  
void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  double beta = 0.0;
  std::uniform_int_distribution <int> dist(0, num_particles - 1); 
  std::default_random_engine gen;
  
  int index = dist(gen);
  vector <double> weights;
  vector <Particle> weighted_particles;

  for (int i=0; i < num_particles; ++i) {
    weights.push_back(particles[i].weight);
  }  

  double mw = *max_element(std::begin(weights), std::end(weights)); 
  std::uniform_real_distribution <double> dist_r(0.0, mw);

  for (int i=0; i < num_particles; ++i) {
    beta += 2.0 * dist_r(gen);
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    weighted_particles.push_back(particles[index]);
  }
  particles = weighted_particles;

  
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