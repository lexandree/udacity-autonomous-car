/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 *
 * Modified on June 19, 2020
 * Author: Alexander Andreev
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
//#include <cassert>

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 50;

  random_device rd;
  mt19937 engine(rd());
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // initialize particle; add a Gaussian noise to the initial GPS coordinates
  particles = vector<Particle>(num_particles);
  for (size_t i = 0; i < num_particles; i++) {
    particles[i].x = dist_x(engine);
    particles[i].y = dist_y(engine);
    particles[i].theta = dist_theta(engine);
    particles[i].weight = 1.0;
    particles[i].id = (int) i; 
  }
  //weights = vector<double>(static_cast<unsigned long>(num_particles), 1.0);
  weights = vector<double>(num_particles, 1.0);
  // initialization step is finished
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
  random_device rd;
  mt19937 engine(rd());
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  for (auto &particle : particles) {
    particle.theta += yaw_rate * delta_t + dist_theta(engine);
    //particles.theta %= 2 * np.pi
	double distance (velocity * delta_t);
	particle.x += distance * cos(particle.theta) + dist_x(engine);
	particle.y += distance * sin(particle.theta) + dist_y(engine);
	// alternative is Constant Turn Rate and Velocity (CTRV) model
	// with the deterministic transition matrix:
	// (v/theta_dot)*(sin(theta_dot*dt + theta) - sin(theta)) + x
	//-(v/theta_dot)*(cos(theta_dot*dt + theta) - cos(theta)) + y
	// (theta_dot*dt + theta)
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
   // find the closest predicted observation
  for (auto &observation : observations) {
	double min_distance(numeric_limits<double>::max());

	for (const auto &pred_obs : predicted) {
	  double distance = dist(observation.x, observation.y, pred_obs.x, pred_obs.y);
	  if (distance < min_distance) {
		observation.id	 = pred_obs.id;
		min_distance = distance;
	  }
	}
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {

  for (size_t i = 0; i < particles.size(); i++) {
	// transform observations to the MAP'S coordinate system with rotation AND translation
    vector<LandmarkObs> observations_transformed(observations.size());
    for (size_t k = 0; k < observations.size(); k++) {
	  double* coords = rotate_translate(particles[i], observations[k]);
      observations_transformed[k].x = coords[0];
      observations_transformed[k].y = coords[1];
      observations_transformed[k].id = 0;  // association is unknown yet
    }
    // select landmarks that are within the range of the sensor
    vector<LandmarkObs> landmarks;
    for (auto const &landmark : map_landmarks.landmark_list) {
      if (dist(particles[i].x, particles[i].y, landmark.x, landmark.y) <= sensor_range) {
		LandmarkObs current_obs {landmark.id, landmark.x, landmark.y};
        landmarks.push_back(current_obs);
      }
    }

    // associate transformed observations with landmarks that are within range, also set id
    dataAssociation(landmarks, observations_transformed);

    for (auto const &observation : observations_transformed) {
	  // attention! the state now is: index of map_landmarks.landmark_list = id - 1
	  // when map list has a not ordered id's then use a nested loop instead
	  particles[i].weight *= normal_pdf(observation, map_landmarks.landmark_list[observation.id - 1], std_landmark);
    }
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
	vector<Particle> new_particles(particles.size());
    random_device rd;
    mt19937 engine(rd());
    discrete_distribution<size_t> distribution(weights.begin(), weights.end());
    for(size_t i=0; i < weights.size(); i++) {
        new_particles[i] = particles[distribution(engine)];
		new_particles[i].weight = 1.0;
    }
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
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
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

  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}


// transform observations to the MAP'S coordinate system with rotation AND translation
double * ParticleFilter::rotate_translate(const Particle &position, const LandmarkObs &observation) {
	static double coordinates[2];
	double cos_theta = cos(position.theta);
	double sin_theta = sin(position.theta);
	coordinates[0] = position.x + cos_theta * observation.x - sin_theta * observation.y;
	coordinates[1] = position.y + sin_theta * observation.x + cos_theta * observation.y;
	return coordinates;
}
