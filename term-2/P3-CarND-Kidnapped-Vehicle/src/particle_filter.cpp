/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <sstream>
#include <iterator>
#include <unordered_map>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    num_particles = 30;
    weights = vector<double>(num_particles);
    particles = vector<Particle>(num_particles);

    default_random_engine gen;

    // Create normal (Gaussian) distributions for x, y, theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++) {
        Particle particle = Particle();
        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1.0d;

        particles[i] = particle;
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find normal_distribution and default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    default_random_engine gen;
    normal_distribution<double> noise_x(0.0d, std_pos[0]);
    normal_distribution<double> noise_y(0.0d, std_pos[1]);
    normal_distribution<double> noise_theta(0.0d, std_pos[2]);

    const double yaw_delta_t = yaw_rate * delta_t;


    for (int i = 0; i < num_particles; i++) {
        if (fabs(yaw_rate) < 0.0001) {
            particles[i].x += velocity * delta_t * cos(particles[i].theta) + noise_x(gen);
            particles[i].y += velocity * delta_t * sin(particles[i].theta) + noise_y(gen);
            particles[i].theta += noise_theta(gen);
        } else {
            particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_delta_t) - sin(particles[i].theta)) + noise_x(gen);
            particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_delta_t)) + noise_y(gen);
            particles[i].theta += yaw_delta_t + noise_theta(gen);
        }
    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs> &observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    for (int i = 0; i < observations.size(); i++) {
        double min_distance_squared = dist(predicted[0].x, predicted[0].y, observations[i].x, observations[i].y);
        observations[i].id = predicted[0].id;;

        for (int j = 1; j < predicted.size(); j++) {
            double current_distance_squared = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);

            if (current_distance_squared < min_distance_squared) {
                min_distance_squared = current_distance_squared;
                observations[i].id = predicted[j].id;
            }
        }
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations, const Map &map_landmarks) {
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


    for (int i = 0; i < num_particles; i++) {

        // convert observation into map coordinates
        vector<LandmarkObs> observations_in_map_coordinate_system;
        for (LandmarkObs obs : observations) {
            LandmarkObs mapObservation;

            // from "Landmarks Quiz Solution" transform to map x, y coordinate
            mapObservation.x = particles[i].x + (cos(particles[i].theta) * obs.x) - (sin(particles[i].theta) * obs.y);
            mapObservation.y = particles[i].y + (sin(particles[i].theta) * obs.x) + (cos(particles[i].theta) * obs.y);

            observations_in_map_coordinate_system.push_back(mapObservation);
        }

        // landmarks
        vector<LandmarkObs> predicted;
        unordered_map<int, LandmarkObs> predicted_dict;
        for (auto map_landmark: map_landmarks.landmark_list) {
            // distance between landmark (map coordinates) and observation into map coordinates
            const double distance = dist(map_landmark.x_f, map_landmark.y_f, particles[i].x, particles[i].y);
            if (distance <= sensor_range) { // check if landmark in sensor_range
                LandmarkObs prediction = LandmarkObs{map_landmark.id_i, map_landmark.x_f, map_landmark.y_f};
                predicted.push_back(prediction);
                predicted_dict[map_landmark.id_i] = prediction;
            }
        }

        dataAssociation(predicted, observations_in_map_coordinate_system);

        vector<int> associations(observations_in_map_coordinate_system.size());
        vector<double> sense_x(observations_in_map_coordinate_system.size());
        vector<double> sense_y(observations_in_map_coordinate_system.size());
        vector<double> new_weights;

        const double sig_x = std_landmark[0];
        const double sig_y = std_landmark[1];
        const double gauss_norm = (1 / (2 * M_PI * sig_x * sig_y)); // calculate normalization term


        for (int j = 0; j < observations_in_map_coordinate_system.size(); j++) {
            LandmarkObs closest_landmark = predicted_dict[observations_in_map_coordinate_system[j].id];

            const double mu_x = closest_landmark.x;
            const double mu_y = closest_landmark.y;

            const double x_obs = observations_in_map_coordinate_system[j].x;
            const double y_obs = observations_in_map_coordinate_system[j].y;

            associations[j] = observations_in_map_coordinate_system[j].id;
            sense_x[j] = x_obs;
            sense_y[j] = y_obs;

            // calculate exponent
            const double exponent = (pow((x_obs - mu_x), 2)) / (2 * pow(sig_x, 2)) + (pow((y_obs - mu_y), 2)) / (2 * pow(sig_y, 2));

            // calculate weight using normalization terms and exponent
            new_weights.push_back(gauss_norm * exp(-exponent));
        }

        SetAssociations(particles[i], associations, sense_x, sense_y);

        double weight = accumulate(new_weights.begin(), new_weights.end(), 1.0, multiplies<double>());
        particles[i].weight = weight;
        weights[i] = weight;

    }

}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    discrete_distribution<int> d(weights.begin(), weights.end());
    vector<Particle> new_particles(num_particles);

    default_random_engine gen;

    for (int i = 0; i < num_particles; i++) {
        int id = d(gen);
        new_particles[i] = particles[id];
    }
    particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, vector<int> associations, vector<double> sense_x,
                                         vector<double> sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
