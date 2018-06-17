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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 100;
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    default_random_engine gen;
    for(int i=0; i<num_particles; i++){
        Particle p = {i, dist_x(gen), dist_y(gen), dist_theta(gen), 1.0};
        particles.push_back(p);
        weights.push_back(1.0);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    normal_distribution<double> noise_x(0, std_pos[0]);
    normal_distribution<double> noise_y(0, std_pos[1]);
    normal_distribution<double> noise_theta(0, std_pos[2]);
    default_random_engine gen;
    for (Particle &p : particles){
        if (fabs(yaw_rate) < 1e-5){
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
        }
        else{
            p.x += velocity/yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            p.y += velocity/yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            p.theta += yaw_rate * delta_t;
        }
        p.x += noise_x(gen);
        p.y += noise_y(gen);
        p.theta += noise_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<Map::single_landmark_s> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    for(LandmarkObs &ob : observations){
        double min_d = 1e9;
        for(Map::single_landmark_s pre : predicted){
            double d = dist(ob.x, ob.y, pre.x_f, pre.y_f);
            if(d < min_d){
                min_d = d;
                ob.id = pre.id_i;
            }
        }
    }

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
    
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    double xm, ym;
    int p_idx = 0;
    for(Particle &p : particles){
        vector<LandmarkObs> transformed_obs;
        for(LandmarkObs ob : observations){
            xm = p.x + cos(p.theta) * ob.x - sin(p.theta) * ob.y;
            ym = p.y + sin(p.theta) * ob.x + cos(p.theta) * ob.y;
            LandmarkObs transformed_ob = {ob.id, xm, ym};
            transformed_obs.push_back(transformed_ob);
        }
        dataAssociation(map_landmarks.landmark_list, transformed_obs);
        p.weight = 1.0;
        for(LandmarkObs &ob : transformed_obs){
            // Since map_landmarks.landmark_list is sorted by ids starting from 1, we can use id-1 as the index in the list
            xm = map_landmarks.landmark_list[ob.id - 1].x_f;
            ym = map_landmarks.landmark_list[ob.id - 1].y_f;
            p.weight *= 1/(2 * M_PI * sig_x * sig_y) *
            exp(-(pow(ob.x - xm,2)/(2 * sig_x*sig_x) + pow(ob.y-ym,2)/(2 * sig_y*sig_y)));
        }
        weights[p_idx] = p.weight;
        p_idx += 1;
    }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    discrete_distribution<int> discrete_dist(weights.begin(), weights.end());
    default_random_engine gen;
    vector<Particle> resamples(particles.size());
    for (int i = 0; i < particles.size(); i++){
        int p_idx = discrete_dist(gen);
        resamples[i] = particles[p_idx];
        weights[i] = resamples[i].weight;
    }
    particles = resamples;
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
