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
	std::cout << "/* init_start */" << '\n';
	num_particles = 50;

	default_random_engine gen;

	normal_distribution<double> dist_x(x,std[0]);
	normal_distribution<double> dist_y(y,std[1]);
	normal_distribution<double> dist_theta(theta,std[2]);
	std::cout << "/* init_1 */" << '\n';
	Particle p1;
	std::cout << "/* init_2 */" << '\n';
	for(int i = 0;i<num_particles;i++){
		p1.x = dist_x(gen);
		p1.y = dist_y(gen);
		p1.theta = dist_theta(gen);
		p1.weight = 1.0;
		p1.id = i;

		particles.push_back(p1);
		weights.push_back(p1.weight);
		is_initialized = true;
	}
	std::cout << "/* init_end */" << '\n';
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::cout << "/* predict_start */" << '\n';
	default_random_engine gen;
	std::cout <<"p1_x="<<particles[0].x<<"p1_y="<<particles[0].y<<"p1_theta="<<particles[0].theta<< '\n';
	std::cout<<"yaw_rate="<<yaw_rate<<'\n';
	for(int i=0;i<num_particles;i++){
		Particle p = particles[i];
		if(yaw_rate==0){
			p.x = p.x+velocity*delta_t*cos(p.theta);
			p.y = p.y +velocity*delta_t*sin(p.theta);;
			p.theta = p.theta;
		}else{
			p.x = p.x+(velocity/yaw_rate)*(sin(p.theta+yaw_rate*delta_t)-sin(p.theta));
			p.y = p.y +(velocity/yaw_rate)*(cos(p.theta)-cos(p.theta+yaw_rate*delta_t));
			p.theta = p.theta+yaw_rate*delta_t;
		}

		std::cout <<"x="<<p.x<<"y="<<p.y<<"theta="<<p.theta<< '\n';
		normal_distribution<double> dist_x(p.x,std_pos[0]);
		normal_distribution<double> dist_y(p.y,std_pos[1]);
		normal_distribution<double> dist_theta(p.theta,std_pos[2]);

		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);

		particles[i] = p;
	}
	std::cout << "/* predict_end */" << '\n';
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	for(int i=0;i<observations.size();i++){
		double distance = 0.0;
		// int id =0;
		for(int j=0;j<predicted.size();j++){
			double temp_dist =dist(predicted[j].x,predicted[j].y,observations[i].x,observations[i].y);
			// std::cout << "/* temp_dist */"<<temp_dist << '\n';
			if(temp_dist<distance||distance==0.0){
				distance = temp_dist;
				observations[i].id = predicted[j].id;
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
	std::cout << "/* update_weights_start */" << '\n';
	for(int p=0;p<num_particles;p++){
		Particle p1 = particles[p];
		std::vector<LandmarkObs> obs_tran;

		std::cout << "/* tramsform */"<<"x="<<observations[0].x<<"y="<<observations[0].y<< '\n';
		std::cout <<"p1_x="<<particles[0].x<<"p1_y="<<particles[0].y<<"p1_theta="<<particles[0].theta<< '\n';
			//transform the observations to MAP's coordinate.
			for(int i=0;i<observations.size();i++){
				LandmarkObs obs;
				obs.x = p1.x+(cos(p1.theta)*observations[i].x)-(sin(p1.theta)*observations[i].y);
				obs.y = p1.y+(sin(p1.theta)*observations[i].x)+(cos(p1.theta)*observations[i].y);
				std::cout << "obs_x="<<obs.x<<"obs_y="<<obs.y<< '\n';
				obs_tran.push_back(obs);
			}


			std::cout << "/* find */"<< obs_tran[0].id<<"x="<<obs_tran[0].x <<"y="<<obs_tran[0].y<< '\n';
			//find the possible avaliable map_landmark
			std::vector<LandmarkObs> predicted;
			for(int j=0;j<map_landmarks.landmark_list.size();j++){
				double distance = dist(map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f,p1.x,p1.y);
				if(distance <= sensor_range){
					LandmarkObs predict_mark;
					predict_mark.id = map_landmarks.landmark_list[j].id_i;
					predict_mark.x = map_landmarks.landmark_list[j].x_f;
					predict_mark.y = map_landmarks.landmark_list[j].y_f;
					predicted.push_back(predict_mark);
				}
			}

			std::cout << "/* association */" << '\n';
			//set association between observation and predicted
			dataAssociation(predicted, obs_tran);

			std::cout << "/* update */" << '\n';
			//update weights
			double sig_x = std_landmark[0];
			double sig_y = std_landmark[1];
			double w=1.0;
			for(int n=0;n<obs_tran.size();n++){
				double x = obs_tran[n].x;
				double y = obs_tran[n].y;
				// std::cout << "w"<< n << '\n';
				// std::cout << "obs_tran_id="<< obs_tran[n].id << '\n';
				double u_x = map_landmarks.landmark_list[obs_tran[n].id-1].x_f;

				double u_y = map_landmarks.landmark_list[obs_tran[n].id-1].y_f;

				w = w*(1.0/(exp(((x-u_x)*(x-u_x)/(2*sig_x*sig_x))+((y-u_y)*(y-u_y)/(2*sig_y*sig_y)))*2*M_PI*sig_x*sig_y));
			}
			std::cout << "/* w =  */"<< w << '\n';
			particles[p].weight = w;
			weights[p] = w;
	}
	std::cout << weights[1] << '\n';
	std::cout << "/* update_weights_end */" << '\n';
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
std::cout << "/* resample_start */" << '\n';
	default_random_engine gen;
	std::discrete_distribution<int> distribution(weights.begin(),weights.end());

	std::vector<Particle> resample_particles;
	for(int i=0;i<num_particles;i++){
		resample_particles.push_back(particles[distribution(gen)]);
	}
	// std::cout << "x=" <<resample_particles[0].x<<"y="<<resample_particles[0].y<<"theta="<<resample_particles[0].theta<< '\n';
	particles = resample_particles;
	std::cout << "x=" <<particles[0].x<<"y="<<particles[0].y<<"theta="<<particles[0].theta<< '\n';
std::cout << "/* resample_end */" << '\n';
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
