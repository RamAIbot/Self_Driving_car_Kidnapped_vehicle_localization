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
   
  cout<<"Initilaiztion particle start"<<endl;
  num_particles = 10;  // TODO: Set the number of particles
  std::default_random_engine gen;
  if(!is_initialized)
  {
  	normal_distribution<double> dist_x(x,std[0]);
  	normal_distribution<double> dist_y(y,std[1]);
  	normal_distribution<double> dist_theta(theta,std[2]);
  	
  	for (int i=0;i<num_particles;i++)
  	{
  	Particle p;
  	p.x = dist_x(gen);
  	p.y = dist_y(gen);
  	p.theta = dist_theta(gen);
  	p.id = i;
  	p.weight = 1.0;
  	particles.push_back(p);
  	weights.push_back(1.0);
  	//cout<<"Particle "<<i<<p.x<<" "<<p.y<<" "<<p.theta<<endl;
  	//IMPORTANT
  	is_initialized=true;
  	}
  }
  
	//cout<<"Initilaiztion particle ends"<<endl;
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
   //However, adding noise to the particles at the end ensures that all they move is somewhat random fashion and cover more possibilities. This is essentially adding noise to the speed and yaw rate because that's how you determine the position of the prediction. You don't want all the particles to converge to the same spot at the end.
   //so we don't add noise to velocity and yaw_rate
   
   std::default_random_engine gen;
   cout<<"Prediction start"<<endl;
   cout<<particles.size()<<endl;
   for (int i=0;i<particles.size();i++)
   {
   	if(fabs(yaw_rate) > 0.00001)
   {
   	particles[i].x = particles[i].x + (velocity/yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
	particles[i].y = particles[i].y + (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
	particles[i].theta = particles[i].theta + (yaw_rate * delta_t);
   }
   else
   {
   	particles[i].x = particles[i].x + (velocity * delta_t * cos(particles[i].theta));
   	particles[i].y = particles[i].y + (velocity * delta_t * sin(particles[i].theta));
   	particles[i].theta = particles[i].theta;
   }
   
   normal_distribution<double> x_noise(particles[i].x,std_pos[0]);
   normal_distribution<double> y_noise(particles[i].y,std_pos[1]);
   normal_distribution<double> theta_noise(particles[i].theta,std_pos[2]);
   double noisex = x_noise(gen);
   double noisey = y_noise(gen);
   double noisetheta = theta_noise(gen);
   
   particles[i].x = particles[i].x + noisex;
   particles[i].y = particles[i].y + noisey;
   particles[i].theta = particles[i].theta + noisetheta;
   }
   
	//cout<<"Prediction ends"<<endl;
	//cout<<particles.size()<<endl;
}

double ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations,
									 Particle& particle,double std_landmark[],int particle_no) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
   
   //For visualization
   std::vector<int> associations;
   std::vector<double> sense_x;
   std::vector<double> sense_y;
   
   double x_part = particle.x;
   double y_part = particle.y;
   double theta_part = particle.theta;
   
   double tot_weight=1;
   //cout<<"dataAssociation start"<<endl;
   
   for(int i=0;i<observations.size();i++)
   {
   	double x_obs = x_part + (cos(theta_part) * observations[i].x) - (sin(theta_part) * observations[i].y);
   	double y_obs = y_part + (sin(theta_part) * observations[i].x) + (cos(theta_part) * observations[i].y);
   	double best_obs = 10;
   	double best_obs_id = 10;
   	int landmark_number=0;
   	for(int j=0;j<predicted.size();j++)
   	{
   		//double x_pred = predicted.at(j).x;
   		//double y_pred = predicted.at(j).y;
   		
   		
   		double dist = sqrt((predicted.at(j).x - x_obs)*(predicted.at(j).x - x_obs) + (predicted.at(j).y - y_obs)*(predicted.at(j).y - y_obs));
   		
   		if(dist < best_obs)
   		{
   			best_obs = dist;
   			best_obs_id = predicted.at(j).id;
   			landmark_number = j;
		}
		
	}
	observations.at(i).id = best_obs_id;
	double weight = multiv_prob(std_landmark[0],std_landmark[1],x_obs,y_obs,predicted[landmark_number].x,predicted[landmark_number].y);
	cout<<"Weight for each "<<weight<<endl;
	tot_weight *= weight;
	
	
	
	
	//For visualization
	associations.push_back(best_obs_id);
	sense_x.push_back(x_obs);
	sense_y.push_back(y_obs);
   }
   
   //Set assosciation
   SetAssociations(particle, associations, sense_x, sense_y);
	//cout<<"dataAssociation ends"<<endl;
   particle.weight = tot_weight;
   weights[particle_no] = tot_weight;
   //cout<<"TOt weight "<<tot_weight<<endl;
   return tot_weight;
}

double ParticleFilter::multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y)
{
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
  //cout<<"Weight at multiv "<<weight<<endl;
  return weight;
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
   cout<<"updateWeights start"<<endl;
   double weight_sum=0;
   cout<<particles.size()<<endl;
   for(int i=0;i<particles.size();i++)
   {
   	
   	
	double x_part = particles[i].x;
	double y_part = particles.at(i).y;
	double theta_part = particles.at(i).theta;
	
	
	
	vector<LandmarkObs> landmarks_predictions;
	
	for(int j=0;j<map_landmarks.landmark_list.size();j++)
	{
		//double x_land = map_landmarks.landmark_list[j].x_f;
		//double y_land = map_landmarks.landmark_list[j].y_f;
		//int land_id = map_landmarks.landmark_list[j].id_i;
		
		if(fabs(map_landmarks.landmark_list[j].x_f - x_part) <=sensor_range && fabs(map_landmarks.landmark_list[j].y_f - y_part) <= sensor_range)
		{
			//dont use poaranthesis
			landmarks_predictions.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f});
		}
	}
					  
//	vector<LandmarkObs> transformed_obs;
//	for(int k=0;k<observations.size();k++)
//	{
//		
//	
//		double xm = x_part + (cos(theta_part) * observations[k].x) - (sin(theta_part) * observations[k].y);
//		double ym = y_part + (sin(theta_part) * observations[k].x) + (cos(theta_part) * observations[k].y);
//		int id = observations[k].id;
//		transformed_obs.push_back(LandmarkObs{id,xm,ym});	
//		
//		
//	}
	vector<LandmarkObs> transformed_obs = observations;
	double tot_weight = dataAssociation(landmarks_predictions,transformed_obs,particles[i],std_landmark,i);
	
	
	//cout<<"total weight "<<tot_weight<<endl;
	weight_sum += tot_weight;
   }
   
   //Normalization
   for(int i=0;i<particles.size();i++)
   {
   	particles.at(i).weight/=weight_sum;
   	weights[i] /= weight_sum;
   	
   	//cout<<"particle weight "<<particles.at(i).weight<<endl;
   	cout<<"weight[i]"<<weights[i]<<endl;
   	
   }
	//cout<<"updateWeights ends"<<endl;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   cout<<"Resampling start"<<endl;
   
//   std::vector<double> weights_resampled;
//   for(int i=0;i<particles.size();i++)
//   {
//   	weights_resampled.push_back(particles[i].weight);
//   }
   
   std::default_random_engine generator;
   std::discrete_distribution<> d(weights.begin(),weights.end());
   
   
   std::vector<Particle> particles_resampled;
   
   for(int i=0;i<particles.size();i++)
   {
   	int sample = d(generator);
   	//particles[sample].id = i;
   	particles_resampled.push_back(particles[sample]);
   	particles_resampled[i].id = i;
   	cout<<"id"<<particles_resampled[i].id<<endl;
   	cout<<"res weight"<<particles_resampled[i].weight<<endl;
   	cout<<"res x "<<particles_resampled[i].x<<endl;
   	cout<<"res y "<<particles_resampled[i].y<<endl;
   	cout<<"res theta "<<particles_resampled[i].theta<<endl;
   }
  
   particles = particles_resampled;
	//cout<<"Resampling ends"<<endl;
	
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
