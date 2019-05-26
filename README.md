## Kidnapped Vehicle (Sparse Localization)


[//]: # (Image References)

[image1]: ./IMG/PF-flow-diagram.png "Particle Filter flow diagram"
[image2]: ./IMG/Update-step.png "Bayesian Posterior diagram"


### Introduction

In this project I implemented a 2-D particle filter in C++ which was used to localize a vehicle, in real-time, as it traversed a mapped space. The particle filter, at initialization, was given a map and some localization information (analogous to what a GPS would provide) and there after at each time step received sensor data observed from surrounding landmarks and control data such as pose, yaw rate etc.. of the vehicle.

### Overview

![alt text][image1]

#### Initialization

At the initialization step the car's position is estimated using data from GPS. The subsequent steps in the process refined this estimate to localize the vehicle. The particle filter was intialized by sampling from a Gaussian distribution, that took into account Gaussian sensor noise, around the initial GPS position and heading estimates.

```cpp

      ...

       // random noise distributions with zero mean and std of respective variables
  	  std::default_random_engine gen;
  	  std::normal_distribution<double> dist_x(0, std[0]);
  	  std::normal_distribution<double> dist_y(0, std[1]);
  	  std::normal_distribution<double> dist_theta(0, std[2]);  

  	  num_particles = 70;  // Set the number of particles
  	  Particle particle;
  	    
  	  for (unsigned int i=0; i< num_particles; i++) {

  	    particle.id = i+1;
  	    particle.x = x + dist_x(gen);
  	    particle.y = y + dist_y(gen);
  	    particle.theta = theta + dist_theta(gen);
  	    particle.weight = 1;
  	    
	    ...

```  

#### Prediction

During the prediction step control input (yaw rate & velocity) was used to update all particles. Using the bicycle motion model this data was used to predict the position / displacement of the particles in the next time step. Due to sensor uncertainty Gaussian noise is added to both the yaw and displacement of each particle.



```cpp
	    
      ...

	    if (yaw_rate == 0) { 
	      particles[i].x += cos(particles[i].theta) * velocity * delta_t;
	      particles[i].y += sin(particles[i].theta) * velocity * delta_t;
	    }
	    
	    else { // x,y displacement update for non zero yaw rate
	      particles[i].x += velocity/yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
	      particles[i].y += velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
	    }
	    particles[i].theta += yaw_rate * delta_t;

	    
	    // adding Gaussian noise to updated displacement and heading
	    particles[i].x += dist_px(gen);
	    particles[i].y += dist_py(gen);
	    particles[i].theta += dist_ptheta(gen);

      ...
      
```  

#### Update

During the update step, the particle weights are updated using map landmark positions and feature measurements. Based on the range of the sensors only landmarks within this distance from each particle are considered. 

I found that using the differences between the landmark x, y positions and that of the particles were sufficent to capture relative proximity and resulted in less computational overhead than to calculate the euclidean distances for all in-range landmarks for every particle. This provided a significant improvement in the execution efficiency of the particle filter.

```cpp
      
      ...

      // create vector to store landmarks within range of sensor of each particle (predicted state)
      vector <LandmarkObs> in_range;

      for (unsigned int j=0; j < map_landmarks.landmark_list.size(); j++) {
        
        int landmark_id = map_landmarks.landmark_list[j].id_i;
        float landmark_x = map_landmarks.landmark_list[j].x_f;
        float landmark_y = map_landmarks.landmark_list[j].y_f;

        // checking for landmark x,y coordinates within sensor range of each predicted state
        if (fabs(landmark_x - particles[i].x) <= sensor_range &&
           fabs(landmark_y - particles[i].y) <= sensor_range) {
           
      ...

```

 Prior to associating the landmarks observed from the car's perspective to the landmarks within the sensors range their coordinates were transformed to the map's point-of-view.

```cpp

   	    ...

        // transform car observation x coordinate to map x coordinate    
        double t_obs_x = particles[i].x + cos(particles[i].theta) * observations[k].x - sin(particles[i].theta) * observations[k].y;

        // transform car observation x coordinate to map y coordinate    
        double t_obs_y = particles[i].y + sin(particles[i].theta) * observations[k].x + cos(particles[i].theta) * observations[k].y;

        ...
```

The association of predicted (in-range) values with observation values was determined by calculating the sum of the absolute difference between their x and y values for each particle. Once again this proved to be sufficient for the association task and much less computationally intensive than calculating the euclidean distances.

```cpp

          ...

          double shortest_dist = std::numeric_limits<double>::infinity();

          for (unsigned int j=0; j < predicted.size(); j++) {
            double diff_dist = fabs(predicted[j].x - observations[i].x) + fabs(predicted[j].y - observations[i].y);
            
            if (diff_dist < shortest_dist) {
              shortest_dist = diff_dist;
            //Assigning the landmark id to the "observation" closest to each in-range landmark 
              observations[i].id = predicted[j].id; 
          ...        

```

Once the closest landmark to each particle is determined the particle's weight was updated using the multi-variate Gaussian distribution. The weights assigned to the particle are inversely proportional to the distance between the observed and closest predicted (in-range) landmark. 

```cpp

        ...

        // re-initialize before calculating weights of new predicted state
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

        ...

```

![alt text][image2]

The new set of particles represents the Bayes filter posterior probability. This results in a refined estimate of the vehicle's position based on input evidence.



#### Resampling

Resampling is executed M times *(M is range of 0 to length of particle array)* drawing a particle i (i is the particle index) proportional to its weight. See code snippet below.

```cpp
      ...

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

        ...

```  

Once resampled the new particle set is returned.



# Udacity's original README

### Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

#### Submission
All you will need to submit is your `src` directory. You should probably do a `git pull` before submitting to verify that your project passes the most up-to-date version of the grading code (there are some parameters in `src/main.cpp` which govern the requirements on accuracy and run time).

### Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

### Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

### Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

### Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

### Success Criteria
If your particle filter passes the current grading code in the simulator (you can make sure you have the current version at any time by doing a `git pull`), then you should pass!

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.

### How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
