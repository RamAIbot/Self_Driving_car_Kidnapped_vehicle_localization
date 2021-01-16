# Kidnapped Vehicle Localization

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="Capture.JPG" alt="image"/>

<h2> Overview </h2>
<p> The project focusses on solving the kidnapped vehicle problem using the Particle filters approach in a map which contains many poles placed in random locations. The robot is placed with sensors which measures the distance from the robot and the nearest poles with the sensor range. The robot doesn't match any reading with the corresponding poles. It gives the overall observations within the sensor range as input to the filter.We also provide the car's velocity and yaw rate with respect to x axis as the input for prediction step. Initially to get the robot's initial position and orientation we use the GPS measurement with some Gaussian noise as the first input. We use the bicycle motion model to predict the motion of particles as the vehicle moves. The bicycle motion model is the simple approximation of the vehcile,hence it is used.</p>

<h2> Algorithmic overall flow </h2>

<img src="" alt="flow"/>

<UL>
    <LI> The robot is initialized with the GPS cooridnates. The position and orientation values are initialized with some Gaussian noise </LI>
    <LI> Now the particles are initialized with Normal distribution. The Particles contains 3 values namely the x position, y position and orientation with respect to x.The weights of all the particles are initially set to 1.0 to provide uniform distribution to the inital belief.</LI>
    <LI> Now the prediction step is done. The robot uses the bicycle motion model to estimate the motion of all the particles. The velcoity and yaw rate of the robot is used to the particle next position along with delta t. There is also some noise in the particle movement characterized by the normal distribution. </LI>
    <LI> In particle filter localization problem the map of the environment in which the robot is navigating is previously known. So for the update step we provide the all the landmarks locations of the map, along with pole measurement distance which is obtained from the car's sensors. The weights are being updated with the help of multivariate Gaussian distribution. The weights are further normalized to be taken as probability for the resampling step </LI>
    <LI> Finally the particles with the best match are chosen by the resampling step. We use the resampling wheel approach to do this. These steps will be repeated continuously as the car moves.</LI>
</UL>
