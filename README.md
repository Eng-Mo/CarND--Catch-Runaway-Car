
# CarND Extended Kalman Filter Project

[//]: # (Image References)
[image1]: ./Output_images/NIS_L.png
[image2]: ./Output_images/NIS_R.png
[image3]: ./Output_images/UKF_L_DS1.png
[image4]: ./Output_images/UKF_L_DS2.png
[image5]: ./Output_images/UKF_R_DS1.png
[image6]: ./Output_images/UKF_R_DS2.png
[image7]: ./Output_images/UKF_LR_DS1.png
[image8]: ./Output_images/UKF_LR_DS2.png
[image9]: ./Output_images/EKF_UKF.png

---
This project is an implementation of a sensor fusion algorithm based on Unscented Kalman filter in order to estimate the position of moving car in constant circular motion and control hunter car to catch it.

---
### Project Goals
This project aims to solve the non-linearity problem in moving object by using the constant turn rate and velocity magnitude motion model(CTRV) in order to predict position of car after time `t`. The strategy is by using the trigonometry and circular motion the center of the runaway car can be calculated by taking 3 points on the circular path. The time between taking each point `dt=.1`. then I controlled the hunter car to the center. the distance from the center to any point on the circle path are equal, hence I calculated the time `t` heading the hunter car after distance radius then predict the position of the runaway car after time `t`.


### Dependencies
* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

### Build instructions
1. Clone this repo.
2. Make a build directory: mkdir build && cd build
3. Compile: cmake .. && make
4. Run it: ./catchCar

---

## Results

The UKF can predict the position of object after time `t` accurately and the hunter car can catch the run away car if the motion is known but more examination required if the motion of the object unknown.
Link to video: https://www.youtube.com/watch?v=qxgfUPMjzMg
## References
1. http://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm
2. http://mathforum.org/library/drmath/view/54323.html