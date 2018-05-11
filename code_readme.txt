Here is data flow:

- I use the first measurements to initialize the state vectors and covariance matrices.
- Upon receiving a measurement after the first, the algorithm predicts object position to the current timestep and then update the prediction using the new measurement.
- the algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type.
- the update and predict function are implemented in kalman_filter.cpp for both radar and Lidar sensors. 
- The algorithm uses a utility class named tools in tools.cpp for calculating Jacobian matrix and RMSE.
- I also use normalization for Radar update (line 99-102)
- for optimization, I use a flag to compute the H.Transpose only once for the Lidar update and use the precalculated matrix in future updates. see line 41-42 of Kalman_filter.cpp
- I tried different values for the initial x_ and found out 4,4,1,1, gives good result.
- The final RMSE are 0.09,0.08,0.45,0.44  (X,Y,VX,VY)






