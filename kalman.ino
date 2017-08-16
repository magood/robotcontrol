#include <BasicLinearAlgebra.h>
#include <Geometry.h>

//Code migrated from python/numpy.
//You apparently need to specify the size of all these matrices at compile time, ouch.
//void kalman_step(Matrix x, Matrix F, Matrix u, Matrix P, Matrix Z, Matrix H, Matrix R, Matrix I) {
  //One Kalman Filter prediction/measurement iteration.
  //All parameters are matrices.
  //x, u, and z should be column-vectors as matrices
  //Have to confirm, but I believe this function will mutate the input params.  Maybe not.  I dunno.
  //prediction
//  x = F * x + u
//  P = F.dot(P).dot(F.T)
//      
//  # measurement update
//  #y: Innovation residual (difference between actual and predicted measurements)
//  y = Z - H.dot(x)
//  #S: Innovation covariance
//  S = H.dot(P).dot(H.T) + R
//  #K: Kalman gain
//  K = P.dot(H.T).dot(S.I)
//  x = x + K.dot(y)
//  P = (I - K.dot(H)).dot(P)
//}

