Using MPU9250 with opentrack for headtracking

Most important - calibrate your Magnetometer using 
https://github.com/YuriMat/MagMaster

Use mpu9250_calibrationmagmaster.ino firmware for that.

You will get calibration matrix and bias vector, use them to edit

mpu9250_tracking.ino
mpu9250_magmaster_test.ino firmwares.

You have to edit this function, the values of calibration_matrix and bias:

~~~~
void transformation(float uncalibrated_values[3])    
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    {1.511, 0.058, -0.037},
    {-0.025, 1.473, -0.127},
    {0.043, 0.126, 1.237}  
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    124.152,
    206.036,
    -107.591
  };  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

~~~~

Using MagViewer.exe and mpu9250_magmaster_test.ino firmware
you can check your calibration - when you rotate your
tracker in all axes you should get nice sphere.

To use with opentrack load mpu9250_tracking.ino firmware and select
HATIRE protocol/plugin in opentrack.
