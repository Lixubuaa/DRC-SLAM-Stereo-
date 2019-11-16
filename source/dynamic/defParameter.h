// Parameters for KITTI dataset

// The number of superpixel
int superpixelTotal = 300;

// The number of iterations
int outerIterationTotal = 3;
int innerIterationTotal = 3;

// Weight parameters
double lambda_pos = 500.0;
double lambda_depth = 3000.0;//yuan 2000
double lambda_bou = 1000.0;
double lambda_smo = 400.0; 

// Inlier threshold
double lambda_d = 5.0;

// Penalty values
double lambda_hinge = 100;
double lambda_occ = 300;
double lambda_pen = 500;

