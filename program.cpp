//Setting up some windows header files needed for the program
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <conio.h>
#include <Windows.h>

	
using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )
#define PI (3.141595654)

// Adding some required header files for vision and simulation functions
#include "image_transfer.h"
#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"
#include "images.h" //Custom header file for setting images

extern robot_system S1;

void set_simulation_model(int& N_obs, double x_obs[], double y_obs[], double size_obs[], double& D,
	double& Lx, double& Ly, double& Ax, double& Ay, double& alpha_max, int& n_robot);

int set_initial_inputs(double& x0, double& y0, double& theta0, int& pw_l, int& pw_r, int& pw_laser,
	int& laser, double& max_speed, double& opponent_max_speed, double& light, double& light_gradient,
	double& light_dir, double& image_noise, int& pw_l_o, int& pw_r_o, int& pw_laser_o, int& laser_o);

int simulate_robots(double& x0, double& y0, double& theta0, double& D, int& pw_l, int& pw_r,
	int& pw_laser, int& laser, int& pw_l_o, int& pw_r_o, int& pw_laser_o, int& laser_o, double& max_speed, double& opponent_max_speed,
	double& light, double& light_gradient, double& light_dir, double& image_noise,
	double& a1x, double& a1y, double& a2x, double& a2y, double& b1x, double& b1y, double& b2x,
	double& b2y, i2byte label_num[], int& tvalue, int& nlabels, image& rgb0, image& rgb,
	image& a, image& b, image& label, image& hue_label, int color[], double& theta_r, double x_obs[],
	double y_obs[]);

int control_manual_opponent(int& pw_l_o, int& pw_r_o, double& dpw, int& pw_laser_o, int& laser_o,
	double& opponent_max_speed);

int track_and_control(i2byte label_num[], image& label, double& a1x, double& a1y, double& a2x,
	double& a2y, double& b1x, double& b2x, double& b1y, double& b2y, double& D, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise, image& a, image& rgb0, image& rgb);

int defence(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y,
	double& b1x, double& b1y, double& b2x, double& b2y, double& D, int& pw_l, int& pw_r, double x_obs[], double y_obs[],
	int& pw_laser, int& laser, double& max_speed, double& opponent_max_speed, double& light,
	double& light_gradient, double& light_dir, double& image_noise);

int detect_objects(i2byte& label_num, int& tvalue, int& nlabels, image& rgb0, image& rgb,
	image& a, image& b, image& label, image& hue_label, int& color);

int label_objects(int& tvalue, int& nlabels, image& rgb0, image& rgb, image& a, image& b,
	image& label);

int select_object(i2byte& label_num, int& tvalue, int& nlabels,
	image& rgb0, image& rgb, image& a, image& b, image& label, image& hue_label, int& color);

int Hue_image_creator(image& rgb0, image& hue_label);

void calculate_HSV(ibyte& R, ibyte& G, ibyte& B, i2byte& hue, double& sat, double& value);

int search_object(i2byte& nlabel, image& label, int is, int js);

int controller(double& a1x, double& a1y, double& a2x, double& a2y, double& b1x, double& b1y,
	double& b2x, double& b2y, double& D, int& pw_l, int& pw_r, double x_obs[], double y_obs[],
	int& pw_laser, int& laser, double& max_speed, double& opponent_max_speed, double& light,
	double& light_gradient, double& light_dir, double& image_noise);

void avoid_obstacle(double& d_robs, double& dox, double& doy, double& difx, double& dify, double& difbx, double& difby,
	double& a1x, double& a1y, double& a2x, double& a2y, double& b1x, double& b1y,
	double& b2x, double& b2y, int& pw_l, int& pw_r, double x_obs[], double y_obs[], int& pw_laser,
	int& laser, double& max_speed, double& opponent_max_speed, double& light, double& light_gradient,
	double& light_dir, double& image_noise);

void right_limit(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y, double& D, double& b1x, double& b1y, double& b2x, double& b2y, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise);

void up_limit(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y, double& D, double& b1x, double& b1y, double& b2x, double& b2y, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise);

void left_limit(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y, double& D, double& b1x, double& b1y, double& b2x, double& b2y, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise);

void bottom_limit(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y, double& D, double& b1x, double& b1y, double& b2x, double& b2y, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise);


int main()
{
	//Declaring simulation parameters and robot variables

	double x0{}, y0{}, theta0{}, max_speed{}, opponent_max_speed{}; //robot position and speed variables
	int pw_l{}, pw_r{}, pw_laser{}, laser{}; //pulse width values of Robot A
	double light{}, light_gradient{}, light_dir{}, image_noise{}; // Lighting parameters
	double width1{}, height1{};
	int N_obs{}, n_robot{}; //No of robots and obstacles
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max; //Robot model dimension variables
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o; //pulse width values of Robot B 


	//Setting image variables
	image rgb, rgb0;// RGB images
	image a, b; //Grey images
	image label, hue_label;// Label images 
	int nlabels; //number of labels
	i2byte label_num[5];
	static int tvalue = 180; //Threshold value

	//colors to track
	const int no_colors = 9;
	int color[no_colors];

	// Hue labels for specific colors
	color[1] = 4;
	color[2] = 5;
	color[3] = 1;
	color[4] = 3;

	/*Declaring variables for robot centroids
	a1x, a1y - blue centroid
	a2x, a2y - purple centroid
	b1x, b1y - red centroid
	b2x, b2y - green centroid
	*/
	double a1x{}, a1y{}, a2x{}, a2y{};
	double b1x{}, b1y{}, b2x{}, b2y{};

	//angle of robot
	double theta_r = 0.0;

	//for simulation
	width1 = 640;
	height1 = 480;

	//Dimensions of image acquired from simulation
	const int IMAGE_WIDTH = 640;
	const int IMAGE_HEIGHT = 480;

	set_simulation_model(N_obs, x_obs, y_obs, size_obs, D, Lx, Ly, Ax, Ay, alpha_max, n_robot);

	cout << "\npress space key to begin program.";
	pause();

	// activating the vision library
	activate_vision();

	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_Bnew.bmp", "robot_A.bmp", "background.bmp", "obstacle_orange.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	set_initial_inputs(x0, y0, theta0, pw_l, pw_r, pw_laser, laser, max_speed, opponent_max_speed,
		light, light_gradient, light_dir, image_noise, pw_l_o, pw_r_o, pw_laser_o, laser_o);

	set_images(IMAGE_WIDTH, IMAGE_HEIGHT, rgb, rgb0, a, b, label, hue_label);

	acquire_image_sim(rgb0);
	view_rgb_image(rgb0);
	pause();
	cout << "Simulation activated. Press space to begin.." << endl;

	simulate_robots(x0, y0, theta0, D, pw_l, pw_r, pw_laser, laser, pw_l_o, pw_r_o, pw_laser_o, laser_o, max_speed, opponent_max_speed, light, light_gradient,
		light_dir, image_noise, a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y, label_num, tvalue, nlabels, rgb0, rgb, a, b,
		label, hue_label, color, theta_r, x_obs, y_obs);

	free_images(rgb, rgb0, a, b, label, hue_label);

	deactivate_vision();

	deactivate_simulation();

	cout << "\ndone.\n";

	return 0;

}
int simulate_robots(double& x0, double& y0, double& theta0, double& D, int& pw_l, int& pw_r,
	int& pw_laser, int& laser, int& pw_l_o, int& pw_r_o, int& pw_laser_o, int& laser_o, double& max_speed, double& opponent_max_speed,
	double& light, double& light_gradient, double& light_dir, double& image_noise,
	double& a1x, double& a1y, double& a2x, double& a2y, double& b1x, double& b1y, double& b2x,
	double& b2y, i2byte label_num[], int& tvalue, int& nlabels, image& rgb0, image& rgb,
	image& a, image& b, image& label, image& hue_label, int color[], double& theta_r, double x_obs[],
	double y_obs[])
{
	//Declaring some variables for calculating angles
	double prev_angle_r, current_angle_r, diff_current_n_prev_angle_r;

	//initial values
	pw_l = 1500;
	pw_r = 1500;

	double dpw = 250;

	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	Sleep(10);
	// Find the required objects and mark centroid to track movements
	detect_objects(label_num[1], tvalue, nlabels, rgb0, rgb, a, b, label, hue_label, color[1]);
	detect_objects(label_num[2], tvalue, nlabels, rgb0, rgb, a, b, label, hue_label, color[2]);
	detect_objects(label_num[3], tvalue, nlabels, rgb0, rgb, a, b, label, hue_label, color[3]);
	detect_objects(label_num[4], tvalue, nlabels, rgb0, rgb, a, b, label, hue_label, color[4]);

	//Finding centroids of the 4 robot colors
	centroid(a, label, label_num[1], a1x, a1y);
	centroid(a, label, label_num[2], a2x, a2y);
	centroid(a, label, label_num[3], b1x, b1y);
	centroid(a, label, label_num[4], b2x, b2y);
	cout << "\nObject detection and centroid marking successful. Press space to continue...";

	//pause();

	cout << "\n\n\npress T to end simulation\n";

	while (1) {

		control_manual_opponent(pw_l_o, pw_r_o, dpw, pw_laser_o, laser_o, opponent_max_speed);

		track_and_control(label_num, label, a1x, a1y, a2x, a2y, b1x, b2x, b1y, b2y, D, pw_l, pw_r, x_obs,
			y_obs, pw_laser, laser, max_speed, opponent_max_speed, light, light_gradient, light_dir,
			image_noise, a, rgb0, rgb);

		view_rgb_image(rgb);

		// acquire an image from a video source (RGB format)
		acquire_image_sim(rgb0);

		// label objects
		label_objects(tvalue, nlabels, rgb0, rgb, a, b, label);

	}
	return 0;
}
int detect_objects(i2byte& label_num, int& tvalue, int& nlabels, image& rgb0, image& rgb,
	image& a, image& b, image& label, image& hue_label, int& color)
{
	acquire_image_sim(rgb0);

	//get the labels for the image
	label_objects(tvalue, nlabels, rgb0, rgb, a, b, label);

	select_object(label_num, tvalue, nlabels, rgb0, rgb, a, b, label, hue_label, color);
	return 0;
}

int label_objects(int& tvalue, int& nlabels, image& rgb0, image& rgb, image& a, image& b,
	image& label)
{

	// convert RGB image to a greyscale image
	copy(rgb0, a);

	// scale the image to enhance contrast
	scale(a, a);

	// use threshold function to make a binary image (0,255)
	threshold(a, a, tvalue);

	// invert the image
	invert(a, a);

	// perform an erosion function to remove noise (small objects)
	erode(a, b);

	// perform a dialation function to fill in 
	// and grow the objects
	dialate(b, a);

	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(a, label, nlabels);
	return 0; 					// no errors
}

int select_object(i2byte& label_num, int& tvalue, int& nlabels,
	image& rgb0, image& rgb, image& a, image& b, image& label, image& hue_label, int& color) {

	i2byte* pl;	// pointer to the original label image
	double xc, yc;	//rough centroids calculated at the hue label image to get the location of the colored object


	copy(rgb0, a);
	// scale the image to enhance contrast
	scale(a, a);

	// use threshold function to make a binary image (0,255)
	threshold(a, a, tvalue);

	// invert the image
	invert(a, a);

	// perform an erosion function to remove noise (small objects)
	erode(a, b);

	// perform a dialation function to fill in 
	// and grow the objects
	dialate(b, a);

	copy(a, rgb); // threshold image is in a

	//calling to create the hue labelled image
	Hue_image_creator(rgb0, hue_label);

	// finding the approximate of the hue label image
	centroid(a, hue_label, color, xc, yc);

	// pointers to a label image
	pl = (i2byte*)label.pdata;



	// get the label value of the label image at the centre		of the required color
	label_num = *(pl + ((int)yc) * label.width + ((int)xc));
	return 0; // no errors
}

int Hue_image_creator(image& rgb0, image& hue_label) {
	ibyte B, G, R;
	ibyte* p;
	i2byte* pl;
	double sat, value; // saturation and value of HSV
	i2byte hue;
	i2byte l{};	//label for hue_label

	//pointers to the image files
	p = rgb0.pdata;
	pl = (i2byte*)hue_label.pdata;

	for (int j = 0; j < rgb0.height * rgb0.width; j++) {

		//reading values from rgb0(primary image)
		B = *p;
		G = *(p + 1);
		R = *(p + 2);


		//calling function to calculate Hue value for corresponding R G B values
		calculate_HSV(R, G, B, hue, sat, value);

		//TO DO refine the selection of colors
		if (value > 200.0) {
			l = 0;
		}
		if (sat < 0.35) {
			l = 0;
		}
		else if (hue >= 1 && hue <= 15 || hue <= 360 && hue >= 315 && sat > 0.4 && value < 200 && value > 100) {
			l = 1; // RED
		}
		else if (hue >= 38 && hue <= 67 && sat > 0.4 && value < 200 && value > 100) {
			l = 2; // YELLOW
		}
		else if (hue >= 68 && hue <= 173 && sat > 0.4 && value < 200 && value > 100) {
			l = 3; // GREEN
		}
		//else if (hue==0 && sat ==0 && value == 255) {
			//l = 4; // Greenish-yellow
		else if (hue == 240 && sat == 1 && value == 255) {
			l = 4; // BLUE
		}
		else if (hue >= 250 && hue <= 315 && sat > 0.3 && value < 200 && value > 100) {
			l = 5; // PURPLE/PINK
		}

		*pl = l;	// allocating hue to hue_label image
		//incrementing pointers
		p += 3;
		pl++;

	}	// end of for loop
	return 0;	//no errors
}

//calculate HSV from RGB
void calculate_HSV(ibyte& R, ibyte& G, ibyte& B, i2byte& hue, double& sat, double& value)
{
	int max, min, delta;
	double H;
	max = min = (int)R;

	if ((int)G > max) max = (int)G;
	if ((int)B > max) max = (int)B;

	if ((int)G < min) min = (int)G;
	if ((int)B < min) min = (int)B;

	delta = max - min;

	value = max;

	if (delta == 0) {
		sat = 0.0;
	}
	else {
		sat = delta / value;
	}

	if (delta == 0) {
		H = 0; // hue undefined, maybe set hue = -1
	}
	else if (max == (int)R) {
		H = (double)(G - B) / delta;
	}
	else if (max == (int)G) {
		H = (double)(B - R) / delta + 2;
	}
	else if (max == (int)B) {
		H = (double)(R - G) / delta + 4;
	}

	hue = (i2byte)(60 * H);

	if (hue < 0) hue += 360;
}

int search_object(i2byte& nlabel, image& label, int is, int js)
// search for a labeled object in an outward spiral pattern
// and inital search location (is,js)
// *** Please study this function carefully
// -- more upcoming assignment and project problems 
// are related to this
{
	i2byte* pl;
	double r, rmax, dr, s, smax, ds, theta;
	int i, j;

	// pointer to a label image
	pl = (i2byte*)label.pdata;

	// check if an object exists at the current location
	nlabel = *(pl + js * label.width + is);
	if (nlabel != 0) return 0;

	rmax = 60.0; // maximum radius of search (pixels)
	dr = 3.0; // radius divisions (pixels)
	ds = 3.0; // arc-length divisions (pixels)

	// search for a labeled object in an outward concentic ring pattern
	for (r = 1.0; r <= rmax; r += dr) {
		smax = 2 * 3.1416 * r; // maximum arc length
		for (s = 0; s <= smax; s += ds) {
			theta = s / r; // s = r*theta
			i = (int)(is + r * cos(theta));
			j = (int)(js + r * sin(theta));

			// limit (i,j) from going off the image
			if (i < 0) i = 0;
			if (i > label.width - 1) i = label.width - 1;
			if (j < 0) j = 0;
			if (j > label.height - 1) j = label.height - 1;

			//			*( b.pdata + j*b.width + i ) = 128; // check pattern

						// check if there is an object at location (i,j)
			nlabel = *(pl + j * label.width + i);
			if (nlabel != 0) return 0;
		}
	}

	return 0; // no errors
}


int controller(double& a1x, double& a1y, double& a2x, double& a2y, double& b1x, double& b1y,
	double& b2x, double& b2y, double& D, int& pw_l, int& pw_r, double x_obs[], double y_obs[],
	int& pw_laser, int& laser, double& max_speed, double& opponent_max_speed, double& light,
	double& light_gradient, double& light_dir, double& image_noise)

{

	int dpw = 350;
	double a1, b1, a, b, c, current_angle_r_screen, prev_angle_r, current_angle_r, diff_current_n_prev_angle_r;

	double difx, dify, difbx, difby, dox, doy; // difference between the centroid and current target
	double d_robs; // distacne between robot and obstacle
	double dist_f, dist_b;


	//simulation anc control time step
	double t = 0.0, dt = 0.5;		//time

	//a1x, a1y - blue centroid
	//a2x, a2y - purple centroid
	//b1x, b1y - red centroid
	//b2x, b2y - green centroid

	difx = abs(b2x - a2x);						//difference in x coordinates
	dify = abs(b2y - a2y);						//difference in y coordinates
	dist_f = sqrt(pow(difx, 2) + pow(dify, 2)); //distance between green and purple centroids
	difbx = abs(b1x - a1x);						//difference in x coordinates
	difby = abs(b1y - a1y);						//difference in y coordinates
	dist_b = sqrt(pow(difbx, 2) + pow(difby, 2)); // distance between red and blue centroids

	pw_l = 1500 - dpw;
	pw_r = 1500 + dpw;
	// setting inputs
	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);
	defence(difx, dify, difbx, difby, a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y, D, pw_l, pw_r, x_obs, y_obs,
		pw_laser, laser, max_speed, opponent_max_speed, light, light_gradient,
		light_dir, image_noise);

	// diff between obs and robot front
	dox = abs(a2x - x_obs[1]);	//difference in x coordinates
	doy = abs(a2y - y_obs[1]);  //difference in y coordinates

	// distance to outside of obstacle from front of our robot
	d_robs = sqrt(pow(dox, 2) + pow(doy, 2));

	current_angle_r = (atan2(a1y - a2y, a1x - a2x)) * 180 / PI;
	if (current_angle_r < 0)
	{
		a1 = 360 - abs(current_angle_r);
	}
	else {
		a1 = current_angle_r;
	}
	if (100 <= d_robs && d_robs <= 110) {
		avoid_obstacle(d_robs, dox, doy, difx, dify, difbx, difby, a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y, pw_l, pw_r, x_obs, y_obs, pw_laser, laser, max_speed,
			opponent_max_speed, light, light_gradient, light_dir, image_noise);
	}
	if (KEY('D')) {
		pw_l = 1500;
		pw_r = 1500;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		cout << b2x << "\t" << b2y << "\n";
	}
	if (a2x > 550) {
		right_limit(difx, dify, difbx, difby, a1x, a1y, a2x, a2y, D, b1x, b1y, b2x, b2y, pw_l, pw_r, x_obs, y_obs, pw_laser, laser, max_speed,
			opponent_max_speed, light, light_gradient, light_dir, image_noise);
	}
	if (a2y > 400)
		up_limit(difx, dify, difbx, difby, a1x, a1y, a2x, a2y, D, b1x, b1y, b2x, b2y, pw_l, pw_r, x_obs, y_obs, pw_laser, laser, max_speed,
			opponent_max_speed, light, light_gradient, light_dir, image_noise);

	if (a2x < 100) {
		left_limit(difx, dify, difbx, difby, a1x, a1y, a2x, a2y, D, b1x, b1y, b2x, b2y, pw_l, pw_r, x_obs, y_obs, pw_laser, laser, max_speed,
			opponent_max_speed, light, light_gradient, light_dir, image_noise);
	}
	if (a2y < 100)
	{
		bottom_limit(difx, dify, difbx, difby, a1x, a1y, a2x, a2y, D, b1x, b1y, b2x, b2y, pw_l, pw_r, x_obs, y_obs, pw_laser, laser, max_speed,
			opponent_max_speed, light, light_gradient, light_dir, image_noise);
	}

	return 0;
}

int defence(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y,
	double& b1x, double& b1y, double& b2x, double& b2y, double& D, int& pw_l_o, int& pw_r_o, double x_obs[], double y_obs[],
	int& pw_laser_o, int& laser_o, double& max_speed, double& opponent_max_speed, double& light,
	double& light_gradient, double& light_dir, double& image_noise) {


	{
		double theta; // angle of target, our robot,desired angle
		double angle; // interim angle correction
		double dpw = 250;

		// our robot angle
		angle = (atan2(a1y - a2y, a1x - a2x)) * 180 / PI;
		if (angle < 0)
		{
			theta = 360 - abs(angle);
		}
		else {
			theta = angle;
		}
		if (theta >= 358 && theta <= 360) theta = 0;

		if (b2x >= 320 && b2y >= 240) // Opponent robot in first quadrant

		{
			if (a2x < 300)
			{
				if (theta <= 360 && theta >= 90)
				{
					pw_l_o = 1500 - dpw;
					pw_r_o = 1500 - dpw;
					set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
					if (theta < 96)
					{
						pw_l_o = 1500;
						pw_r_o = 1500;
						set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
						pw_l_o = 1500 - dpw;
						pw_r_o = 1500 + dpw;
						set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
						if (110 <= a2y && a2y <= 130)
						{
							pw_l_o = 1500;
							pw_r_o = 1500;
							set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

						}
					}
				}
				if (theta <= 90 && theta >= 0)
				{
					pw_l_o = 1500 + dpw;
					pw_r_o = 1500 + dpw;
					set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,opponent_max_speed);
					if (theta > 85)
					{
						pw_l_o = 1500;
						pw_r_o = 1500;
						set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,opponent_max_speed);
						pw_l_o = 1500 - dpw;
						pw_r_o = 1500 + dpw;
						set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
						if (110 <= a2y && a2y <= 130)
						{
							pw_l_o = 1500;
							pw_r_o = 1500;
							set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

						}
					}
				}
			}

			if (a2x > 300 && a2y < 240)
			{
				if (KEY('X')) {
					pw_l_o = 1500;
					pw_r_o = 1500;
					set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
					cout << a1x << "\t" << a1y << "\t" << b2x << "\t" << b2y << "\t" << theta << "\n";
				}
				if (theta <= 270 && theta >= 0)
				{
					pw_l_o = 1500 - dpw;
					pw_r_o = 1500 - dpw;
					set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
					if (theta < 1)
					{
						pw_l_o = 1500;
						pw_r_o = 1500;
						set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
						pw_l_o = 1500 - dpw;
						pw_r_o = 1500 + dpw;
						set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
						if (110 <= a2x && a2x <= 130)
						{
							pw_l_o = 1500;
							pw_r_o = 1500;
							set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

						}
					}
				}
				if (theta <= 360 && theta >= 270)
				{
					pw_l_o = 1500 + dpw;
					pw_r_o = 1500 + dpw;
					set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
					if (theta > 358)
					{
						pw_l_o = 1500;
						pw_r_o = 1500;
						set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
						pw_l_o = 1500 - dpw;
						pw_r_o = 1500 + dpw;
						set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
						if (110 <= a2x && a2x <= 130)
						{
							pw_l_o = 1500;
							pw_r_o = 1500;
							set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

						}
					}
				}
			}
		}
		return 0;
	}
}


void right_limit(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y, double& D, double& b1x, double& b1y, double& b2x, double& b2y, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise)
{
	double a1, current_angle_r;
	int dpw = 250;
	pw_l = 1500;
	pw_r = 1500;

	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);
	pw_l = 1500 + dpw;
	pw_r = 1500 + dpw;
	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);
	current_angle_r = (atan2(a1y - a2y, a1x - a2x)) * 180 / PI;
	if (current_angle_r < 0)
	{
		a1 = 360 - abs(current_angle_r);
	}
	else {
		a1 = current_angle_r;
	}
	if (a1 >= 358 && a1 <= 360) a1 = 0;
	
	if (a1 > 272)
	{
		pw_l = 1500;
		pw_r = 1500;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		pw_l = 1500 - dpw;
		pw_r = 1500 + dpw;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
	}
	if (KEY('D')) {
		pw_l = 1500;
		pw_r = 1500;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		cout << b2x << "\t" << b2y << "\n";
	}
}

void up_limit(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y, double& D, double& b1x, double& b1y, double& b2x, double& b2y, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise)
{
	double a1, current_angle_r;
	int dpw = 250;
	pw_l = 1500;
	pw_r = 1500;
	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	pw_l = 1500 + dpw;
	pw_r = 1500 + dpw;
	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);
	current_angle_r = (atan2(a1y - a2y, a1x - a2x)) * 180 / PI;
	if (current_angle_r < 0)
	{
		a1 = 360 - abs(current_angle_r);
	}
	else {
		a1 = current_angle_r;
	}
	if (a1 >= 358 && a1 <= 360) a1 = 0;
	if (a1 == 0)
	{
		pw_l = 1500;
		pw_r = 1500;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
	}

	if (a1 >= 0 && a1 <= 10)
	{
		pw_l = 1500 + dpw;
		pw_r = 1500 + dpw;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		if (a1 > 5)
		{
			pw_l = 1500;
			pw_r = 1500;
			set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
				max_speed, opponent_max_speed);
			pw_l = 1500 - dpw;
			pw_r = 1500 + dpw;
			set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
				max_speed, opponent_max_speed);
		}

	}

	if (KEY('W')) {
		pw_l = 1500;
		pw_r = 1500;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		cout << b2x << "\t" << b2y << "\t" << a1 << "\n";
	}
}

void left_limit(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y, double& D, double& b1x, double& b1y, double& b2x, double& b2y, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise)
{
	double a1, current_angle_r;
	int dpw = 250;
	pw_l = 1500;
	pw_r = 1500;

	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);
	pw_l = 1500 + dpw;
	pw_r = 1500 + dpw;
	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	current_angle_r = (atan2(a1y - a2y, a1x - a2x)) * 180 / PI;
	if (current_angle_r < 0)
	{
		a1 = 360 - abs(current_angle_r);
	}
	else {
		a1 = current_angle_r;
	}
	if (a1 >= 358 && a1 <= 360) a1 = 0;

	if (a1 > 92)
	{
		pw_l = 1500;
		pw_r = 1500;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		pw_l = 1500 - dpw;
		pw_r = 1500 + dpw;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
	}
	if (KEY('A')) {
		pw_l = 1500;
		pw_r = 1500;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		cout << b2x << "\t" << b2y << "\t" << a1 << "\n";
	}
}

void bottom_limit(double& difx, double& dify, double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y, double& D, double& b1x, double& b1y, double& b2x, double& b2y, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise)
{
	double a1, current_angle_r;
	int dpw = 250;
	pw_l = 1500;
	pw_r = 1500;

	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	pw_l = 1500 + dpw;
	pw_r = 1500 + dpw;
	set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	current_angle_r = (atan2(a1y - a2y, a1x - a2x)) * 180 / PI;
	if (current_angle_r < 0)
	{
		a1 = 360 - abs(current_angle_r);
	}
	else {
		a1 = current_angle_r;
	}

	if (a1 > 182)
	{
		pw_l = 1500;
		pw_r = 1500;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		pw_l = 1500 - dpw;
		pw_r = 1500 + dpw;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
	}
	if (KEY('S')) {
		pw_l = 1500;
		pw_r = 1500;
		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		cout << b2x << "\t" << b2y << "\t" << a1 << "\n";
	}
}
void avoid_obstacle(double& d_robs, double& dox, double& doy, double& difx, double& dify,
	double& difbx, double& difby, double& a1x, double& a1y, double& a2x, double& a2y, double& b1x,
	double& b1y, double& b2x, double& b2y, int& pw_l_o, int& pw_r_o, double x_obs[], double y_obs[],
	int& pw_laser_o, int& laser_o, double& max_speed, double& opponent_max_speed, double& light,
	double& light_gradient, double& light_dir, double& image_noise)
{
	double a1, current_angle_r_screen, prev_angle_r, current_angle_r, diff_current_n_prev_angle_r;
	int dpw = 250;
	pw_l_o = 1500;
	pw_r_o = 1500;

	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

	current_angle_r = (atan2(a1y - a2y, a1x - a2x)) * 180 / PI;
	if (current_angle_r < 0)
	{
		a1 = 360 - abs(current_angle_r);
	}
	else {
		a1 = current_angle_r;
	}
	//Used for testing purposes
	if (KEY('C')) {
		pw_l_o = 1500;
		pw_r_o = 1500;
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
		cout << a2x << "\t" << a2y << "\t" << a1 << "\t" << d_robs << "\n";
	}

	if (a1 >= 0 && a1 <= 80)
	{
		pw_l_o = 1500 - dpw;
		pw_r_o = 1500 - dpw;
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
		if (a1 < 10)
		{
			pw_l_o = 1500;
			pw_r_o = 1500;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

			pw_l_o = 1500 - dpw;
			pw_r_o = 1500 + dpw;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
		}
		//Used for testing purposes
		if (KEY('T')) {
			pw_l_o = 1500;
			pw_r_o = 1500;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
			cout << a2x << "\t" << a2y << "\t" << a1 << "\t" << d_robs << "\n";
		}
	}
	if (a1 >= 80 && a1 <= 170)
	{
		pw_l_o = 1500 - dpw;
		pw_r_o = 1500 - dpw;
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
		if (a1 < 82)
		{
			pw_l_o = 1500;
			pw_r_o = 1500;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

			pw_l_o = 1500 - dpw;
			pw_r_o = 1500 + dpw;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
		}
		//Used for testing purposes
		if (KEY('F')) {
			pw_l_o = 1500;
			pw_r_o = 1500;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
			cout << a2x << "\t" << a2y << "\t" << a1 << "\t" << d_robs << "\n";
		}
	}
	if (a1 >= 170 && a1 <= 270)
	{
		pw_l_o = 1500 - dpw;
		pw_r_o = 1500 - dpw;
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
		if (a1 < 172)
		{
			pw_l_o = 1500;
			pw_r_o = 1500;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

			pw_l_o = 1500 - dpw;
			pw_r_o = 1500 + dpw;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
		}
		if (KEY('G')) {
			pw_l_o = 1500;
			pw_r_o = 1500;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
			cout << a2x << "\t" << a2y << "\t" << a1 << "\t" << d_robs << "\n";
		}
	}
	if (a1 >= 270 && a1 <= 358)
	{
		pw_l_o = 1500 - dpw;
		pw_r_o = 1500 - dpw;
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
		if (a1 < 280)
		{
			pw_l_o = 1500;
			pw_r_o = 1500;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

			pw_l_o = 1500 - dpw;
			pw_r_o = 1500 + dpw;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
		}
		if (KEY('H')) {
			pw_l_o = 1500;
			pw_r_o = 1500;
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);
			cout << a2x << "\t" << a2y << "\t" << a1 << "\t" << d_robs << "\n";
		}
	}

}

int control_manual_opponent(int& pw_l_o, int& pw_r_o, double& dpw, int& pw_laser_o, int& laser_o,
	double& opponent_max_speed) {

	// stop for default case
	pw_l_o = 1500;
	pw_r_o = 1500;

	// read the keyboard and set the opponent inputs
	if (KEY('I')) {
		pw_l_o = 1500 - dpw;
		pw_r_o = 1500 + dpw;
	}

	if (KEY('K')) {
		pw_l_o = 1500 + dpw;
		pw_r_o = 1500 - dpw;
	}

	if (KEY('J')) {
		pw_l_o = 1500 + dpw;
		pw_r_o = 1500 + dpw;
	}

	if (KEY('L')) {
		pw_l_o = 1500 - dpw;
		pw_r_o = 1500 - dpw;
	}

	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
		opponent_max_speed);

	return 0;
}


int track_and_control(i2byte label_num[], image& label, double& a1x, double& a1y, double& a2x,
	double& a2y, double& b1x, double& b2x, double& b1y, double& b2y, double& D, int& pw_l,
	int& pw_r, double x_obs[], double y_obs[], int& pw_laser, int& laser, double& max_speed,
	double& opponent_max_speed, double& light, double& light_gradient, double& light_dir,
	double& image_noise, image& a, image& rgb0, image& rgb) {

	search_object(label_num[1], label, a1x, a1y);
	search_object(label_num[2], label, a2x, a2y);
	search_object(label_num[3], label, b1x, b1y);
	search_object(label_num[4], label, b2x, b2y);

	centroid(a, label, label_num[1], a1x, a1y);
	centroid(a, label, label_num[2], a2x, a2y);
	centroid(a, label, label_num[3], b1x, b1y);
	centroid(a, label, label_num[4], b2x, b2y);

	controller(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y, D, pw_l, pw_r, x_obs, y_obs,
		pw_laser, laser, max_speed, opponent_max_speed, light, light_gradient, light_dir,
		image_noise);

	copy(rgb0, rgb);
	draw_point_rgb(rgb, (int)a1x, (int)a1y, 120, 140, 150);
	draw_point_rgb(rgb, (int)a2x, (int)a2y, 120, 140, 150);
	draw_point_rgb(rgb, (int)b1x, (int)b1y, 120, 140, 150);
	draw_point_rgb(rgb, (int)b2x, (int)b2y, 120, 140, 150);

	return 0;
}

void set_simulation_model(int& N_obs, double x_obs[], double y_obs[], double size_obs[], double& D,
	double& Lx, double& Ly, double& Ax, double& Ay, double& alpha_max, int& n_robot) {

	// number of obstacles
	N_obs = 1;

	x_obs[1] = 320; // pixels 200
	y_obs[1] = 240; // pixels 300
	size_obs[1] = 1; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////
	D = 121.0; // distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)

	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;
}

int set_initial_inputs(double& x0, double& y0, double& theta0, int& pw_l, int& pw_r, int& pw_laser,
	int& laser, double& max_speed, double& opponent_max_speed, double& light, double& light_gradient,
	double& light_dir, double& image_noise, int& pw_l_o, int& pw_r_o, int& pw_laser_o, int& laser_o)

{

	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	int mode = 0;
	int level = 1;
	set_simulation_mode(mode, level);

	//Setting Robot A's initial position
	x0 = 250;
	y0 = 300;
	theta0 = PI/4;
	set_robot_position(x0, y0, theta0);

	//Setting opponent's initial position

	x0 = 500;
	y0 = 380;
	theta0 = 0;
	set_opponent_position(x0, y0, theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l_o = 1500; // pulse width for left wheel servo (us)
	pw_r_o = 1500; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// lighting parameters 
	light = 1.5;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 0.5;

	// set initial inputs
	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	pw_l_o = 1500; // pulse width for left wheel servo (us)
	pw_r_o = 1500; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
		opponent_max_speed);

	return 0;

}
