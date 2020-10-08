#include "kilolib.h"
#include <stdlib.h>
#include <math.h>
//#define DEBUG
//#include <debug.h>

// --------------------------------------------------------------------------------------------------------------
// -------------------------------------------- Constant Declaration --------------------------------------------
// --------------------------------------------------------------------------------------------------------------

// The CODE is scalable trough 3 variables: number of kilobots, number of grid rows, number of grid coloumns. 
#define NUM_Kilobots 25						// Number of kilobots 
#define PI 3.1415926535						// Pi
#define Num_Matrix_Rows 5					// Number of grid rows
#define Num_Matrix_Coloumns 5			// Number of grid coloumns  

#define STOP 0								// variables used for the motion (see function: void set_motion())
#define FORWARD 1
#define LEFT 2 
#define RIGHT 3



#define MINIMUM_DEGREES 85
#define MAXIMUM_DEGREES 95
#define DIFFERENCE_THRESHOLD  10
#define STRAIGHT_THRESHOLD 70
#define DIAGONAL_THRESHOLD_LOWER 100
#define DIAGONAL_THRESHOLD_UPPER  110

#define MOTION_DELAY 500


// --------------------------------------------------------------------------------------------------------------
// -------------------------------------------- Variable Declaration --------------------------------------------
// --------------------------------------------------------------------------------------------------------------

// ---------- Message Variables ------------
message_t message; 
int new_message = 0;

int composed_message = 0;					// the firs byte is composed of two part, one is the kilobot ID and the other is the status is which the Kilobot is operating
int composed_message_rx = 0;

int message_ID = 0;							
int message_distance = 0;

int ID_N_dist = 0;							// ID of the Kilobot positioned above, on the NORD direction; variable used for the receiving messages
int ID_NE_dist = 0;
int ID_E_dist = 0;
int ID_SE_dist = 0;
int ID_S_dist = 0;
int ID_SW_dist = 0;
int ID_W_dist = 0;
int ID_NW_dist = 0;



// ---------- State Variables ------------
unsigned int State = 0;
int operating_mode_tx = 0;					// variable transmitted and received in the composed message together with the Kilobot ID
int operating_mode_rx = 0;


// ---------- Neighbour Variables ------------
int kil_dist_matrix[9][9];			//Matrix to store the distance information of all neighbours
int neighbours_IDs_array[9] = {0};		//Matrix to store IDs of neighbours
int temp_dist = 0;

int kil_dist_N = 0;							// variables used in the motion algorithm
int kil_dist_NE = 0;
int kil_dist_E = 0;
int kil_dist_SE = 0;
int kil_dist_S = 0;
int kil_dist_SW = 0;
int kil_dist_W = 0;
int kil_dist_NW = 0;


// ---------- Angle Variables ------------
int alpha_deg[3];					// starting from the top RIGHT position the 90-degrees angles are ordered in clockwise
int alpha_deg_two[3];
float temp_angle_float = 0;
float dist_float[3];
float dist_float_two[3];			// in order to compute angles relative to differents triangular shapes	

// ---------- Loop Variables ------------
int i = 0;
int j = 0;
int k = 0;
int l = 0;

// ---------- Move Variables ------------
unsigned int current_motion = 0;

// ---------- Time Variables ------------
unsigned long int last_timereset = 0;


// --------------------------------------------------------------------------------------------------------------
// ----------------------------------------------- Motor Function -----------------------------------------------
// --------------------------------------------------------------------------------------------------------------

void set_motion(unsigned int new_motion){

	if(current_motion != new_motion)
	{
	 	current_motion = new_motion;
	 	if (current_motion == STOP)
	 	{
	 		//set_color(RGB(3, 0, 0)); //red
	   		set_motors(0,0);
	 	}
	 	else if(current_motion == FORWARD)
	 	{
	 		//set_color(RGB(0, 3, 0)); //green
	   		spinup_motors();
	   		set_motors(kilo_straight_left, kilo_straight_right);
	 	}
	 	else if (current_motion == LEFT)
	 	{
	 		//set_color(RGB(0, 0, 3)); //blue
			spinup_motors();
		  	set_motors(kilo_turn_left,0);
		 
	 	}
	 	else if (current_motion == RIGHT)
	 	{
	 		//set_color(RGB(3, 0, 3)); //pink
	   		spinup_motors();
		  	set_motors(0,kilo_turn_right);
 		}
	}
}

// --------------------------------------------------------------------------------------------------------------
// --------------------------------------- Message Construction Function ----------------------------------------
// --------------------------------------------------------------------------------------------------------------

void msg_codedist_tx(uint8_t operating_mode, uint8_t N_dist, uint8_t NE_dist, uint8_t E_dist, uint8_t SE_dist, uint8_t S_dist, uint8_t SW_dist, uint8_t W_dist, uint8_t NW_dist){

	composed_message = operating_mode << 7 | kilo_uid;

	message.type = NORMAL;	
	message.data[0] = composed_message;														
	message.data[1] = N_dist;							
	message.data[2] = NE_dist;
	message.data[3] = E_dist;
	message.data[4] = SE_dist;
	message.data[5] = S_dist;
	message.data[6] = SW_dist;
	message.data[7] = W_dist;
	message.data[8] = NW_dist;				
	message.crc = message_crc(&message);

	composed_message = operating_mode << 7 | kilo_uid;

	//printf("ID = %d and N_dist is %d and NE_dist is %d and E_dist is %d and SE_dist is %d and S_dist is %d and SW_dist is %d and W_dist is %d and NW_dist is %d \n \n", kilo_uid, N_dist, NE_dist, E_dist,SE_dist,S_dist,SW_dist,W_dist,NW_dist);
	
}


message_t *message_tx(){

	return &message;
}

void tx_message_success(){

	msg_codedist_tx(operating_mode_tx,kil_dist_matrix[0][1],kil_dist_matrix[0][2],kil_dist_matrix[0][3],kil_dist_matrix[0][4],kil_dist_matrix[0][5],kil_dist_matrix[0][6],kil_dist_matrix[0][7],kil_dist_matrix[0][8]);	
}


// --------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- Initialization ------------------------------------------------
// --------------------------------------------------------------------------------------------------------------

void setup(){ 

	// Setting initialised variables to 0 to avoid NaN errors in case of communication delays
	for (i = 0; i < 9; ++i){								
		for (j = 0; j < 9; ++j)
		{
			kil_dist_matrix[i][j] = 0;				
		}
	}

	for (i = 0; i < 3; ++i)
	{
		alpha_deg[i] = 0;
		dist_float[i] = 0;
		dist_float_two[i] = 0;
	}

	// saving the neighbours ID in the array following the proper order, in the position 0 own ID is always saved


	/*	DIAGRAM
		/----\        /----\        /----\				/----\        /----\        /----\					
		|  A |========|  B |========|  C |				|  8 |========|  1 |========|  2 |
		\----/        \----/        \----/				\----/        \----/        \----/
		  ||			||			  ||	 		 	  ||			||			  ||
		  ||			||			  ||	 		 	  ||			||			  ||
		  ||			||			  ||	  			  ||			||			  ||
		/----\        /----\        /----\				/----\        /----\        /----\
		|  D |========|  E |========|  F |				|  7 |========|  0 |========|  3 |
		\----/        \----/        \----/				\----/        \----/        \----/
		  ||			||			  ||	 		 	  ||			||			  ||
		  ||			||			  ||	 		 	  ||			||			  ||
		  ||			||			  ||	  			  ||			||			  ||
		/----\        /----\        /----\				/----\        /----\        /----\
		|  G |========|  H |========|  I |				|  6 |========|  5 |========|  4 |
		\----/        \----/        \----/				\----/        \----/        \----/
	*/

	if(kilo_uid <= Num_Matrix_Coloumns)				//If KB is on the first row (1 to n) (A to C)
	{					
		if(kilo_uid == 1)							//If first in row (A on diagram) (storing A (itself) and B,E,D)
		{
			neighbours_IDs_array[0] = kilo_uid;
			neighbours_IDs_array[3] = kilo_uid + 1;
			neighbours_IDs_array[4] = kilo_uid + Num_Matrix_Coloumns + 1;
			neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
		}
		else if(kilo_uid == Num_Matrix_Coloumns)	//If last in row (C on diagram)
		{
			neighbours_IDs_array[0] = kilo_uid;
			neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
			neighbours_IDs_array[6] = kilo_uid + Num_Matrix_Coloumns - 1;
			neighbours_IDs_array[7] = kilo_uid - 1;
		}
		else										//If any other (B on diagram)
		{
			neighbours_IDs_array[0] = kilo_uid;
			neighbours_IDs_array[3] = kilo_uid + 1;
			neighbours_IDs_array[4] = kilo_uid + Num_Matrix_Coloumns + 1;
			neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
			neighbours_IDs_array[6] = kilo_uid + Num_Matrix_Coloumns - 1;
			neighbours_IDs_array[7] = kilo_uid - 1;
		}
	}
	else if(kilo_uid > Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns) //If KB is on the last row ((n^2)-n  to n^2) (G to I)
	{
		if((kilo_uid - 1) % Num_Matrix_Coloumns == 0)					//If first in row (G on diagram) (storing G (itself) and D,E,H)
		{
			neighbours_IDs_array[0] = kilo_uid;
			neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
			neighbours_IDs_array[2] = kilo_uid - Num_Matrix_Coloumns + 1;
			neighbours_IDs_array[3] = kilo_uid + 1;
		}
		else if(kilo_uid == Num_Matrix_Rows * Num_Matrix_Coloumns)		//If last in row (I on diagram)
		{
			neighbours_IDs_array[0] = kilo_uid;
			neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
			neighbours_IDs_array[7] = kilo_uid - 1;
			neighbours_IDs_array[8] = kilo_uid - Num_Matrix_Coloumns - 1;				
		}
		else 															//If any other (H on diagram)
		{
			neighbours_IDs_array[0] = kilo_uid;
			neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
			neighbours_IDs_array[2] = kilo_uid - Num_Matrix_Coloumns + 1;
			neighbours_IDs_array[3] = kilo_uid + 1;
			neighbours_IDs_array[7] = kilo_uid - 1;
			neighbours_IDs_array[8] = kilo_uid - Num_Matrix_Coloumns - 1;
		}
	}
	else if((kilo_uid - 1) % Num_Matrix_Coloumns == 0)	//If KB is on the left edge (D)
	{
		neighbours_IDs_array[0] = kilo_uid;
		neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
		neighbours_IDs_array[2] = kilo_uid - Num_Matrix_Coloumns + 1;
		neighbours_IDs_array[3] = kilo_uid + 1;
		neighbours_IDs_array[4] = kilo_uid + Num_Matrix_Coloumns + 1;
		neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;

	}
	else if(kilo_uid % Num_Matrix_Coloumns == 0)		//If KB is on the right edge (F)
	{

		neighbours_IDs_array[0] = kilo_uid;
		neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
		neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
		neighbours_IDs_array[6] = kilo_uid + Num_Matrix_Coloumns - 1;
		neighbours_IDs_array[7] = kilo_uid - 1;
		neighbours_IDs_array[8] = kilo_uid - Num_Matrix_Coloumns - 1;
	} 
	else												//Otherwise its in the centre (E)
	{
		neighbours_IDs_array[0] = kilo_uid;
		neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
		neighbours_IDs_array[2] = kilo_uid - Num_Matrix_Coloumns + 1;
		neighbours_IDs_array[3] = kilo_uid + 1;
		neighbours_IDs_array[4] = kilo_uid + Num_Matrix_Coloumns + 1;
		neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
		neighbours_IDs_array[6] = kilo_uid + Num_Matrix_Coloumns - 1;
		neighbours_IDs_array[7] = kilo_uid - 1;
		neighbours_IDs_array[8] = kilo_uid - Num_Matrix_Coloumns - 1;
	}

	//Logic check to ensure that any invalid Neighbour IDs are set to 0
	for (i = 0; i < 9; ++i){
		
		if(neighbours_IDs_array[i] < 0 || neighbours_IDs_array[i] > NUM_Kilobots){

			neighbours_IDs_array[i] = 0;
		}
	}	
	
	if (kilo_uid == 1){										// LEDs turned on for 3 seconds in order to check the proper uploading of the code on the Kilobot,
																	// each robot LED colour depends on the ID, so on the position, of the robot itself in the grid
		
		set_color(RGB(1,0,0));		// red LED

	} else if (kilo_uid < Num_Matrix_Coloumns && kilo_uid > 1){
		
		set_color(RGB(0,1,0));		// green LED

	} else if (kilo_uid == Num_Matrix_Coloumns){
		
		set_color(RGB(0,0,1));		// blue LED
	
	} else if ((kilo_uid - 1) % Num_Matrix_Coloumns == 0 && kilo_uid != 1 && kilo_uid < (Num_Matrix_Rows * Num_Matrix_Coloumns) - Num_Matrix_Coloumns){

		set_color(RGB(1,1,0));		// orange LED

	} else if (kilo_uid % Num_Matrix_Coloumns == 0 && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows && kilo_uid != Num_Matrix_Coloumns){

		set_color(RGB(1,0,1));		// violet LED

	} else if (kilo_uid == Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1){

		set_color(RGB(0,1,1));		// magenta LED 

	} else if (kilo_uid > (Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1) && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows){

		set_color(RGB(1,1,1));		// white LED

	} else if (kilo_uid == Num_Matrix_Coloumns * Num_Matrix_Rows){
		
		set_color(RGB(1,0,0));		// red LED	

	} else {

		set_color(RGB(0,0,0));		// LED turned off
	}

	delay(3000);

	// start the message transmission
	message.type = NORMAL;																	
	message.data[0] = kilo_uid;
	message.crc = message_crc(&message);

	// variable for the timer
	last_timereset = kilo_ticks;

	//Setting state to 0 (only state in current implementation)
	State = 0;						

}


// --------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Main Loop --------------------------------------------------
// --------------------------------------------------------------------------------------------------------------

void loop(){

	if (State == 0){						/******************************* State 0 ********************************/	
											// in this code there is only one state, but other states can be easily added	

		// ----------------------------------------------------------------------------------------------------------
		// Computing the average distance between node i and j (from stored values of dist(i,j) and dist(j,i)
		for (i = 0; i < 9; ++i)
		{
		 	for (j = i; j < 9; ++j)
		 	{
		 		if(kil_dist_matrix[i][j] != 0 && kil_dist_matrix[j][i] == 0)
		 		{						
		 			kil_dist_matrix[j][i] = kil_dist_matrix[i][j];
	 			}
	 			else if(kil_dist_matrix[j][i] != 0 && kil_dist_matrix[i][j] == 0)
	 			{
	 				kil_dist_matrix[i][j] = kil_dist_matrix[j][i];
	 			} 
	 			else if(kil_dist_matrix[i][j] != 0 && kil_dist_matrix[j][i] != 0)	// compute the average value
	 			{				
					temp_dist = ((float)kil_dist_matrix[i][j] + (float)kil_dist_matrix[j][i]) / 2.0 + 0.5;		// approximation by excess		
					kil_dist_matrix[i][j] = (int)temp_dist;
					kil_dist_matrix[j][i] = (int)temp_dist;
				}
		 	}
		}

		// ----------------------------------------------------------------------------------------------------------
		// Comptuing the angles based on the distance information - where two angles can be compted they are (in centre)
		// Depending on the position of the Kilobot in the grid, different distances are needed to compute the angles

		if (kilo_uid == 1)												// kiloID 1 (in a 3x3 grid)
		{																
			dist_float[0] = (float)kil_dist_matrix[5][3];							
			dist_float[1] = (float)kil_dist_matrix[0][5];
			dist_float[2] = (float)kil_dist_matrix[0][3];
		}
		else if (kilo_uid < Num_Matrix_Coloumns && kilo_uid > 1)		// kiloID 2 (in a 3x3 grid)
		{ 			
			dist_float[0] = (float)kil_dist_matrix[5][3];				
			dist_float[1] = (float)kil_dist_matrix[0][5];
			dist_float[2] = (float)kil_dist_matrix[0][3];

			dist_float_two[0] = (float)kil_dist_matrix[7][5];
			dist_float_two[1] = (float)kil_dist_matrix[0][7];
			dist_float_two[2] = (float)kil_dist_matrix[0][5];	
		}
		else if(kilo_uid == Num_Matrix_Coloumns * Num_Matrix_Rows)		// kiloID 9 (in a 3x3 grid)
		{			
			dist_float[0] = (float)kil_dist_matrix[7][1];
			dist_float[1] = (float)kil_dist_matrix[0][7];
			dist_float[2] = (float)kil_dist_matrix[0][1]; 
		}
		else if(kilo_uid == Num_Matrix_Coloumns)						// kiloID 3 (in a 3x3 grid)
		{									
			dist_float[0] = (float)kil_dist_matrix[7][5];
			dist_float[1] = (float)kil_dist_matrix[0][7];
			dist_float[2] = (float)kil_dist_matrix[0][5];	
		}
		else if (kilo_uid % Num_Matrix_Coloumns == 0 && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows && kilo_uid != Num_Matrix_Coloumns)
		{																// kiloID 6 (in a 3x3 grid)
			dist_float[0] = (float)kil_dist_matrix[7][5];
			dist_float[1] = (float)kil_dist_matrix[0][7];
			dist_float[2] = (float)kil_dist_matrix[0][5];

			dist_float_two[0] = (float)kil_dist_matrix[7][1];
			dist_float_two[1] = (float)kil_dist_matrix[0][7];
			dist_float_two[2] = (float)kil_dist_matrix[0][1];
		}
		else if (kilo_uid > (Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1) && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows)
		{																// kiloID 8 (in a 3x3 grid)
			dist_float[0] = (float)kil_dist_matrix[3][1];
			dist_float[1] = (float)kil_dist_matrix[0][3];
			dist_float[2] = (float)kil_dist_matrix[0][1];

			dist_float_two[0] = (float)kil_dist_matrix[7][1];
			dist_float_two[1] = (float)kil_dist_matrix[0][7];
			dist_float_two[2] = (float)kil_dist_matrix[0][1];
		}
		else if (kilo_uid == Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1)
		{																// kiloID 7 (in a 3x3 grid)
			dist_float[0] = (float)kil_dist_matrix[3][1];
			dist_float[1] = (float)kil_dist_matrix[0][3];
			dist_float[2] = (float)kil_dist_matrix[0][1];	
		}
		else if ((kilo_uid - 1) % Num_Matrix_Coloumns == 0 && kilo_uid != 1 && kilo_uid < (Num_Matrix_Rows * Num_Matrix_Coloumns) - Num_Matrix_Coloumns)
		{																// kiloID 4 (in a 3x3 grid)
			dist_float[0] = (float)kil_dist_matrix[3][1];
			dist_float[1] = (float)kil_dist_matrix[0][3];
			dist_float[2] = (float)kil_dist_matrix[0][1];

			dist_float_two[0] = (float)kil_dist_matrix[5][3];
			dist_float_two[1] = (float)kil_dist_matrix[0][5];
			dist_float_two[2] = (float)kil_dist_matrix[0][3];
		}
		else 
		{																// kiloID 5 (in a 3x3 grid)
			dist_float[0] = (float)kil_dist_matrix[5][3];
			dist_float[1] = (float)kil_dist_matrix[0][5];
			dist_float[2] = (float)kil_dist_matrix[0][3]; 

			dist_float_two[0] = (float)kil_dist_matrix[7][1];
			dist_float_two[1] = (float)kil_dist_matrix[0][7];
			dist_float_two[2] = (float)kil_dist_matrix[0][1];			
		}


		// ----------------------------------
		kil_dist_N = kil_dist_matrix[0][1];								// using an easy name for the distances
		kil_dist_NE = kil_dist_matrix[0][2];
		kil_dist_E = kil_dist_matrix[0][3];
		kil_dist_SE = kil_dist_matrix[0][4];
		kil_dist_S = kil_dist_matrix[0][5];
		kil_dist_SW = kil_dist_matrix[0][6];
		kil_dist_W = kil_dist_matrix[0][7];
		kil_dist_NW = kil_dist_matrix[0][8];


		// ----------------------------------------------------------------------------------------------------------
		// Using the cosine rule to calculate the angle between two of the linkages attached to the KB

		if(dist_float[0] != 0 && dist_float[1] != 0 && dist_float[2] != 0)
		{
			temp_angle_float = acos((dist_float[2]*dist_float[2] + dist_float[1]*dist_float[1] - dist_float[0]*dist_float[0]) / (2.0 * dist_float[1] * dist_float[2]));
			alpha_deg[0] = (int)(temp_angle_float / PI * 180.0 + 0.5);  //comverting from radians to degrees			

			// If needed, the computation of the other angles of the triangle considered can be easy
			// in the motion algorithm only one angle is used
			// temp_angle_float = acos((dist_float[0]*dist_float[0] + dist_float[2]*dist_float[2] - dist_float[1]*dist_float[1]) / (2.0 * dist_float[0] * dist_float[2]));					 
			// alpha_deg[1] = (int)(temp_angle_float / PI * 180.0 + 0.5);																																					
			// alpha_deg[2] = 180 - alpha_deg[1] - alpha_deg[0];
		} 
		if(dist_float_two[0] != 0 && dist_float_two[1] != 0 && dist_float_two[2] != 0)
		{
			temp_angle_float = acos((dist_float_two[2]*dist_float_two[2] + dist_float_two[1]*dist_float_two[1] - dist_float_two[0]*dist_float_two[0]) / (2.0 * dist_float_two[1] * dist_float_two[2]));
			alpha_deg_two[0] = (int)(temp_angle_float / PI * 180.0 + 0.5);			
		} 



		// ----------------------------------------------------------------------------------------------------------
		// Movement and shape control


		if (kilo_uid == 1)
		{
			if (alpha_deg[0] >= MINIMUM_DEGREES && alpha_deg[0] <= MAXIMUM_DEGREES)  //if within limits, minor adjustments
			{	// if the angles is computed, the distances (used in that computation) should be known and different from zero				
				if ((kil_dist_E < STRAIGHT_THRESHOLD && kil_dist_E > 0) || (kil_dist_S < STRAIGHT_THRESHOLD && kil_dist_S > 0)) 	
				{											//If either south or east distances are less than threshold						
					if (kil_dist_E - kil_dist_S > DIFFERENCE_THRESHOLD)
					{										// the direction of the 1st Kilobot is influenced by the two robots behind it, this sometimes helps in keeping a straight direction
						set_motion(RIGHT);
						delay(MOTION_DELAY);
					}
					else if (kil_dist_S - kil_dist_E > DIFFERENCE_THRESHOLD)
					{
						set_motion(LEFT);
						delay(MOTION_DELAY);
					}

					set_motion(FORWARD);
					delay(MOTION_DELAY);
				} 
				else 
				{
					set_motion(STOP);
					delay(MOTION_DELAY);
				}

			}
			else if (alpha_deg[0] < MINIMUM_DEGREES && alpha_deg[0] > 0)		//If below limits
			{
				if (kil_dist_E > STRAIGHT_THRESHOLD || kil_dist_S > STRAIGHT_THRESHOLD) 	//If distance greater than threshold, stop
				{
					set_motion(STOP);
					delay(MOTION_DELAY);
				
				}
				else 						//otherwise keep moving forwards. 
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			}
			else if (alpha_deg[0] > MAXIMUM_DEGREES)				//Otherwise if above limits
			{
				if (kil_dist_E != 0 && kil_dist_S != 0)				//and if distances are not 0
				{
					if (kil_dist_E - kil_dist_S > DIFFERENCE_THRESHOLD)
					{
						set_motion(RIGHT);
						delay(MOTION_DELAY);
					}
					else if (kil_dist_S - kil_dist_E > DIFFERENCE_THRESHOLD)
					{
						set_motion(LEFT);
						delay(MOTION_DELAY);
					}
				}
					
				set_motion(FORWARD);	
				delay(MOTION_DELAY);			
			} 
			else 
			{		// when the angle is not computed the module ID 1 keeps a forward motion, because it has the role to "pull" the system
				set_motion(FORWARD);
			}
		}
		else if (kilo_uid < Num_Matrix_Coloumns && kilo_uid > 1)			// kilobot uid 2
		{			
			if (alpha_deg_two[0] >= MINIMUM_DEGREES && alpha_deg_two[0] <= MAXIMUM_DEGREES)
			{
				if (kil_dist_SW >= DIAGONAL_THRESHOLD_LOWER && kil_dist_SW <= DIAGONAL_THRESHOLD_UPPER)
				{												// keep the diagonal distance
					set_motion(FORWARD);	
					delay(MOTION_DELAY);					
				} 
				else if (kil_dist_SW <= DIAGONAL_THRESHOLD_LOWER && kil_dist_SW > 0)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);

					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
				else if (kil_dist_SW >= DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
					
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}

			}
			else if (alpha_deg_two[0] < MINIMUM_DEGREES && alpha_deg_two[0] > 0)
			{
				set_motion(LEFT);
				delay(MOTION_DELAY);

				set_motion(FORWARD);
				delay(MOTION_DELAY);
			}
			else if (alpha_deg_two[0] > MAXIMUM_DEGREES){

				set_motion(RIGHT);
				delay(MOTION_DELAY);
				set_motion(FORWARD);
				delay(MOTION_DELAY);
			} 
			else 	// if the angle is not computed, the diagonal distance is checked because could be too small
			{
				if (kil_dist_SW < DIAGONAL_THRESHOLD_LOWER && kil_dist_SW > 0) 
				{								
					set_motion(RIGHT);
					delay(MOTION_DELAY);
				}

				set_motion(FORWARD);
				delay(MOTION_DELAY);
			}	

		}
		else if (kilo_uid == Num_Matrix_Coloumns)
		{															// kilobot uid 3 (in a 3x3 grid)	
			if (alpha_deg[0] <= MAXIMUM_DEGREES && alpha_deg[0] >= MINIMUM_DEGREES)
			{	
				if (kil_dist_SW >= DIAGONAL_THRESHOLD_LOWER && kil_dist_SW <= DIAGONAL_THRESHOLD_UPPER)
				{	
					set_motion(FORWARD);	
					delay(MOTION_DELAY);					
				}
				else if (kil_dist_SW < DIAGONAL_THRESHOLD_LOWER && kil_dist_SW > 0)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);					

					if (kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0)
					{												// not to hit the front Kilobot
						set_motion(STOP);
						delay(MOTION_DELAY);
					} 
					else 
					{
						set_motion(FORWARD);
						delay(MOTION_DELAY);
					}
				} 
				else if (kil_dist_SW > DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
				
					if (kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0)
					{
						set_motion(STOP);	
						delay(MOTION_DELAY);
					} 
					else 
					{
						set_motion(FORWARD);
						delay(MOTION_DELAY);
					}
				}
			}
			else if (alpha_deg[0] >= MAXIMUM_DEGREES)
			{
				set_motion(RIGHT);	
				delay(MOTION_DELAY);

				if (kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0)
				{
					set_motion(STOP);
					delay(MOTION_DELAY);
				}
				else
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			}
			else if (alpha_deg[0] <= MINIMUM_DEGREES && alpha_deg[0] > 0)
			{
				set_motion(LEFT);	
				delay(MOTION_DELAY);

				if (kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0)
				{
					set_motion(STOP);
					delay(MOTION_DELAY);
				}
				else
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			} 
			else 
			{
				if (kil_dist_SW < DIAGONAL_THRESHOLD_LOWER && kil_dist_SW > 0) 
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);
				}

				set_motion(FORWARD);
				delay(MOTION_DELAY);
			}
		}
		else if ((kilo_uid - 1) % Num_Matrix_Coloumns == 0 && kilo_uid != 1 && kilo_uid < (Num_Matrix_Rows * Num_Matrix_Coloumns) - Num_Matrix_Coloumns)
		{																			// kilobot uid 4
			if (alpha_deg[0] >= MINIMUM_DEGREES && alpha_deg[0] <= MAXIMUM_DEGREES)
			{
				if (kil_dist_NE >= DIAGONAL_THRESHOLD_LOWER && kil_dist_NE <= DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(FORWARD);	
					delay(MOTION_DELAY);
				}
				else if (kil_dist_NE < DIAGONAL_THRESHOLD_LOWER && kil_dist_NE > 0)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
						
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
				else if (kil_dist_NE > DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);
					
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			}
			else if (alpha_deg[0] < MINIMUM_DEGREES && alpha_deg[0] > 0)
			{
				set_motion(RIGHT);
				delay(MOTION_DELAY);

				set_motion(FORWARD);
				delay(MOTION_DELAY);
			} 
			else if (alpha_deg[0] > MAXIMUM_DEGREES)
			{
				set_motion(LEFT);
				delay(MOTION_DELAY);
				set_motion(FORWARD);
				delay(MOTION_DELAY);

			}
			else
			{
				if (kil_dist_NE < DIAGONAL_THRESHOLD_LOWER && kil_dist_NE > 0)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
				}
				set_motion(FORWARD);
				delay(MOTION_DELAY);
			}			
		}
		else if (kilo_uid % Num_Matrix_Coloumns == 0 && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows && kilo_uid != Num_Matrix_Coloumns)
		{								// kilobot uid 6	
			if (alpha_deg[0] >= MINIMUM_DEGREES && alpha_deg[0] <= MAXIMUM_DEGREES)
			{
				if (kil_dist_SW >= DIAGONAL_THRESHOLD_LOWER && kil_dist_SW <= DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
				else if (kil_dist_SW < DIAGONAL_THRESHOLD_LOWER && kil_dist_SW > 0)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);

					if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0)) 
					{ 			// not to hit both front and side Kilobots
						set_motion(STOP);	
						delay(MOTION_DELAY);								
					}
					else
					{
						set_motion(FORWARD);
						delay(MOTION_DELAY);
					}
				}
				else if (kil_dist_SW > DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
					
					if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0)) 
					{
						set_motion(STOP);	
						delay(MOTION_DELAY);								
					}
					else
					{
						set_motion(FORWARD);
						delay(MOTION_DELAY);
					}
				}
			}
			else if (alpha_deg[0] < MINIMUM_DEGREES && alpha_deg[0] > 0)
			{
				set_motion(LEFT);
				delay(MOTION_DELAY);

				if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0)) 
				{
					set_motion(STOP);	
					delay(MOTION_DELAY);								
				} 
				else
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			}
			else if (alpha_deg[0] > MAXIMUM_DEGREES)
			{
				set_motion(RIGHT);
				delay(MOTION_DELAY);

				if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0))
				{
					set_motion(STOP);	
					delay(MOTION_DELAY);								
				}
				else
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			} 
			else
			{
				if (kil_dist_SW < DIAGONAL_THRESHOLD_LOWER && kil_dist_SW > 0)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);
				}

				set_motion(FORWARD);
				delay(MOTION_DELAY);
			}
		}
		else if (kilo_uid == Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1)
		{ 																								// kilobot uid 7
			if (alpha_deg[0] <= MAXIMUM_DEGREES && alpha_deg[0] >= MINIMUM_DEGREES)
			{
				if (kil_dist_NE >= DIAGONAL_THRESHOLD_LOWER && kil_dist_NE <= DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
				else if (kil_dist_NE < DIAGONAL_THRESHOLD_LOWER && kil_dist_NE > 0)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);

					if (kil_dist_N > STRAIGHT_THRESHOLD)
					{
						set_motion(FORWARD);	
						delay(MOTION_DELAY);
					} 
					else 
					{
						set_motion(STOP);
						delay(MOTION_DELAY);
					}
				}
				else if (kil_dist_NE > DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);
					
					if (kil_dist_N > STRAIGHT_THRESHOLD)
					{
						set_motion(FORWARD);	
						delay(MOTION_DELAY);
					}
					else
					{
						set_motion(STOP);
						delay(MOTION_DELAY);
					}
				}
			}
			else if (alpha_deg[0] < MINIMUM_DEGREES && alpha_deg[0] > 0)
			{
				set_motion(RIGHT);	
				delay(MOTION_DELAY);

				if (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0)
				{
					set_motion(STOP);
					delay(MOTION_DELAY);
				}
				else
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			} 
			else if (alpha_deg[0] > MAXIMUM_DEGREES)
			{		
				set_motion(LEFT);	
				delay(MOTION_DELAY);

				if (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0)
				{
					set_motion(STOP);
					delay(MOTION_DELAY);
				}
				else 
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			}
			else
			{
				if (kil_dist_NE < DIAGONAL_THRESHOLD_LOWER && kil_dist_NE > 0)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
				}

				set_motion(FORWARD);
				delay(MOTION_DELAY);
			}
		}
		else if (kilo_uid > (Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1) && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows)
		{						// kilobot uid 8
			if (alpha_deg[0] >= MINIMUM_DEGREES && alpha_deg[0] <= MAXIMUM_DEGREES)
			{
				if (kil_dist_NE >= DIAGONAL_THRESHOLD_LOWER && kil_dist_NE <= DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(FORWARD);		
					delay(MOTION_DELAY);			
				}
				else if (kil_dist_NE < DIAGONAL_THRESHOLD_LOWER && kil_dist_NE > 0)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
					
					if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0))
					{
						set_motion(STOP);	
						delay(MOTION_DELAY);								
					} 
					else 
					{
						set_motion(FORWARD);
						delay(MOTION_DELAY);
					}
				}
				else if (kil_dist_NE > DIAGONAL_THRESHOLD_UPPER)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);
					
					if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0))
					{
						set_motion(STOP);	
						delay(MOTION_DELAY);								
					}
					else
					{
						set_motion(FORWARD);
						delay(MOTION_DELAY);
					}
				}
			}
			else if (alpha_deg[0] < MINIMUM_DEGREES && alpha_deg[0] > 0)
			{
				set_motion(RIGHT);
				delay(MOTION_DELAY);

				if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0))
				{
					set_motion(STOP);	
					delay(MOTION_DELAY);								
				}
				else 
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			}
			else if (alpha_deg[0] > MAXIMUM_DEGREES)
			{
				set_motion(LEFT);
				delay(MOTION_DELAY);

				if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0))
				{
					set_motion(STOP);		
					delay(MOTION_DELAY);							
				}
				else
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			}
			else
			{
				if (kil_dist_NE < DIAGONAL_THRESHOLD_LOWER && kil_dist_NE > 0)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
				}
				set_motion(FORWARD);
				delay(MOTION_DELAY);
			}
		}
		else if (kilo_uid == Num_Matrix_Coloumns * Num_Matrix_Rows)
		{																// kilobot uid 9
			if (alpha_deg[0] >= MINIMUM_DEGREES && alpha_deg[0] <= MAXIMUM_DEGREES)
			{
				if (kil_dist_N - kil_dist_W > DIFFERENCE_THRESHOLD)
				{																										
					set_motion(RIGHT);
					delay(MOTION_DELAY);
				}
				else if (kil_dist_W - kil_dist_N > DIFFERENCE_THRESHOLD)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
				}		
				if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0))
				{
					set_motion(STOP);
					delay(MOTION_DELAY);	
				}
				else 
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			}
			else if (alpha_deg[0] < MINIMUM_DEGREES && alpha_deg[0] > 0)
			{
				if (kil_dist_N - kil_dist_W > DIFFERENCE_THRESHOLD)
				{																	
					set_motion(RIGHT);
					delay(MOTION_DELAY);
				}
				else if (kil_dist_W - kil_dist_N > DIFFERENCE_THRESHOLD)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
				}
				if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0))
				{
					set_motion(STOP);
					delay(MOTION_DELAY);
				}
				else
				{
					set_motion(FORWARD);
					delay(MOTION_DELAY);
				}
			}
			else if (alpha_deg[0] > MAXIMUM_DEGREES)
			{
				set_motion(STOP);
				delay(MOTION_DELAY);				
			}
			else
			{				
				if ((kil_dist_W < STRAIGHT_THRESHOLD && kil_dist_W > 0) || (kil_dist_N < STRAIGHT_THRESHOLD && kil_dist_N > 0))
				{
					set_motion(STOP);
					delay(MOTION_DELAY);
				}
				else
				{
					set_motion(FORWARD);	
					delay(MOTION_DELAY);
				}
			}
		}
		else 					// kilobot uid 5
		{																								
			if ((alpha_deg[0] <= MINIMUM_DEGREES && alpha_deg[0] > 0) || alpha_deg_two[0] >= MAXIMUM_DEGREES)
			{
				if (kil_dist_SW - kil_dist_NE >= DIFFERENCE_THRESHOLD)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
				}
				else if (kil_dist_NE - kil_dist_SW >= DIFFERENCE_THRESHOLD)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);					
				}
				set_motion(STOP);
				delay(MOTION_DELAY);
			}
			else if (alpha_deg[0] >= MAXIMUM_DEGREES || (alpha_deg_two[0] <= MINIMUM_DEGREES && alpha_deg_two[0] > 0))
			{
				if (kil_dist_SW - kil_dist_NE >= DIFFERENCE_THRESHOLD)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
				}
				else if (kil_dist_NE - kil_dist_SW >= DIFFERENCE_THRESHOLD)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);					
				}
				set_motion(FORWARD);
				delay(MOTION_DELAY);
			}
			else
			{
				if (kil_dist_SW - kil_dist_NE >= DIFFERENCE_THRESHOLD)
				{
					set_motion(LEFT);
					delay(MOTION_DELAY);
				}
				else if (kil_dist_NE - kil_dist_SW >= DIFFERENCE_THRESHOLD)
				{
					set_motion(RIGHT);
					delay(MOTION_DELAY);					
				}
				set_motion(FORWARD);																					// the priority is to move forward to not brake the system
				delay(MOTION_DELAY);
			}
		}


		//----------------------------------------------------------------------------------------------------------
		//Clearing the variables twice per second

		if (kilo_ticks > last_timereset + 64){ 								// twice per second the variables are cleaned
				
			for (i = 0; i < 9; ++i){
			 	for (j = i; j < 9; ++j){
			 			
		 			kil_dist_matrix[i][j] = 0;
		 			kil_dist_matrix[j][i] = 0;
				}		 			
			}
			alpha_deg[0] = 0;
			alpha_deg[1] = 0;
			alpha_deg[2] = 0;

			last_timereset = kilo_ticks;

			set_color(RGB(0,0,0));
		}

		//----------------------------------------------------------------------------------------------------------
		State = 0;
	}
}


// --------------------------------------------------------------------------------------------------------------
// ----------------------------------------- Message Recieve Function -------------------------------------------
// --------------------------------------------------------------------------------------------------------------

void message_rx(message_t *m, distance_measurement_t *distance_measurement){

	if (m->type == 120) { // initial identification
		
		int id = (m->data[0] << 8) | m->data[1];
		if (id == kilo_uid) {
			set_color(RGB(0,0,3));
		} else {
			set_color(RGB(3,0,0));
		} 
	
	} else {

		new_message = 1;

		composed_message_rx = m->data[0];

		operating_mode_rx = composed_message_rx >> 7;		// only the first bit is used to transmit the operation mode
	  	message_ID = composed_message_rx & 127;				// & 01111111 -> considering the latest seven bits
	 	ID_N_dist = m->data[1];										// receiving distances from a near Kilobot ad its neighbourhood
	 	ID_NE_dist = m->data[2];
	 	ID_E_dist = m->data[3];
	 	ID_SE_dist = m->data[4];
	 	ID_S_dist = m->data[5];
	 	ID_SW_dist = m->data[6];
	 	ID_W_dist = m->data[7];
	 	ID_NW_dist = m->data[8]; 
	  	message_distance = estimate_distance(distance_measurement);   

		
		if (message_ID != 0 && message_distance != 0){				// filtering the distances detected, saving them following the proper order in the matrix of distances

			for (l = 1; l < 9; ++l){
				
				if(neighbours_IDs_array[l] == message_ID){

					kil_dist_matrix[0][l] = message_distance;									

					break;
				}				
			}

			if(message_ID == kilo_uid - 1)				// saving the distances of the neighbourhood transmitted in the message
			{								
				kil_dist_matrix[7][8] = ID_N_dist;		// distance between the Kilobot ID, from which the message is received, and its neighbour at the NORD.
				kil_dist_matrix[7][1] = ID_NE_dist;
				kil_dist_matrix[7][0] = ID_E_dist;
				kil_dist_matrix[7][5] = ID_SE_dist;
				kil_dist_matrix[7][6] = ID_S_dist;
			}
			else if(message_ID == kilo_uid + 1)
			{
				kil_dist_matrix[3][2] = ID_N_dist;
				kil_dist_matrix[3][1] = ID_NW_dist;
				kil_dist_matrix[3][0] = ID_W_dist;
				kil_dist_matrix[3][5] = ID_SW_dist;
				kil_dist_matrix[3][4] = ID_S_dist;
			}
			else if(message_ID == kilo_uid - Num_Matrix_Coloumns)
			{
				kil_dist_matrix[1][2] = ID_E_dist;
				kil_dist_matrix[1][3] = ID_SE_dist;
				kil_dist_matrix[1][0] = ID_S_dist;
				kil_dist_matrix[1][7] = ID_SW_dist;
				kil_dist_matrix[1][8] = ID_W_dist;	
			}
			else if(message_ID == kilo_uid + Num_Matrix_Coloumns)
			{
				kil_dist_matrix[5][0] = ID_N_dist;
				kil_dist_matrix[5][3] = ID_NE_dist;
				kil_dist_matrix[5][4] = ID_E_dist;
				kil_dist_matrix[5][6] = ID_W_dist;
				kil_dist_matrix[5][7] = ID_NW_dist;	
			}
			else if(message_ID == kilo_uid - Num_Matrix_Coloumns - 1)
			{
				kil_dist_matrix[8][1] = ID_E_dist;
				kil_dist_matrix[8][0] = ID_SE_dist;
				kil_dist_matrix[8][7] = ID_S_dist;
			}
			else if(message_ID == kilo_uid - Num_Matrix_Coloumns + 1)
			{
				kil_dist_matrix[2][3] = ID_S_dist;
				kil_dist_matrix[2][0] = ID_SW_dist;
				kil_dist_matrix[2][1] = ID_W_dist;
			}
			else if(message_ID == kilo_uid + Num_Matrix_Coloumns - 1)
			{
				kil_dist_matrix[6][7] = ID_N_dist;
				kil_dist_matrix[6][0] = ID_NE_dist;
				kil_dist_matrix[6][5] = ID_E_dist;
			}
			else if(message_ID == kilo_uid + Num_Matrix_Coloumns + 1)
			{
				kil_dist_matrix[4][3] = ID_N_dist;
				kil_dist_matrix[4][0] = ID_NW_dist;
				kil_dist_matrix[4][5] = ID_W_dist;
			}
			else
			{
				// the message has come from a neighbour of a neighbour
			}		
		}
	}	
}

// --------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Main ----------------------------------------------------
// --------------------------------------------------------------------------------------------------------------


int main(){

  	kilo_init();
  	//debug_init();
  	kilo_message_rx = message_rx;
  	kilo_message_tx = message_tx;
  	kilo_message_tx_success = tx_message_success;
  	kilo_start(setup, loop);
    
  	return 0;
}