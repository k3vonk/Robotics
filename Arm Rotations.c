#pragma config(Sensor, S1,     touchSensor,    sensorEV3_Touch)
#pragma config(Sensor, S3,     lightSensor,    sensorEV3_Color)
#pragma config(Sensor, S4,     gyroSensor,     sensorEV3_Gyro)
#define PI 3.14159265

//Stop all Motor Function
void pause() {
	motor[motorA]= 0;
	motor[motorB]= 0;
	motor[motorC]= 0;
	wait1Msec(1000);
}

//Pauses the robot from doing anything
/*void pause() {
	stopAllMotors();
	wait1Msec(1000);
}*/

/*
 *Calculates the tick for the encoder to rotate (arm sweep)
 *x = x- coordinate, y = y- coordinate
 */
float number(float x, float y) {
	float angleRadians, angleDegrees, numberTicks;

	//when x is 0 the angle is 90 degrees as tan of 90 does not exist we directly input the encoder value for 90 degrees
	if (x == 0){
		numberTicks = 300;
		return numberTicks;
	}else if (x > 0) { //(1st and 4th quadrant)
		angleRadians = atan(y/x);
		angleDegrees = (angleRadians)*(180/PI);
		numberTicks = (angleDegrees)*(10/2.95);
		return numberTicks;
	} else { //(2nd and 3rd quadrant)
		float w;
		w = abs(x);
		angleRadians = atan(y/w);
		angleDegrees = (angleRadians)*(180/PI);
		numberTicks = (180-angleDegrees)*(10/2.95);
		return numberTicks;
  }
}

//Turn counter clock-wise based on encoder ticks
void baseTurnCCW(float number)
{
	nMotorEncoder[motorC] = 0;

	while(nMotorEncoder[motorC] > -number) {
		motor[motorC] = -45;
	}
}

//Turn clock-wise based on encoder ticks
void baseTurnCW(float number)
{
	nMotorEncoder[motorC] = 0;

	while(nMotorEncoder[motorC] < number) {
		motor[motorC] = 45;
	}
}

/*
 *Sets the degrees of the Z rotational matrix
 */
void setZRotation(float *rotation, int degrees){
	rotation[0] = cosDegrees(degrees);
	rotation[1] = -sinDegrees(degrees);
	rotation[4] = sinDegrees(degrees);
	rotation[5] = cosDegrees(degrees);
}

/*
 *Multiplication of the rotational matrix and the claw matrix
 */
void matrixMultplication(float *rotation, float *claw, float *after){

	//Rotated matrix
  for(int i = 0; i < 4; i++){
  	float rowByColumnSum = 0;
  	int position = 0;

  	//Dividing the rotation array into a 4 x 4 matrix
   	for(int j = i*4; j < 4*(i+1); j++){
   		rowByColumnSum += rotation[j] * claw[position++];
    }

    after[i] = rowByColumnSum;
  }
}

//Move the arm down
void Down(float value){
	while(SensorValue(lightSensor) > (value * 25)){
		motor[motorB]= 30;
	}
}

//Go up until the light reflected percentage < 25
void Up(){
	while(SensorValue(lightSensor) < 25){
	motor[motorB]= -30;
	}
}

//Open claw
void ClawOut(){
	motor[motorA] = 30;
	wait1Msec(200);
}

//Close Claw
void ClawIn(){
	motor[motorA] = -30;
	wait1Msec(200);
}

//Pickup containing a set of actions
void pickUp(float down){
	//Open Claw
	pause();
	ClawOut();

	//Move Down
	pause();
	Down(down);

	//Close Claw
	pause();
	ClawIn();

	//Move Up
	pause();
	Up();

	pause();
}

/*
 *Resets the robots arm sweep to its starting point (no matrix)
 *Resets elbow pitch
 *Resets claw position
 */
void reset(){
	//reset arm sweep
	while(SensorValue(touchSensor) != 1){
		motor[motorC] = 40;
	}

	pause();

	//reset elbow pitch
	while(SensorValue(lightSensor) < 25){
		motor[motorB]= -30;
	}

	pause();

	//resets claw
	ClawIn();
}


// Setting the z value
void ZTranslation ( float * array, float z) {
	array [10]=z;
}

//Drops object at its new
void drop(float down) {
	//1. Move down crane
	pause();
	Down(down);

	//2. Open claw
	pause();
	ClawOut();

	//3. Move Up
	pause();
	Up();

	//4. Close claw
	pause();
	ClawIn();
	pause();

}

/* (test to see new matrix values)
int X = 0;
int Y = 0;
int Z = 0;
int A = 0;
*/

task main()
{
	reset(); //Ensures robot starts at the correct position

	//User enters what angle robot should move to and reposition pickup item
	int pickupDegrees = 135;
	int dropDegrees   = 45;

	//(2/25) <= lower <=  1 because the light sensor minum is 2 and the light sensor maximum is 25
	float lower = 0.14;  		//crane z axis for pickup
	float dropLower = 0.08;	//crane z axis for drop

	//Default rotational Matrix about the Z axis (4x4)
	float rotationZMatrix[] = { cosDegrees(1), -sinDegrees(1)	, 0, 0,
							 								sinDegrees(1), cosDegrees(1)  , 0, 0,
							 								0									 , 0				, 1, 0,
							 								0									 , 0			  , 0, 1};

	//Default translational Matrix about the Z axis (4x4)
	float translationZAxis[] = { 1, 0	, 0, 0,
							 								0 , 1 , 0, 0,
							 								0 , 0	, 1, 0,
							 								0 , 0 , 0, 1};

	//4x1 Matrix position of our claw at its starting position
  float clawMatrix[] = { 1,
  	                     0,
  	                     1,
  	                     1};

  float rotatedMatrix[4]; //The matrix multiplication of [4x4][4x1]  (rotation)(position)

  //Sets the Z matrix with the pickup degrees
	setZRotation(&rotationZMatrix[0], pickupDegrees);

	//Multiplies the two matrices
	matrixMultplication(rotationZMatrix, clawMatrix, &rotatedMatrix[0]);

	/* (to see the what the values of the new matrix is) - test
	X = rotatedMatrix[0];
	Y = rotatedMatrix[1];
	Z = rotatedMatrix[2];
	A = rotatedMatrix[3];
	*/

	//////////////////////STEP ONE//////////////////////////////

	//rotation based on the new matrix
	float tick = number(rotatedMatrix[0], rotatedMatrix[1]);
	baseTurnCCW(tick);
	pause();

	//pickup & reset
	ZTranslation( &translationZAxis[0], lower);
	matrixMultplication(translationZAxis, rotatedMatrix,&rotatedMatrix[0]);
	pickUp(rotatedMatrix[2]);
  baseTurnCW(tick);
	pause();

	//////////////////////STEP TWO//////////////////////////////

	//Matrix setup
	setZRotation(&rotationZMatrix[0], dropDegrees);
	matrixMultplication(rotationZMatrix, clawMatrix, &rotatedMatrix[0]);

	//rotation of robot
	tick = number(rotatedMatrix[0], rotatedMatrix[1]);
	baseTurnCCW(tick);
	pause();

	//Drop and reset
	ZTranslation( &translationZAxis[0], dropLower);
	matrixMultplication(translationZAxis, rotatedMatrix,&rotatedMatrix[0]);
	drop(rotatedMatrix[2]);
  baseTurnCW(tick);
	pause();


}
