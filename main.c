const float WHEEL_RADIUS = 2.75;
const float CLICK_TO_CM = PI * WHEEL_RADIUS / 180;
const float CM_TO_CLICK = 180 / (PI * WHEEL_RADIUS);
const int ROBOT_WIDTH = 30;
const int ROBOT_LENGTH = 31;

const int ultra = S1;
const int rightColor = S2;
const int leftColor = S3;
const int gyro = S4;

const int DRIVE_RIGHT = motorA;
const int SPIN_BRUSH = motorB;
const int RAISE_BRUSH = motorC;
const int DRIVE_LEFT = motorD;


const float GLOBAL_SPEED=14.71;    // cm/s
const float TURN_TIME=17.05;      //

const int DRIVE_POWER = 30;
const int TURN_POWER = 15;

const int MIN_OBJ_DIST = 30;

const float REV_DIST = 44; // Distance to reverse before a turn

const int REV_ALIGN = 19;

const int BRUSH_DEGREES = 120;
const int SPIN_POWER = 100;
const int RAISE_POWER = 30;

const int MAX_WAIT_TIME = 10*1000;



void DriveSame(int motorPower);
void DriveDist(int motorPower, int distance);
void DriveDif(int motorPowerLeft, int motorPowerRight);
void DrivingDirections(float *directions,float desk_length, float desk_width);
int BlowAndMop();
int EstimatedTime(float *directions, int cleaningOptions);
float MeasureEdge();
bool DriveCleaningPath(float const *deskData, int estTime);
void TurnRobot(bool turnCW, int angle, float driveBack);
int CheckObjects();
void ConfigureSensors();
void StartMop();
void StopMop();
int DriveRectangle(float length, float width, bool guided_edge);
int DriveToEdge(int motorPower);
int DriveEdge(float dist);


task main()
{
    ConfigureSensors();

    int cleaningOptions = BlowAndMop();

    // If not timed out choosing cleaning options, continue with cleaning
    if(cleaningOptions != -1)
    {
        float deskData[3];
        float deskLength = 0;
        float deskWidth = 0;

        int estTime = 0;

        for(int run = 1; run <= 4; run++) // Map the desk
        {
            if(run ==1 || run == 3)
            {
                deskLength=+(MeasureEdge()+ROBOT_LENGTH);
                TurnRobot(true,90,REV_DIST); // Robot starts at bottom left corner
            }
            else
            {
                deskWidth=+ (MeasureEdge()+ROBOT_LENGTH);
                TurnRobot(true,90,REV_DIST);
            }
            DriveDist(-DRIVE_POWER,REV_ALIGN); // Align the desk
        }

        DrivingDirections(deskData, deskLength, deskWidth); // Store the dimensions

        estTime = EstimatedTime(deskData, cleaningOptions); // Store and display estimated time

        if(cleaningOptions == 0) // Mop only
        {
            StartMop();
            DriveCleaningPath(deskData, estTime);
            StopMop();
        }

        else // Blow and Mop
        {
            // Prompt user to start blower
            eraseDisplay();
            displayString(6, "Turn on the blower and");
            displayString(7,"press the enter button");
            time1[T1]=0;
            while(!getButtonPress(buttonEnter) && time1[T1] < MAX_WAIT_TIME)
            {}

            // User timed out, program ends
            if(time1[T1] >= MAX_WAIT_TIME)
            {
                eraseDisplay();
                displayString(6, "Goodbye");
                wait1Msec(2500);
            }

            else
            {
                // Blower
                if(DriveCleaningPath(deskData, estTime))
                {
                    // Prompt user to stop blower
                    eraseDisplay();
                    displayString(6, "Turn off the blower and");
                    displayString(7,"press the enter button");
                    time1[T1]=0;
                    while(!getButtonPress(buttonEnter) && time1[T1] < MAX_WAIT_TIME)
                    {}

                    // If timed out, exit program
                    if(time1[T1] >= MAX_WAIT_TIME)
                    {
                        eraseDisplay();
                        displayString(6, "Goodbye");
                        wait1Msec(2500);
                    }

                        // Mop
                    else
                    {
                        StartMop();
                        DriveCleaningPath(deskData, estTime/2);
                        StopMop();
                    }

                }
            }

        }

    }
}

void ConfigureSensors()
{
    SensorType[ultra] = sensorEV3_Ultrasonic;
    wait1Msec(50);
    SensorType[rightColor] = sensorEV3_Color;
    wait1Msec(50);
    SensorMode[rightColor] = modeEV3Color_Reflected;
    wait1Msec(50);
    SensorType[leftColor] = sensorEV3_Color;
    wait1Msec(50);
    SensorMode[leftColor] = modeEV3Color_Reflected;
    wait1Msec(50);
    SensorType[gyro] = sensorEV3_Gyro;
    wait1Msec(50);
    SensorMode[gyro] = modeEV3Gyro_Calibration;
    wait1Msec(50);
}

// Same motor power
void DriveSame(int motorPower)
{
    motor[DRIVE_LEFT] = motor[DRIVE_RIGHT] = motorPower;
}

// Different motor power
void DriveDif(int motorPowerLeft, int motorPowerRight)
{
    motor[DRIVE_LEFT] = motorPowerLeft;
    motor[DRIVE_RIGHT] = motorPowerRight;
}

// Creates driving directions for cleaning path
void DrivingDirections(float *directions,float desk_length, float desk_width)
{
    float excess=(desk_width/2)%ROBOT_WIDTH;

    directions[0]=desk_width/2/ROBOT_WIDTH; //1st element is #rectangles
    directions[1]=desk_length; //2nd element is length per run

    //Checking for excess and setting the third element as the distance to drive back
    if(excess < 2)
    {
        directions[2]=0; //If excess<2cm, we'll ignore it
    }
    else
    {
        directions[2]=(float)ROBOT_WIDTH-excess; //If excess>2cm, we'll have the robot drive back this distance so that the final width is 15cm (PASS_WIDTH)
    }

    directions[3]=desk_width/2; // Driven width = 1/2 desk width (ie. rectangle driving path)
}

// Returns -1 if time out, 1 if blow and mop, 0 if mop only
int BlowAndMop()
{
    eraseDisplay();
    displayString(0, "Press and hold an option:");
    displayString(5, "Left button: mopping only");
    displayString(9, "Right button:");
    displayString(10, "blowing and mopping");

    time1[T1] = 0;

    while(!getButtonPress(buttonLeft) && !getButtonPress(buttonRight) && time1[T1] < MAX_WAIT_TIME)
    {} // Wait for button press and time out

    if(getButtonPress(buttonLeft))
    {
        eraseDisplay();
        displayString(6, "Mopping only selected");
        wait1Msec(2500);
        return 0;
    }
    else if (getButtonPress(buttonRight))
    {
        eraseDisplay();
        displayString(6, "Blowing and mopping selected");
        wait1Msec(2500);
        return 1;
    }

    else
    {
        eraseDisplay();
        displayString(6, "Goodbye");
        wait1Msec(2500);
        return -1;
    }
}

// Calculates estimated time for cleaning cycle
int EstimatedTime(float *directions, int cleaningOptions)
{
    float totRecDist=0,calcTime=0;
    //num rectangles	//two lengths  //one normal width   //one offset width
    totRecDist=directions[0]*((2*directions[1])+directions[3]+(directions[3]-ROBOT_WIDTH));

    if(directions[2] != 0)
    {								//adding an addidional rectangle for excess consisting of two lengths and two modified widths
        totRecDist+=(2*directions[1] + (directions[3]-directions[2]) + 2*directions[3]);
        calcTime+=4*TURN_TIME;
    }

    calcTime+=(totRecDist/(float)GLOBAL_SPEED)+((4* (int)directions[0])*TURN_TIME);
    //total non-turn time									//total turn time

    // If mop and vacuum, double cleaning time
    if(cleaningOptions == 1)
    {
        calcTime*=2;
    }

    eraseDisplay();
    displayString(6,"Estimated Time: %d s", (int)calcTime);
    wait1Msec(3000);

    return (int)calcTime*1000; // Returns in seconds

}


float MeasureEdge()
{
    int edge = SensorValue[leftColor];
    int table = SensorValue[rightColor];
    bool end = false;
    float length=0;
    nMotorEncoder[DRIVE_LEFT]=0;

    while (end == false)
    {
        while (edge <= 1 && end == false) // while sensor is off the edge
        {

            DriveDif(DRIVE_POWER,DRIVE_POWER-8);

            edge = SensorValue[leftColor];

            table = SensorValue[rightColor];
            if (table < 1) // if the sonsor detects the end of the table
            {
                end = true;
            }
        }
        while (edge > 1 && end == false) // while sensor is on the table
        {
            DriveDif(DRIVE_POWER-4, DRIVE_POWER);
            edge = SensorValue[leftColor];

            table = SensorValue[rightColor];
            if (table < 1) // if the sonsor detects the end of the table
            {
                end = true;
            }
        }
    }
    DriveSame(0);
    length = nMotorEncoder[DRIVE_LEFT]*CLICK_TO_CM;
    return length;
}

bool DriveCleaningPath(float const *deskData, int estTime) //uses the drive rectangle function repeatedly until                                                             // table is covered
//returns true if path was completed without termination
{
    int timeElapsed = 0; //used to sum the total time elapsed

    for (int rectNum = 0; rectNum < (int)deskData[0]; rectNum++) //loops drive rectangle for each full rectangle
    {
        int status = 0;
        if(rectNum == 0) //for the first rectangle, uses colour sensor for guiding instead of gyro
        {
            status = DriveRectangle(deskData[1]-ROBOT_LENGTH, deskData[3]-(0.5*ROBOT_WIDTH),true);
            if(status == -1) //if -1 is returned from drive rectangle, terminate program
            {
                return false;
            }
            else //if not terminated during drive rectangle, add time to total time
            {
                timeElapsed += status;
            }
        }
        else
        {
            status = DriveRectangle(deskData[1]-ROBOT_LENGTH, deskData[3]-ROBOT_WIDTH,false); //for rest of
            //rectangle, guide with gyro, not colour
            if(status == -1)
            {
                return false;
            }
            else
            {
                timeElapsed += status;
            }
        }

        //update the time remaining after each rectangle
        eraseDisplay();
        displayString(6, "%d s left", (estTime-timeElapsed)/1000);

    }

    int lengthTime = 0; // variable to read in value returned from drive rectangle

    //The rest follows a format similar to drive rectangle to deal with the excess amount left
    //but is guided by the colour all the way around
    //look to DriveRectangle for more explanation
    lengthTime = DriveToEdge(DRIVE_POWER);
    if(lengthTime < 0)
    {
        return false;
    }
    else
    {
        timeElapsed += lengthTime;
    }
    time1[T1]=0;
    TurnRobot(true, 90, REV_DIST);
    DriveDist(-DRIVE_POWER,REV_ALIGN);
    timeElapsed+= (int)time1[T1];

    lengthTime = DriveEdge(deskData[3] + deskData[2]);
    if(lengthTime < 0)
    {
        return false;
    }
    else
    {
        timeElapsed += lengthTime;
    }
    time1[T1]=0;
    TurnRobot(true, 90, REV_DIST);
    DriveDist(-DRIVE_POWER,REV_ALIGN);
    timeElapsed+= (int)time1[T1];

    lengthTime = DriveEdge(deskData[1]);
    if(lengthTime < 0)
    {
        return false;
    }
    else
    {
        timeElapsed += lengthTime;
    }
    time1[T1]=0;
    TurnRobot(true, 90, REV_DIST);
    DriveDist(-DRIVE_POWER,REV_ALIGN);
    timeElapsed+= (int)time1[T1];

    lengthTime = DriveEdge(deskData[3]*2);
    if(lengthTime < 0)
    {
        return false;
    }
    else
    {
        timeElapsed += lengthTime;
    }
    time1[T1]=0;
    TurnRobot(true, 90, REV_DIST);
    DriveDist(-DRIVE_POWER,REV_ALIGN);
    timeElapsed+= (int)time1[T1];

    eraseDisplay();
    displayString(6, "%d s left", (estTime-timeElapsed)/1000);

    return true; // returns true if the whole process is completed without termination
}


// Turns the robot CW or CCW a certain # of deg with the option for to reverse before turning
void TurnRobot( bool turnCW,  int angle,  float driveBack)
{
    // If reverse chosen, drive back that distance
    if(driveBack > 0)
    {
        DriveDist(-DRIVE_POWER, driveBack);
        wait1Msec(250);
    }

    resetGyro(gyro);

    if(turnCW==true)
    {
        DriveDif(TURN_POWER, TURN_POWER-10);
    }
    else
    {
        DriveDif(TURN_POWER-10,TURN_POWER);
    }


    while(abs(getGyroDegrees(gyro)) < angle) // Works for CW and CCW
    {}
    DriveSame(0);
    wait1Msec(250);
}

// Return -1 if time is over 10000, 0 if no object in way and continue driving, return > 0 if time was waited
int CheckObjects()
{

    time1[T2]=0;

    if(SensorValue[ultra] < MIN_OBJ_DIST) // if object is detected
    {

        while (SensorValue[ultra] < MIN_OBJ_DIST && time1[T2] < MAX_WAIT_TIME) // while object is detected and wait time has not passed
        {
            motor[DRIVE_RIGHT] = motor[DRIVE_LEFT] = 0;
            eraseDisplay();
            displayString(5, "Robot will shut down in:");
            displayString(6, "%f", (MAX_WAIT_TIME - (int)time1[T2]) / 1000.0);
        }
// at this point, object has been removed or max wait time has passed
        if (time1[T2] < MAX_WAIT_TIME)
            // object has been removed before max wait time, program continues
        {
            return (int)time1[T2];
        }
        if (SensorValue[ultra] < MIN_OBJ_DIST && time1[T2] >= MAX_WAIT_TIME)
            // object has not been removed and max wait time has passed, program ends
        {
            return -1;
        }
    }
    return 0; // no object has been detected
}

// Lower mop and start spinning
void StartMop()
{
    nMotorEncoder[RAISE_BRUSH] = 0;
    motor[RAISE_BRUSH] = RAISE_POWER;
    while(nMotorEncoder[RAISE_BRUSH] < BRUSH_DEGREES)
    {}
    motor[RAISE_BRUSH] = 0;

    wait1Msec(250);
    motor[SPIN_BRUSH] = SPIN_POWER;
}

// Raise mops and start spinning
void StopMop()
{
    motor[SPIN_BRUSH] = 0;
    wait1Msec(250);

    nMotorEncoder[RAISE_BRUSH] = 0;
    motor[RAISE_BRUSH] = -1*RAISE_POWER;
    while(abs(nMotorEncoder[RAISE_BRUSH]) < BRUSH_DEGREES)
    {}
    motor[RAISE_BRUSH] = 0;
}

// Returns the length of time to drive rectangle OR -1 if timed out because object in path
// Allows for the first edge to be guided with color sensor
int DriveRectangle(float length, float width, bool guided_edge)
{
    int totalTime = 0;  // Time to drive rectangle
    int lengthTime = 0; // Time per length

    if (guided_edge) // If guided edge, call drive edge function and return time or time out
    {
        lengthTime = DriveEdge(length);
        if(lengthTime < 0)
        {
            return -1; // Time out
        }
        else
        {
            totalTime += lengthTime; // Add time if not time out
        }
    }
    else // No guided edge
    {
        // Process repeated with drive to edge (ie. use color sensor to reach end of table)
        lengthTime = DriveToEdge(DRIVE_POWER);
        if(lengthTime < 0)
        {
            return -1;
        }
        else
        {
            totalTime += lengthTime;
        }
    }

    // Reset timer, turn, add time for turn, drive width, check for time out
    time1[T1]=0;
    TurnRobot(true, 90, REV_DIST);
    DriveDist(-DRIVE_POWER,REV_ALIGN);
    totalTime+=time1[T1];
    lengthTime = DriveEdge(width);
    if(lengthTime < 0)
    {
        return -1;
    }
    else
    {
        totalTime += lengthTime;
    }

    // Reset timer, turn, add time for turn, drive length, check for time out
    time1[T1]=0;
    TurnRobot(true, 90, REV_DIST);
    DriveDist(-DRIVE_POWER,REV_ALIGN);
    totalTime+=time1[T1];
    lengthTime = DriveToEdge(DRIVE_POWER);
    if(lengthTime < 0)
    {
        return -1;
    }
    else
    {
        totalTime += lengthTime;
    }

    // Reset timer, turn, add time for turn, drive width but adjust for "next rectangle", check for time out
    time1[T1]=0;
    TurnRobot(true, 90, REV_DIST);
    DriveDist(-DRIVE_POWER,REV_ALIGN);
    totalTime+=time1[T1];
    lengthTime = DriveEdge(width-ROBOT_WIDTH);
    if(lengthTime < 0)
    {
        return -1;
    }
    else
    {
        totalTime += lengthTime;
    }

    // Reset timer, turn, add time for turn, realign, check for time out
    time1[T1]=0;
    TurnRobot(true, 90, REV_DIST);
    DriveDist(-DRIVE_POWER,REV_ALIGN);
    totalTime+=time1[T1];

    return totalTime;
}

void DriveDist(int motorPower, int distance) // given a motor power and distance, drives until distance reached
{
    motor[DRIVE_LEFT] = motor[DRIVE_RIGHT] = motorPower;
    nMotorEncoder[DRIVE_RIGHT]=0;
    while(abs(nMotorEncoder[DRIVE_RIGHT]*CLICK_TO_CM) < distance)
    {}
    motor[DRIVE_LEFT] = motor[DRIVE_RIGHT] = 0;
    wait1Msec(500);
}

int DriveToEdge(int motorPower)// guides robot with gyro until edge is detected by right colour sensor
// returns time elapsed or negative value if done program
{
    int timeStopped = 0;
    time1[T1] = 0;
    DriveSame(DRIVE_POWER);
    resetGyro(gyro);
    while(SensorValue[rightColor] > 0)
    {
        int pause_time = CheckObjects(); //Check objects will give -1 to terminate program or amount of time stopped
        if (pause_time == -1)
        {
            return -1; // end function and return int signifying to terminate program
        }
        else if (pause_time > 0)
        {
            timeStopped += pause_time; // if the robot was stopped, add to the total amount of time stopped
        }

        if(SensorValue[gyro] > 0) //adjusts the robot to adjust to have the gyro = 0 (robot goes straight)
        {
            DriveDif(motorPower-1,motorPower);
        }
        else
        {
            DriveDif(motorPower,motorPower-1);
        }
    }
    return time1[T1]-timeStopped; // return the time on the timer - the total time it was stopped for
}

int DriveEdge(float dist)
{
    int edge = SensorValue[leftColor];
    int table = SensorValue[rightColor];
    bool end = false;
    int timeStopped = 0;
    time1[T1] = 0;

    nMotorEncoder[DRIVE_LEFT]=0;

    while (end == false) // while end of table not detected
    {
        while (edge <= 1 && end == false) // while sensor is off the table
        {

            DriveDif(DRIVE_POWER,DRIVE_POWER-10);

            edge = SensorValue[leftColor];

            table = SensorValue[rightColor];
            if (table < 1 || dist*CM_TO_CLICK < nMotorEncoder[DRIVE_LEFT])
            {
                end = true; // if distance or end of table is reached, robot exits loop and stops motors
            }
            int pause_time = CheckObjects();
            if (pause_time == -1)
            {
                return -1; // object not removed, time limit exceeded, program ends
            }
            else if (pause_time > 0)
            {
                timeStopped += pause_time;
            }
        }
        while (edge > 1 && end == false) // if sensor detects the table
        {
            DriveDif(DRIVE_POWER-10, DRIVE_POWER);
            edge = SensorValue[leftColor];

            table = SensorValue[rightColor];
            if (table < 1 || dist*CM_TO_CLICK < nMotorEncoder[DRIVE_LEFT]) // if distance or end of table is reached, robot exits loop and stops motors
            {
                end = true;
            }
            int pause_time = CheckObjects();
            if (pause_time == -1)
            {
                return -1;
            }
            else if (pause_time > 0)
            {
                timeStopped += pause_time;
            }
        }
    }
    DriveSame(0);
    return time1[T1]-timeStopped; // time paused due to objects subtracted from time elapased
}
