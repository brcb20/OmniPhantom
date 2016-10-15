// Test.cpp : définit le point d'entrée pour l'application console.
//

#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
#include <time.h>
# include "conio.h"
# include <string.h>
#define FALSE 0
#define TRUE 1
#endif

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "coordinates.h"

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

static const hduVector3Dd nominalBaseTorque(200.0,350.0,200.0); //mNm

HDCallbackCode HDCALLBACK jointTorqueCallback(void *data);
HDSchedulerHandle hEESimulation = HD_INVALID_HANDLE;

void mainLoop(void);
bool initDemo(void);

void PrintHelp()
{
    static const char help[] = {\
"Endonasal endoscopic surgery simulation Help\n\
---\n\
>> Use the first button on the stylus to advance through the\n\
>> different nasal zones. After the final zone is reached,\n\
>> the simulation will end.\n\
>> Use example: The device initially move to the starting pos\n\
>>				Press button once to move around\n\
				the nasal cavity\n\
>>              Press button again to move back to start pos\n\
>>              And so on...\n\
Warning: If the device attempts to guide you do not restrict it!\n\n\
P: Prints device state\n\
C: Continuously prints device state\n\
H: Prints help menu\n\
Q: Quits the program\n\
---"};
    
    printf("\n%s\n", help);
}

/* Synchronization structure. */
typedef struct
{
    HDdouble posValues[3];
    HDdouble jointAngleValues[3];   
    HDdouble gimbalAngleValues[3];   
} DeviceStateStruct;

/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
HDCallbackCode HDCALLBACK GetDeviceStateCallback(void *pUserData)
{
    DeviceStateStruct *pState = (DeviceStateStruct *) pUserData;

    hdGetDoublev(HD_CURRENT_POSITION, pState->posValues);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, pState->jointAngleValues);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, pState->gimbalAngleValues);

    return HD_CALLBACK_DONE;
}

/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
void PrintDeviceState(HDboolean bContinuous)
{
    int i, j;
	double *gimbalPos_Omni;
    DeviceStateStruct state;

    memset(&state, 0, sizeof(DeviceStateStruct));

    do
    {
        hdScheduleSynchronous(GetDeviceStateCallback, &state,
            HD_DEFAULT_SCHEDULER_PRIORITY);

		gimbalPos_Omni = gimbal_position(state.posValues, state.jointAngleValues, state.gimbalAngleValues);

        printf("\n");

		printf("Current position of the device (mm):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.posValues[i]);
        }
        printf("\n");

		printf("Stylus position in omni coord (mm):");
		for (i = 0; i < 3; i++)
        {
			printf(" %f", *(gimbalPos_Omni + i));
		}
		printf("\n");

        printf("Current Joint Angle Values (rad):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.jointAngleValues[i]);
        }
        printf("\n");

		state.gimbalAngleValues[0] -= 0.07;
        printf("Current Gimbal Angle Values (rad):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.gimbalAngleValues[i]);
        }
        printf("\n");

		if (bContinuous)
        {
#if defined(WIN32)
       		Sleep(500);
#elif defined(linux)
		struct timespec timeOut;
                timeOut.tv_sec = 0;
                timeOut.tv_nsec = 5*100000000;
                nanosleep(&timeOut, NULL);
#endif

        }

    } while (!_kbhit() && bContinuous);
}
/*******************************************************************************
 Main function.
 Initializes the device, starts the schedule, creates a schedule callback
 to handle endonasal endoscopic , waits for the user to press a button, exits
 the application.
*******************************************************************************/
int main(int argc, char* argv[])
{    
    HDErrorInfo error;
    /* Initialize the device, must be done before attempting to call any hd 
       functions. Passing in HD_DEFAULT_DEVICE causes the default device to be 
       initialized. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
        return -1;
    }

    printf("Surgical simulation Demo!\n");
    printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
	
	if (!initDemo())
    {
        printf("Demo Initialization failed\n");
        printf("Press any key to exit\n");
        _getch();
        
    }

	/* Schedule the main callback that will render forces to the device. */
    hEESimulation = hdScheduleAsynchronous(
		jointTorqueCallback, 0, 
		HD_MAX_SCHEDULER_PRIORITY);

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }

	PrintHelp();
	
	/* Start the main application loop */
    mainLoop();

    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();
    hdUnschedule(hEESimulation);

    /* Disable the device. */
    hdDisableDevice(hHD);

    return 0;
}

/******************************************************************************
 The main loop of execution.  Detects and interprets keypresses.  Monitors and 
 initiates error recovery if necessary.
******************************************************************************/
void mainLoop()
{
    int keypress;
    int nMotorIndex = 0;

    while (TRUE)
    {
        if (_kbhit())
        {
            keypress = _getch();
            keypress = toupper(keypress);
            
            switch (keypress)
            {
				case 'P': PrintDeviceState(FALSE); break;
                case 'C': PrintDeviceState(TRUE); break;
                case 'H': PrintHelp(); break;
                case 'Q': return;
                default: PrintHelp(); break;
            }
        }

        /* Check if the scheduled callback has stopped running */
        if (!hdWaitForCompletion(hEESimulation, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            _getch();
            return;
        }
    }
}

/*******************************************************************************
 Servo callback.  
 Called every servo loop tick.  Simulates a gravity well, which sucks the device 
 towards its center whenever the device is within a certain range.
*******************************************************************************/
HDCallbackCode HDCALLBACK jointTorqueCallback(void *data)
{
	double *gimbalPos_Omni;
	const HDdouble kGain = 0.5;
	/* # of zones = # of impair # in count */
	const int zoneCount = 7;
	/* i is used to keep count of simulation zone */
	static int i = 0;
	/* This is the position of the starting position in cartesian
       (i.e. x,y,z) space. */
    static const hduVector3Dd startPos(0,0,0);
	/* The cylinderPos is located along the z axis from 0 to -40 mm
	   the relevant value is the magnitude of x and y */
	const double cylinderRadius = 15.0; //mm
	hduVector3Dd cylinderPos(0,0,0);

	HDErrorInfo error;
    hduVector3Dd position;
	hduVector3Dd stylusPosition;

	hduVector3Dd force;
    hduVector3Dd positionTstart;
	hduVector3Dd positionInCyl;
	hduVector3Dd gimbalAngles;
	hduVector3Dd jointAngles;
	HDint nCurrentButton;
	HDint nLastButton;

	HHD hHD = hdGetCurrentDevice();

    /* Begin haptics frame.  ( In general, all state-related haptics calls
       should be made within a frame. ) */
    hdBeginFrame(hHD);

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_POSITION, position);
	/* Get other relevant Device parameters */
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES,gimbalAngles );
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES,jointAngles );
	hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButton);
    hdGetIntegerv(HD_LAST_BUTTONS, &nLastButton);

	gimbalAngles[0] -= 0.07; // Not perfectly calibrated (manually deduced value)
	/* Stylus end-effector position */
	gimbalPos_Omni = gimbal_position(position, jointAngles, gimbalAngles);
	stylusPosition = hduVector3Dd(gimbalPos_Omni);

	/* Setting force to zero */
	memset(force, 0, sizeof(hduVector3Dd));

	/* Move to starting Position if the simulation has just begun 
	   or if the user has pressed the button_1 a pair # of times 
	   to move to the next zone */
	if ((nCurrentButton & HD_DEVICE_BUTTON_1) == 0 &&
		(nLastButton & HD_DEVICE_BUTTON_1) != 0)
	{
		i++;
	}

	if (i < zoneCount+1)
	{
		if (!(i%2))
		{	
			/* >  positionTstart = startPos-position  < 
			Create a vector from the device position towards the start pos 
			. */
			hduVecSubtract(positionTstart, startPos, position);
			hduVecScale(force, positionTstart, kGain);
		}
		else
		{
			/* if device position outside cylinder
			create force vector towards x,y (0,0) */
			cylinderPos[2] = stylusPosition[2];
			if (hduVecDistance(cylinderPos, stylusPosition) > cylinderRadius)
			{
				hduVecSubtract(positionInCyl, cylinderPos, stylusPosition);
				hduVecScale(force, positionInCyl, 0.1);
			}
		}
	}

	/* Send the forces & torques to the device. */
	hdSetDoublev(HD_CURRENT_FORCE, force);

	/* End haptics frame. */
    hdEndFrame(hHD);

	if (i > zoneCount)
	{
		printf("Simulation finished\n");
		return HD_CALLBACK_DONE;
	}

    /* Check for errors and abort the callback if a scheduler error
       is detected. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, 
                      "Error detected while starting simulation\n");
        
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

   /* Signify that the callback should continue running, i.e. that
       it will be called again the next scheduler tick. */
    return HD_CALLBACK_CONTINUE;
}


bool initDemo(void)
{
    HDErrorInfo error;
    int calibrationStyle;
    printf("Calibration\n");

    hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);
    if (calibrationStyle & HD_CALIBRATION_AUTO || calibrationStyle & HD_CALIBRATION_INKWELL)
    {
        printf("Please prepare for starting the demo by \n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");
        _getch();
        return 1;
    }
    if (calibrationStyle & HD_CALIBRATION_ENCODER_RESET )
    {
        printf("Please prepare for starting the demo by \n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");

        _getch();

        hdUpdateCalibration(calibrationStyle);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
            printf("Calibration complete.\n\n");
            return 1;
        }
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Reset encoders reset failed.");
            return 0;           
        }
    }
}