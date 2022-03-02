package exerciseTracker;


public class ExerciseTracker 
{
    // Boolean to set the context frame of reference to calculate pitch and roll angles
    // Default is vertical direction. 
    boolean useHorizontalReference = false; 
    boolean invertYaxis = false;
    boolean doubleTheSensitivity = false;
    final int Fs = 50;// Sampling Frequency
    
    //Exercise States
    final int EXERCISE_STATE_NULL = -1;
    final int EXERCISE_STATE_INITIALISED = 0;
    final int EXERCISE_STATE_STARTED = 1;
    final int EXERCISE_STATE_PAUSED = 2;
    
    // Quasistatic range is the range below which ACC values are considered s static
    final double QUASISTATIC_RANGE = 0.5;
    
    // Sensitivity Vlues used for calibration
    final double sens = 0.0048;
    
    
    double refVec1[] = {0, 0, 0}, refVec2[] = {0, 1, 0};
    
    // sendForDisplay is used to send exercise parameter for display on screen etc
    float[] sendForDisplay = {0, 0, 0, 0};
	int exerciseState = EXERCISE_STATE_NULL, samplesElapsed, trend;
	double [] initGravDir = {0, 1 ,0}, initVec = {1,0,0};
    public int count;
    double rotationAngle = 0;
    
    // Variable to holdrotated XYZ ACC values
    double rotatedX, rotatedY, rotatedZ;
	protected double [] currentVec = {1,0,0}, initialVector = {1, 0 ,0};
    protected double currentMag, temp;
    public int MAX_EXERCISE_DURATION ,MIN_EXERCISE_DURATION;
	
    //Init buffers
	CircleBufferDouble initializationBufferX = null;
	CircleBufferDouble initializationBufferY = null;
	CircleBufferDouble initializationBufferZ = null;
	
	//Segment buffers
	CircleBufferDouble segmentBufferX = null;	
	CircleBufferDouble segmentBufferY = null;	
	CircleBufferDouble segmentBufferZ = null;	
	
	double calibratedX,calibratedY,calibratedZ, pitch, azimuth, roll, currentMeanX, currentMeanY, currentMeanZ; 
	
	//Exercise Manager
	ExerciseTrackerManager exerciseManager = null;
	
	// Moving Average Filter used to remove trend from the ACC values
        MovingAverageFilter shortTrendFilterX = new MovingAverageFilter((int)(0.5 *Fs) );
        MovingAverageFilter shortTrendFilterY = new MovingAverageFilter((int) (0.5 *Fs));
        MovingAverageFilter shortTrendFilterZ = new MovingAverageFilter((int) (0.5 *Fs));
        //int trendRemovedX, trendRemovedY,trendRemovedZ;
        int shortTrendX, shortTrendY, shortTrendZ;
        
        // Moving Average Filter used to get DELTA of XYZ ACC values
        MovingAverageFilter longTrendFilterX = new MovingAverageFilter((int)(3 *Fs) );
        MovingAverageFilter longTrendFilterY = new MovingAverageFilter((int)(3 *Fs));
        MovingAverageFilter longTrendFilterZ = new MovingAverageFilter((int)(3 *Fs));
        int longTrendRemovedX, longTrendRemovedY, longTrendRemovedZ;

        // Delta ACC values
        double deltaX, deltaY, deltaZ;
        
	protected ExerciseTracker(ExerciseTrackerManager manager)
	{
		exerciseManager = manager;
	}
       /**
        * Function to rotate XY acc Values using pitch angle.
        */
        protected void rotateToGlobalFrameofRef2D()
{
        rotationAngle = 3.14 *pitch/180;
        if(calibratedX <0)
            rotationAngle = -3.14 *pitch/180;
        rotatedX = calibratedX*Math.cos(rotationAngle) - calibratedY * Math.sin(rotationAngle) ;
        rotatedY = calibratedX*Math.sin(rotationAngle) + calibratedY * Math.cos(rotationAngle) ;
        // Send Rotated values for display
        sendForDisplay[0] = (float)rotatedX;
        sendForDisplay[1] = (float)rotatedY;
        sendForDisplay[2] = (float)0;
       	exerciseManager.sendExerciseEvent(ZSensorAnalyzerEventListener.ROTATED_ACCEL_ARRAY, sendForDisplay);
}

public void addAccToRotatedBuffer()
{
    
}
/**
  * Function to calculate Delta of ACC XYZ values by subtracting current saples from mean XYZ(calculated using MA filter)
  */        
 protected void getDeltaAccel() 
 {
        longTrendRemovedX = removeLongTrendX((int) (calibratedX*100));
        longTrendRemovedY = removeLongTrendY((int) (calibratedY*100));
        longTrendRemovedZ = removeLongTrendZ((int) (calibratedZ*100));
        deltaX = longTrendRemovedX/100.0f;
        deltaY = longTrendRemovedY/100.0f;
        deltaZ = longTrendRemovedZ/100.0f;
        
        addDeltaToBuffer();// Function Add delta values to the concerned circular buffer
        sendForDisplay[0] = (float)calibratedX;//deltaX;
        sendForDisplay[1] = (float)calibratedY;//deltaY;
       	sendForDisplay[2] = (float)calibratedZ;//deltaZ;
        exerciseManager.sendExerciseEvent(ZSensorAnalyzerEventListener.DELTA_ACCEL_ARRAY, sendForDisplay);
 }    
    protected void addDeltaToBuffer()
{
 
}
protected void getNeckOrientation() 
{
     // Change the reference Vector if Exercise requires a horizontal World Frame of Reference
       if(useHorizontalReference)
       {   refVec2[0] = 1;
           refVec2[1] = 0;
           refVec2[2] = 0;
       }
        shortTrendX = getShortTrendX((int) (calibratedX*100));
        shortTrendY = getShortTrendY((int) (calibratedY*100));
        shortTrendZ = getShortTrendZ((int) (calibratedZ*100));
        
        refVec1[0] = shortTrendX/100.0f;
        refVec1[1] = shortTrendY/100.0f;
        refVec1[2] = shortTrendZ/100.0f;
        azimuth = VectorFunctions.findAngle(refVec1, refVec2);
        
        
        exerciseManager.communicator.notifyAnalyzerListeners(0, ZSensorAnalyzerEventListener.EXERCISE_NECK_ANGLE, (int)(azimuth));

        refVec1[0] = shortTrendX/100.0f;
        refVec1[1] = shortTrendY/100.0f;
        refVec1[2] = shortTrendZ/100.0f;
        
        temp = Math.sqrt(Math.pow(refVec1[0], 2) + Math.pow(refVec1[1], 2) + Math.pow(refVec1[2], 2));
        refVec1[0] =  refVec1[0]/temp;
        refVec1[1] =  refVec1[1]/temp;
        refVec1[2] =  0*refVec1[2]/temp;
        refVec2[0] = 0;
        refVec2[1] = 1;
        refVec2[2] = 0;
        
        // For PITCH ANGLE Calculation
        // Change the reference Vector if Exercise requires a horizontal World Frame of Reference
        if(useHorizontalReference)
       {   refVec2[0] = 1;
           refVec2[1] = 0;
           refVec2[2] = 0;
       }
        
        pitch = VectorFunctions.findAngle(refVec1, refVec2);
        exerciseManager.communicator.notifyAnalyzerListeners(0, ZSensorAnalyzerEventListener.EXERCISE_NECK_PITCH_ANGLE, (int)(pitch));

        refVec1[0] = shortTrendX/100.0f;
        refVec1[1] = shortTrendY/100.0f;
        refVec1[2] = shortTrendZ/100.0f;
        temp = Math.sqrt(Math.pow(refVec1[0], 2) + Math.pow(refVec1[1], 2) + Math.pow(refVec1[2], 2));
        
        
        refVec2[0] = 0;
        refVec2[1] = 1;
        refVec2[2] = 0;
        
        // For ROLL Calculation
        // Change the reference Vector if Exercise requires a horizontal World Frame of Reference

        if(useHorizontalReference)
       {   
           refVec2[0] = 1;
           refVec2[1] = 0;
           refVec2[2] = 0;
           refVec1[0] =  refVec1[0]/temp;
           refVec1[1] =  0*refVec1[1]/temp;
           refVec1[2] =  refVec1[2]/temp;
       }
       else
       {
            refVec1[0] =  0*refVec1[0]/temp;
            refVec1[1] =  refVec1[1]/temp;
            refVec1[2] =  refVec1[2]/temp;
       }
        
        roll = VectorFunctions.findAngle(refVec1, refVec2);
        exerciseManager.communicator.notifyAnalyzerListeners(0, ZSensorAnalyzerEventListener.EXERCISE_NECK_ROLL_ANGLE, (int)(roll));
        sendForDisplay[0] = (float)azimuth;
        sendForDisplay[1] = (float)pitch;
       	sendForDisplay[2] = (float)roll;
        //exerciseManager.sendExerciseEvent(ZSensorAnalyzerEventListener.NECK_ANGLE_ARRAY, sendForDisplay);

  }

protected void update(float x, float y, float z)
{       
		calibrateData(x,y,z);
                sendForDisplay[0] = (float)calibratedX;
                sendForDisplay[1] = (float)calibratedY;
       	        sendForDisplay[2] = (float)calibratedZ;
                //exerciseManager.sendExerciseEvent(ZSensorAnalyzerEventListener.DELTA_ACCEL_ARRAY, sendForDisplay);

	        getNeckOrientation();
                addNeckAgletoBuffer();
                getDeltaAccel();
                rotateToGlobalFrameofRef2D();
                addAccToRotatedBuffer();
		if (exerciseState == EXERCISE_STATE_NULL)
		{
			checkInitialization();
		}
		else
		{
                    segmentData();
		}	
}
	
public void initializeTracker()
        {
        }
public void addNeckAgletoBuffer()
        {
        }

protected void calibrateData(float x, float y, float z)
	{     if(!doubleTheSensitivity)          
              {      calibratedX = x*sens;
                     calibratedY = y*sens;
                     calibratedZ = z*sens;
              }
        else{
            calibratedX = 2*x*sens;
            calibratedY = 2*y*sens;
            calibratedZ = 2*z*sens;
        }
            if(invertYaxis)
            calibratedY = -calibratedY;
        }
 
protected void reset()
        {
            initializationBufferX.reset();initializationBufferY.reset();initializationBufferZ.reset();
            segmentBufferX.reset();segmentBufferY.reset();segmentBufferZ.reset();
        }
        
protected void segmentData()
        {   
        }
protected void checkInitialization()
{
            initializationBufferX.addElement(calibratedX);
            initializationBufferY.addElement(calibratedY);
            initializationBufferZ.addElement(calibratedZ);
            initializationBufferX.calculateCurrentStats();
            initializationBufferY.calculateCurrentStats();
            initializationBufferZ.calculateCurrentStats();

            currentVec[0] = calibratedX;
            currentVec[1] = calibratedY;
            currentVec[2] = calibratedZ;
            if (initializationBufferX.getStandardDeviation() < QUASISTATIC_RANGE && initializationBufferY.getStandardDeviation() < QUASISTATIC_RANGE && initializationBufferZ.getStandardDeviation() < QUASISTATIC_RANGE && VectorFunctions.isStatic(calibratedX, calibratedY, calibratedZ) && VectorFunctions.isFlat(currentVec, initGravDir)) 
            {
                exerciseState = EXERCISE_STATE_INITIALISED;
                initialVector[0] = calibratedX;initialVector[1] = calibratedY;initialVector[2] = calibratedZ;
                exerciseManager.sendExerciseEvent(ZSensorAnalyzerEventListener.EXERCISE_INITIALISED, 0);
            } 
}
	
	protected void cropSegmentedData()
	{
	}
	
	protected void segmentValidation()
	{
		//When segment validated as exercise
		//sendExerciseEvent();
	}
		
    //Get Mean
    protected int getShortTrendX(int accVal) 
    {
        trend = shortTrendFilterX.update(accVal);
        return trend;
    }
    
    //Get Mean
    protected int getShortTrendY(int accVal) 
    {
         trend = shortTrendFilterY.update(accVal);
        return trend;
    }
    
    //Get Mean
    protected int getShortTrendZ(int accVal) 
    {
         trend = shortTrendFilterZ.update(accVal);
        return trend;
    }
    
     protected int removeLongTrendX(int accVal) 
     {
        //Dtrend signal via moving average filter
        trend = longTrendFilterX.update(accVal);
        currentMeanX = trend;
        accVal -= trend;
        return accVal;
        
    }
    
    //Remove trend
    protected int removeLongTrendY(int accVal) 
    {
        //Dtrend signal via moving average filter
        trend = longTrendFilterY.update(accVal);
        currentMeanY = trend;
        accVal -= trend;
        return accVal;
    }
    
    //Remove trend
    protected int removeLongTrendZ(int accVal) 
    {
        //Dtrend signal via moving average filter
        trend = longTrendFilterZ.update(accVal);
        currentMeanZ = trend;
        accVal -= trend;
        return accVal;
    }
    
    
    
	
}
