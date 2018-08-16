/************************************************************
************************************************************/
#pragma once

/************************************************************
************************************************************/
#include "ofMain.h"
#include "ofxOsc_BiDirection.h"
#include "ofxNetwork.h"

/************************************************************
************************************************************/
#define ERROR_MSG(); printf("Error in %s:%d\n", __FILE__, __LINE__);

/************************************************************
************************************************************/

/**************************************************
**************************************************/
class MESSAGE_OUT{
private:

public:
	enum{
		CALIB_STARTED,
		READY_TO_STEP_FORWARD,
		CALIB_SUCCEEDED,
		CALIB_FAILED,
		TRACKING_STOPPED,
		
		NUM_MESSAGE_TYPES,
	};
};

/**************************************************
**************************************************/
class MESSAGE_IN{
private:
	enum{
		MAX_PARAMS = 10,
	};
	
	float Param[MAX_PARAMS];
	
public:
	enum{
		GET_STATUS,
		START_CALIB,
		START_TRACKING,
		STOP_TRACKING,
		CLOSE_GOLEM,
		GET_SKELTON_DEFINITION,
		
		NUM_MESSAGE_TYPES,
	};
	
	void SetParam(int id, float val){
		if((0 <= id) && (id < MAX_PARAMS)) { Param[id] = val; }
	}
	void GetParam(int id){
		if((0 <= id) && (id < MAX_PARAMS))	{ return Param[id]; }
		else								{ return 0;}
	}
};

/**************************************************
**************************************************/
class ofApp : public ofBaseApp{
private:
	/****************************************
	****************************************/
	enum{
		WINDOW_WIDTH	= 200,
		WINDOW_HEIGHT	= 200,
	};
	enum{
		BUFSIZE_L = 10000,
		BUFSIZE_S = 512,
		
		NUM_SENSORS = 6,
	};
	
	enum STATE{
		STATE__WAIT_ALL_SENSOR_ON,
		STATE__WAIT_CALIB_COMMAND,
		STATE__CALIBINPROGRESS_1,
		STATE__CALIBINPROGRESS_2,
		STATE__TRACKING_NOOSC_OUT,
		STATE__TRACKING_OSC_OUT,
		
		STATE__TEST,
	};
	
	/****************************************
	****************************************/
	bool b_go;
	bool b_1st;
	
	ofTrueTypeFont font;
	
	FILE* fp_log;
	FILE* fp_Skelton_FrameData;
	FILE* fp_Skelton_Definition;
	
	int Next_FrameId_of_MotionData;
	
	OSC_TARGET Osc;
	STATE State;
	
	bool SensorStatus[NUM_SENSORS];
	float t_Enter_WaitAllSensorOn;
	float t_StartCalib_1;
	float t_StartCalib_2;
	
	float now;
	
	ofxUDPManager udpConnection;
	
	/****************************************
	****************************************/
	void AllSensor_Ready();
	void AllSensor_Booting();
	
	void SendOsc_SensorStatus();
	void SendOsc_FrameData();
	void SendOsc_SkeltonDefinition();
	
	void SendUdp_SkeltonDefinition();
	void SendUdp_FrameData();
	
	void ResTo_GetStatus();
	void ResTo_CloseGolem();
	void ResTo_StartCalib(ofxOscMessage& m_receive);
	void ResTo_StartTracking();
	void ResTo_StopTracking();
	void ResTo_GetSkeltonDefinition();
	
	void Process_CalibInProgress_1();
	void Process_CalibInProgress_2();
	void Process_TrackingOscOut();
	void Process_Test();
	
	bool checkIf_ContentsExist(char* ret, char* buf);
	void Align_StringOfData(string& s);
	
public:
	/****************************************
	****************************************/
	ofApp();
	~ofApp();
	void exit();

	void setup();
	void update();
	void draw();
	

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	
};
