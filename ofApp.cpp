/************************************************************
************************************************************/
#include "ofApp.h"

/************************************************************
************************************************************/

/******************************
******************************/
ofApp::ofApp()
: Osc("127.0.0.1", 12345, 12350)
, State(STATE__WAIT_ALL_SENSOR_ON)
, fp_log(NULL)
, fp_Skelton_FrameData(NULL)
, fp_Skelton_Definition(NULL)
, t_Enter_WaitAllSensorOn(0)
, t_StartCalib_1(0)
, t_StartCalib_2(0)
, Next_FrameId_of_MotionData(0)
, b_1st(true)
{
	font.load("RictyDiminished-Regular.ttf", 20, true, true, true);
	
	AllSensor_Booting();
}

/******************************
******************************/
ofApp::~ofApp()
{
}

/******************************
******************************/
void ofApp::exit()
{
	/********************
	********************/
	if(fp_log)					fclose(fp_log);
	if(fp_Skelton_FrameData)	fclose(fp_Skelton_FrameData);
	if(fp_Skelton_Definition)	fclose(fp_Skelton_Definition);
}

//--------------------------------------------------------------
void ofApp::setup(){
	/********************
	********************/
	ofSetBackgroundAuto(true);
	
	ofSetWindowTitle("study:ofBuffer");
	ofSetVerticalSync(false);
	ofSetFrameRate(50);
	ofSetWindowShape(WINDOW_WIDTH, WINDOW_HEIGHT);
	ofSetEscapeQuitsApp(false);
	
	ofEnableAlphaBlending();
	ofEnableBlendMode(OF_BLENDMODE_ALPHA);
	// ofEnableBlendMode(OF_BLENDMODE_ADD);
	// ofEnableSmoothing();
	
	/********************
	********************/
	udpConnection.Create();
	udpConnection.Connect("127.0.0.1",12351);
	udpConnection.SetNonBlocking(true);
	
	/********************
	********************/
	fp_log					= fopen("../../../data/Log.txt", "w");
	fp_Skelton_FrameData	= fopen("../../../data/sample_motion_data.csv", "r");
	fp_Skelton_Definition	= fopen("../../../data/sample_skeleton_definition.csv", "r");
	
	if(!fp_log || !fp_Skelton_FrameData || !fp_Skelton_Definition) { ERROR_MSG(); std::exit(1); }
}

//--------------------------------------------------------------
void ofApp::update(){
	/********************
	起動前に溜まっていたmessageは一旦clear.
	********************/
	if(b_1st){
		b_1st = false;
		
		while(Osc.OscReceive.hasWaitingMessages()){
			// none.
		}

		return;
	}
	
	/********************
	********************/
	now = ofGetElapsedTimef();
	
	/********************
	OSC messageに呼応する処理、OSC responce
	********************/
	while(Osc.OscReceive.hasWaitingMessages()){
		ofxOscMessage m_receive;
		Osc.OscReceive.getNextMessage(&m_receive);
		
		if(m_receive.getAddress() == "/Message"){
			int MessageId = m_receive.getArgAsInt(0);
			switch(MessageId){
				case MESSAGE_IN::GET_STATUS:
					// printf("> got message = GET_STATUS\n"); // too often to printf.
					ResTo_GetStatus();
					break;
					
				case MESSAGE_IN::START_CALIB:
					printf("> got message = START_CALIB\n");
					ResTo_StartCalib(m_receive);
					break;
					
				case MESSAGE_IN::START_TRACKING:
					printf("> got message = START_TRACKING\n");
					ResTo_StartTracking();
					break;
					
				case MESSAGE_IN::STOP_TRACKING:
					printf("> got message = STOP_TRACKING\n");
					ResTo_StopTracking();
					break;
					
				case MESSAGE_IN::CLOSE_GOLEM:
					printf("> got message = CLOSE_GOLEM\n");
					ResTo_CloseGolem();
					break;
					
				case MESSAGE_IN::GET_SKELTON_DEFINITION:
					printf("> got message = GET_SKELTON_DEFINITION\n");
					ResTo_GetSkeltonDefinition();
					break;
			}
		}
	}
	
	/********************
	状態で決まる処理、OSC send to oF in mac(note that this app is dummy of Golem).
	********************/
	switch(State){
		case STATE__WAIT_ALL_SENSOR_ON:
			// none.
			break;
			
		case STATE__WAIT_CALIB_COMMAND:
			// none.
			break;
			
		case STATE__CALIBINPROGRESS_1:
			Process_CalibInProgress_1();
			break;
			
		case STATE__CALIBINPROGRESS_2:
			Process_CalibInProgress_2();
			break;
			
		case STATE__TRACKING_NOOSC_OUT:
			// none.
			break;
			
		case STATE__TRACKING_OSC_OUT:
			Process_TrackingOscOut();
			break;
			
		case STATE__TEST:
			Process_Test();
			break;
	}
}

/******************************
******************************/
void ofApp::AllSensor_Ready()
{
	for(int i = 0; i < NUM_SENSORS; i++)	SensorStatus[i] = true;
}

/******************************
******************************/
void ofApp::AllSensor_Booting()
{
	for(int i = 0; i < NUM_SENSORS; i++)	SensorStatus[i] = false;
}

/******************************
******************************/
void ofApp::ResTo_GetStatus()
{
	SendOsc_SensorStatus();
}

/******************************
******************************/
void ofApp::SendOsc_SensorStatus()
{
	/********************
	********************/
	if(2.0 < now - t_Enter_WaitAllSensorOn){
		AllSensor_Ready();
		
		if(State == STATE__WAIT_ALL_SENSOR_ON){
			State = STATE__WAIT_CALIB_COMMAND;
			printf("> State->STATE__WAIT_CALIB_COMMAND\n");
		}
	}
	
	/********************
	********************/
	ofxOscMessage m;
	m.setAddress("/Golem/Status");
	
	for(int i = 0; i < NUM_SENSORS; i++){
		m.addIntArg(SensorStatus[i]);
	}
	
	Osc.OscSend.sendMessage(m);
}

/******************************
******************************/
void ofApp::ResTo_CloseGolem()
{
	// CloseAllSensorAndSystem();
	
	State = STATE__WAIT_ALL_SENSOR_ON;
	AllSensor_Booting();
	t_Enter_WaitAllSensorOn = now;
	
	printf("> State->STATE__WAIT_ALL_SENSOR_ON\n");
}

/******************************
******************************/
void ofApp::ResTo_StartCalib(ofxOscMessage& m_receive)
{
	if( (State == STATE__WAIT_CALIB_COMMAND) || (State == STATE__TRACKING_NOOSC_OUT) || (State == STATE__TRACKING_OSC_OUT) ){
		float UserHeight = m_receive.getArgAsFloat(1);
		// cal_SkeltonDefinition(UserHeight);
		
		/********************
		********************/
		ofxOscMessage m;
		m.setAddress("/Golem/Message");
		m.addIntArg(MESSAGE_OUT::CALIB_STARTED);
		
		Osc.OscSend.sendMessage(m);
		
		/********************
		********************/
		State = STATE__CALIBINPROGRESS_1;
		t_StartCalib_1 = now;
		printf("> State->STATE__CALIBINPROGRESS_1\n");
	}
}

/******************************
******************************/
void ofApp::ResTo_StartTracking()
{
	if( State == STATE__TRACKING_NOOSC_OUT ){
		State = STATE__TRACKING_OSC_OUT;
		printf("> State->STATE__TRACKING_OSC_OUT\n");
	}
}

/******************************
******************************/
void ofApp::ResTo_StopTracking()
{
	if( State == STATE__TRACKING_OSC_OUT ){
		/********************
		********************/
		ofxOscMessage m;
		m.setAddress("/Golem/Message");
		m.addIntArg(MESSAGE_OUT::TRACKING_STOPPED);
		
		Osc.OscSend.sendMessage(m);
		
		/********************
		********************/
		State = STATE__TRACKING_NOOSC_OUT;
		printf("> State->STATE__TRACKING_NOOSC_OUT\n");
	}
}

/******************************
******************************/
void ofApp::ResTo_GetSkeltonDefinition()
{
	SendUdp_SkeltonDefinition();
}

/******************************
******************************/
void ofApp::SendUdp_SkeltonDefinition()
{
	/********************
	********************/
	string message="";
	
	message += "/Golem/SkeletonDefinition";
	message += "<p>";
	
	/********************
	********************/
	char buf[BUFSIZE_L];
	
	fseek(fp_Skelton_Definition, 0, SEEK_SET);
	while(fgets(buf, BUFSIZE_L, fp_Skelton_Definition) != NULL){
		string str_Line = buf;
		Align_StringOfData(str_Line);
		
		vector<string> str_vals = ofSplitString(str_Line, ",");
		message += str_vals[0];	message += "|";		// BoneId
		message += str_vals[1];	message += "|";		// ParentId
		message += str_vals[2];	message += "|";		// BoneName
		message += str_vals[3];	message += "|";		// pos x
		message += str_vals[4];	message += "|";		// pos y
		message += str_vals[5];	message += "|";		// pos z
		message += str_vals[6];	message += "|";		// Quaternion x
		message += str_vals[7];	message += "|";		// Quaternion y
		message += str_vals[8];	message += "|";		// Quaternion z
		message += str_vals[9];	message += "<p>";	// Quaternion w
	}
	
	udpConnection.Send(message.c_str(),message.length());
	
	/********************
	********************/
	printf("> send skelton definition : Udp\n");
}

/******************************
******************************/
void ofApp::SendOsc_SkeltonDefinition()
{
	/********************
	********************/
	char buf[BUFSIZE_L];
	ofxOscMessage m;
	
	m.setAddress("/Golem/SkeletonDefinition");
	
	fseek(fp_Skelton_Definition, 0, SEEK_SET);
	while(fgets(buf, BUFSIZE_L, fp_Skelton_Definition) != NULL){
		string str_Line = buf;
		Align_StringOfData(str_Line);
		
		vector<string> str_vals = ofSplitString(str_Line, ",");
		m.addIntArg(atoi(str_vals[0].c_str()));		// BoneId
		m.addIntArg(atoi(str_vals[1].c_str()));		// ParentId
		m.addStringArg(str_vals[2]);				// BoneName
		m.addFloatArg(atof(str_vals[3].c_str()));	// pos x
		m.addFloatArg(atof(str_vals[4].c_str()));	// pos y
		m.addFloatArg(atof(str_vals[5].c_str()));	// pos z
		m.addFloatArg(atof(str_vals[6].c_str()));	// Quaternion x
		m.addFloatArg(atof(str_vals[7].c_str()));	// Quaternion y
		m.addFloatArg(atof(str_vals[8].c_str()));	// Quaternion z
		m.addFloatArg(atof(str_vals[9].c_str()));	// Quaternion w
	}
	
	Osc.OscSend.sendMessage(m);
	
	/********************
	********************/
	printf("> send skelton definition : Osc\n");
}

/******************************
******************************/
void ofApp::Align_StringOfData(string& s)
{
	size_t pos;
	while((pos = s.find_first_of(" 　\t\n\r")) != string::npos){ // 半角・全角space, \t 改行 削除
		s.erase(pos, 1);
	}
}

/******************************
******************************/
void ofApp::Process_CalibInProgress_1()
{
	/********************
	********************/
	if(3.0 < now - t_StartCalib_1){
		State = STATE__CALIBINPROGRESS_2;
		t_StartCalib_2 = now;
		printf("> State->STATE__CALIBINPROGRESS_2\n");
		
		/********************
		********************/
		ofxOscMessage m;
		m.setAddress("/Golem/Message");
		m.addIntArg(MESSAGE_OUT::READY_TO_STEP_FORWARD);
		
		Osc.OscSend.sendMessage(m);
	}
}

/******************************
******************************/
void ofApp::Process_CalibInProgress_2()
{
	/********************
	********************/
	if(3.0 < now - t_StartCalib_2){
		State = STATE__TRACKING_NOOSC_OUT;
		printf("> State->STATE__TRACKING_NOOSC_OUT\n");
		
		// StartTracking_inside();
	
		/********************
		********************/
		ofxOscMessage m;
		m.setAddress("/Golem/Message");
		m.addIntArg(MESSAGE_OUT::CALIB_SUCCEEDED);
		
		Osc.OscSend.sendMessage(m);
	}
}

/******************************
******************************/
void ofApp::Process_Test()
{
	/********************
	********************/
	ofxOscMessage m;
	m.setAddress("/Test");
	m.addFloatArg(mouseX);
	m.addFloatArg(mouseY);
	
	Osc.OscSend.sendMessage(m);
}

/******************************
******************************/
void ofApp::Process_TrackingOscOut()
{
	SendUdp_FrameData();
	
	SendOsc_SensorStatus();
}

/******************************
******************************/
void ofApp::SendUdp_FrameData()
{
	/********************
	********************/
	string message="";
	
	message += "/Golem/SkeltonData";
	message += "<p>";
	
	/********************
	********************/
	char* ret;
	char buf[BUFSIZE_L];
	
	ret = fgets(buf, BUFSIZE_L, fp_Skelton_FrameData);
	Next_FrameId_of_MotionData++;
	if(!checkIf_ContentsExist(ret, buf)){
		fseek(fp_Skelton_FrameData, 0, SEEK_SET);
		printf("> fp FrameData to Top\n");
		
		ret = fgets(buf, BUFSIZE_L, fp_Skelton_FrameData);
		if(!checkIf_ContentsExist(ret, buf)){
			printf("No Contents in fp FrameData\n");
			std::exit(1);
		}
		
		Next_FrameId_of_MotionData = 1;
	}
	
	/********************
	********************/
	string str_Line = buf;
	Align_StringOfData(str_Line);
	
	vector<string> str_vals = ofSplitString(str_Line, ",");
	for(int i=0; i < str_vals.size(); i++){
		message += str_vals[i];	message += "|";
	}
	message += ofToString(Next_FrameId_of_MotionData - 1);
	
	udpConnection.Send(message.c_str(),message.length());
}

/******************************
******************************/
void ofApp::SendOsc_FrameData()
{
	/********************
	********************/
	char* ret;
	char buf[BUFSIZE_L];
	
	ofxOscMessage m;
	m.setAddress("/Golem/SkeltonData");
	
	ret = fgets(buf, BUFSIZE_L, fp_Skelton_FrameData);
	Next_FrameId_of_MotionData++;
	if(!checkIf_ContentsExist(ret, buf)){
		fseek(fp_Skelton_FrameData, 0, SEEK_SET);
		printf("> fp FrameData to Top\n");
		
		ret = fgets(buf, BUFSIZE_L, fp_Skelton_FrameData);
		if(!checkIf_ContentsExist(ret, buf)){
			printf("No Contents in fp FrameData\n");
			std::exit(1);
		}
		
		Next_FrameId_of_MotionData = 1;
	}
	
	/********************
	********************/
	string str_Line = buf;
	Align_StringOfData(str_Line);
	
	vector<string> str_vals = ofSplitString(str_Line, ",");
	for(int i=0; i < str_vals.size(); i++){
		m.addFloatArg(atof(str_vals[i].c_str()));
	}
	
	m.addIntArg(Next_FrameId_of_MotionData - 1);
	
	static int frame = 0;
	if(frame % 2 == 0){
		Osc.OscSend.sendMessage(m);
		frame = 0;
	}
	frame++;
}

/******************************
******************************/
bool ofApp::checkIf_ContentsExist(char* ret, char* buf)
{
	if( (ret == NULL) || (buf == NULL)) return false;
	
	string str_Line = buf;
	Align_StringOfData(str_Line);
	vector<string> str_vals = ofSplitString(str_Line, ",");
	if( (str_vals.size() == 0) || (str_vals[0] == "") ){ // no_data or exist text but it's",,,,,,,".
		return false;
	}else{
		return true;
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(30);
	ofSetColor(255);
	
	char buf[BUFSIZE_S];
	sprintf(buf, "%05d:%6.2f", Next_FrameId_of_MotionData - 1, ofGetFrameRate());
	
	font.drawStringAsShapes(buf, 10, 30);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch(key){
		case 's':
			SendUdp_SkeltonDefinition();
			break;
			
		case 'c':
			ResTo_CloseGolem();
			break;
			
		case 'g':
			State = STATE__TRACKING_NOOSC_OUT;
			printf("> State->STATE__TRACKING_NOOSC_OUT\n");
			break;
			
		case 't':
			State = STATE__TEST;
			printf("> State->STATE__TEST\n");
			break;
			
		case ' ':
			State = STATE__TRACKING_OSC_OUT;
			printf("> State->STATE__TRACKING_OSC_OUT\n");
			break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
