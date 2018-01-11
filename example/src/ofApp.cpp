#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	psMoveReceiver.setup();
	//ofxAddPSMoveListeners(this);
    psMoveReceiver.startThread();

    right.load("./assets/right.jpg");
    lantern.load("./assets/reveal.png");
}

//--------------------------------------------------------------
void ofApp::update(){

}
void ofApp::update(ofEventArgs & args) {
    //psMoveReceiver.update(args);
    if (psMoveReceiver.tryLock()) {
        cursor.x = psMoveReceiver.cursorx;
        cursor.y = psMoveReceiver.cursory;
        psMoveReceiver.unlock();
    }
    update();
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofEnableAlphaBlending();

    right.draw(0,0);
    lantern.draw(cursor.x-(lantern.getWidth()/2),cursor.y-(lantern.getHeight()/2));
    ofDisableAlphaBlending();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

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
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
void ofApp::onAccelerometerUpdated( ofxPSMove::EventArgs & psmoveEvent )
{
}
void ofApp::onGyroscopeUpdated ( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onMagnetometerUpdated( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onTrianglePressed( ofxPSMove::EventArgs & psmoveEvent )
{
	ofLogNotice() << "Triangle Pressed";
}
void ofApp::onCirclePressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << "Circle Pressed";
}
void ofApp::onCrossPressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << "Cross Pressed";
}
void ofApp::onSquarePressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << "Square Pressed";
}
void ofApp::onSelectPressed( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onStartPressed( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onMovePressed( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onTPressed( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onPSPressed( ofxPSMove::EventArgs & psmoveEvent )
{
	
}

void ofApp::onTriangleReleased( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onCircleReleased( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onCrossReleased( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onSquareReleased( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onSelectReleased( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onStartReleased( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onMoveReleased( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onTReleased( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onPSReleased( ofxPSMove::EventArgs & psmoveEvent )
{
	
}

void ofApp::onTriggerUpdated( ofxPSMove::EventArgs & psmoveEvent )
{
	
}
void ofApp::onBatteryUpdated( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Battery updated: %d", psmoveEvent.data->battery);
}
void ofApp::onTemperatureUpdated( ofxPSMove::EventArgs & psmoveEvent )
{
    //ofLogNotice() << printf("Temperature updated: %d", psmoveEvent.data->temperature) ;
}

void ofApp::onPSMoved( ofxPSMove::EventArgs & psmoveEvent )
{
    /*ofLogNotice() << printf("PS Moved: (%.2f,%.2f,%.2f)",psmoveEvent.data->position.x,
                            psmoveEvent.data->position.y,
                            psmoveEvent.data->position.z);*/


    //cursor.x = psmoveEvent.data->position.x;
    //cursor.y = psmoveEvent.data->position.y;
}

void ofApp::exit() {
    psMoveReceiver.stopThread();
}