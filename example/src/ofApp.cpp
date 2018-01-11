#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	psMoveReceiver.setup();
	ofxAddPSMoveListeners(this);

    right.load("./assets/right.jpg");
    lantern.load("./assets/reveal.png");
}

//--------------------------------------------------------------
void ofApp::update(){

}
void ofApp::update(ofEventArgs & args) {
    psMoveReceiver.update(args);
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
    ofLogNotice() << printf("Triangle pressed");
}
void ofApp::onCirclePressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Circle pressed");
}
void ofApp::onCrossPressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Cross pressed");
}
void ofApp::onSquarePressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Square pressed");
}
void ofApp::onSelectPressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Select pressed");
}
void ofApp::onStartPressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Start pressed");
}
void ofApp::onMovePressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Move pressed");
}
void ofApp::onTPressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("T pressed");
}
void ofApp::onPSPressed( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("PS pressed");
}

void ofApp::onTriangleReleased( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Triangle released");
}
void ofApp::onCircleReleased( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Circle released");
}
void ofApp::onCrossReleased( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Cross released");
}
void ofApp::onSquareReleased( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Square released");
}
void ofApp::onSelectReleased( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Select released");
}
void ofApp::onStartReleased( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Trigger released");
}
void ofApp::onMoveReleased( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Move released");

}
void ofApp::onTReleased( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Trigger Released");

}
void ofApp::onPSReleased( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("PS Released");
}

void ofApp::onTriggerUpdated( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Trigger updated: %d", psmoveEvent.data->TRIGGER);
}
void ofApp::onBatteryUpdated( ofxPSMove::EventArgs & psmoveEvent )
{
    ofLogNotice() << printf("Battery updated: %d\n", psmoveEvent.data->battery);
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


    cursor.x = psmoveEvent.data->position.x;
    cursor.y = psmoveEvent.data->position.y;
}

void ofApp::exit() {

}