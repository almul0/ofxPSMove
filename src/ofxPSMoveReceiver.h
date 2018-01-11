/**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2011 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/


#ifndef __emptyExample__ofxPSMove__
#define __emptyExample__ofxPSMove__

#include <iostream>
#include "psmove.h"
#include "psmove_tracker.h"
#include "psmove_fusion.h"
#include "ofMain.h"
#include "ofThread.h"
#include "ofxOpenCv.h"
#include "ofxPSMoveEvent.h"
#include "ofxPSMoveData.h"
namespace ofxPSMove {
	class Receiver : public ofThread
	{
	public:
		Receiver();
		~Receiver();
		int cursorx, cursory;
		void setup();
		void enable();
		void disable();
		void threadedFunction();
		void update(ofEventArgs & args);
		void draw();
		void setLedColor(int _id , int r , int g , int b);
		
		
	private:
		vector<bool>bSetup;
		vector<PSMove *>move;
		int count;
		enum PSMove_Connection_Type ctype;
		vector<ofxPSMove::Data> psmoveData;
		PSMoveTrackerSettings settings;
		PSMoveTracker* tracker;
        	PSMoveFusion *fusion;
        bool linePlaneIntersection(const cv::Vec3f& n, const cv::Vec3f& c, const cv::Vec3f& x0, const cv::Vec3f& v, cv::Vec3f& vecIntersection, float& flFraction);
        bool getFrontIntersectionPoint(int move_id, float& xi, float& yi, float& zi );


	};
}
#endif /* defined(__emptyExample__ofxPSMove__) */
