
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

#include "ofxPSMove.h"

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

#define VRES_WIDTH 1024
#define VRES_HEIGHT 768

namespace ofxPSMove {

    const unsigned char PSMOVE_LED_R = 0;
    const unsigned char PSMOVE_LED_G = 0;
    const unsigned char PSMOVE_LED_B = 255;



    Receiver::Receiver()
    {

//		bSetup = false;
//		    psmoveData.accelerometer.set(0);
//		    psmoveData.gyroscope.set(0);
//		    psmoveData.magnetometer.set(0);
    }
    Receiver::~Receiver() {
        for (int id=0; id<count; id++) {
            bSetup[id] = false;
            if(move[id]!=NULL) {
                psmove_set_leds(move[id], 0,0,0);
                psmove_update_leds(move[id]);
                psmove_disconnect(move[id]);
            }
        }
        psmove_fusion_free(fusion);
        psmove_tracker_free(tracker);
        ofLog(OF_LOG_VERBOSE,"PSMove Disconnected\n");
    }
    void Receiver::enable() {
        //ofAddListener(ofEvents().update, this, &Receiver::update);
    }

    void Receiver::disable() {
        //ofRemoveListener(ofEvents().update, this, &Receiver::update);
    }

    void Receiver::setup() {
        count = psmove_count_connected();
        if (count<=0) {
            ofLogWarning("ofxPSMoveReceiver") << "None of the PSMove device found!!!";
        }
        bSetup.resize(count);
        move.resize(count);
        psmoveData.resize(count);
        // ofLog(OF_LOG_VERBOSE,"Connected controllers: %d\n", psmoveData.id);

        psmove_tracker_settings_set_default(&settings);
        settings.color_mapping_max_age = 0;
        settings.exposure_mode = Exposure_LOW;
        settings.camera_mirror = PSMove_True;
        tracker = psmove_tracker_new_with_settings(&settings);
        fusion = psmove_fusion_new(tracker, 1., 1000.);

        if (!tracker) {
            ofLogError("ofxPSMoveReceiver") <<  "Could not init PSMoveTracker.\n";
            ofExit(1);
        }

        for (int id=0; id<count; id++) {
            move[id] = psmove_connect_by_id(id);
            if (move[id] == NULL) {
                ofLogError("ofxPSMoveReceiver") << "Could not connect to default Move controller.\n"
                        "Please connect one via USB or Bluetooth.";

                bSetup[id] = false;
            }
            else {
                int result;
                for (;;) {
                    ofLogNotice("ofxPSMoveReceiver") << ("Calibrating controller %d...", id);

                    result = psmove_tracker_enable(tracker, move[id]);

                    if (result == Tracker_CALIBRATED) {
                        enum PSMove_Bool auto_update_leds =
                                psmove_tracker_get_auto_update_leds(tracker,
                                                                    move[id]);
                        ofLogNotice("ofxPSMoveReceiver") << ("OK, auto_update_leds is %s\n",
                                (auto_update_leds == PSMove_True)?"enabled":"disabled");
                        break;
                    } else {
                        ofLogNotice("ofxPSMoveReceiver") << "ERROR - retrying\n";
                    }
                }

                bSetup[id] = true;
            }

            if(bSetup[id]) {
                psmoveData[id].intrinsics = cv::cvarrToMat((CvMat*) cvLoad(psmove_util_get_file_path("intrinsics.xml"), 0, 0, 0));

                char *serial = psmove_get_serial(move[id]);
                ofLogVerbose("ofxPSMoveReciver") <<"Serial: "<< serial;
                free(serial);

                ctype = psmove_connection_type(move[id]);
                switch (ctype) {
                    case Conn_USB:
                        ofLogNotice("ofxPSMoveReceiver") <<"Connected via USB";
                        break;
                    case Conn_Bluetooth:
                        ofLogNotice("ofxPSMoveReceiver") <<"Connected via Bluetooth.";
                        break;
                    case Conn_Unknown:
                        ofLogNotice("ofxPSMoveReceiver") <<"Unknown connection type.";
                        break;
                }

                enum PSMove_Bool auto_update_leds = psmove_tracker_get_auto_update_leds(tracker, move[id]);
                ofLogNotice("ofxPSMoveReceiver") << ("OK, auto_update_leds is %s\n",
                        (auto_update_leds == PSMove_True)?"enabled":"disabled");


                for (int i=0; i<10; i++) {
                    psmove_set_rumble(move[id], 255*(i%2));
                    usleep(10000*(i%10));
                }

                for (int i=250; i>=0; i-=5) {
                    psmove_set_rumble(move[id], 0);
                }

                /* Enable rate limiting for LED updates */
                //psmove_set_rate_limiting(move[id], PSMove_False);


                psmove_enable_orientation(move[id], PSMove_True);
                assert(psmove_has_orientation(move[id]));

                while (psmove_tracker_enable(tracker, move[id]) != Tracker_CALIBRATED);
                psmove_update_leds(move[id]);


                psmove_tracker_update_image(tracker);
                psmove_tracker_update(tracker, NULL);
                psmove_tracker_annotate(tracker);

                void *frame;
                frame = psmove_tracker_get_frame(tracker);
                if (frame) {
                    cvShowImage("live camera feed", frame);
                    cvWaitKey(1);
                }


                /*--------------------------------------------------------------------------------------*/
                /*----------------------------ALIGN POINTER TO CAMERA-----------------------------------*/
                /*--------------------------------------------------------------------------------------*/

                ofLogNotice("ofxPSMoveReceiver") << "Center the move and press the move button\n";
                int buttons;
                while(!(buttons & Btn_MOVE)) {
                    while (psmove_poll(move[id])){
                        psmove_update_leds(move[id]);
                    };
                    buttons = psmove_get_buttons(move[id]);
                }
                while(buttons & Btn_MOVE) {
                    while (psmove_poll(move[id])){
                        psmove_update_leds(move[id]);
                    };
                    buttons = psmove_get_buttons(move[id]);
                }
                psmove_reset_orientation(move[id]);

                psmove_update_leds(move[id]);


                /*--------------------------------------------------------------------------------------*/
                /*--------------------------------DETERMINE FRONT PANEL---------------------------------*/
                /*--------------------------------------------------------------------------------------*/


                float xi, yi, zi;
                ofLogNotice("ofxPSMoveReceiver") << "FRONT SCREEN: Point to upper left and press MOVE\n";
                buttons = 0;
                while(!(buttons & Btn_MOVE)) {
                    while (psmove_poll(move[id])){
                        psmove_update_leds(move[id]);
                    };
                    buttons = psmove_get_buttons(move[id]);
                }
                while(buttons & Btn_MOVE) {
                    while (psmove_poll(move[id])){
                        psmove_update_leds(move[id]);
                    };
                    buttons = psmove_get_buttons(move[id]);
                }

                getFrontIntersectionPoint(id, xi, yi, zi);
                psmoveData[id].p11 = ofVec3f(xi,yi,zi);

                psmove_update_leds(move[id]);

                printf("FRONT SCREEN: Point to bottom right and press MOVE\n");
                buttons = 0;
                while(!(buttons & Btn_MOVE)) {
                    while (psmove_poll(move[id])){
                        psmove_update_leds(move[id]);
                    };
                    buttons = psmove_get_buttons(move[id]);
                }
                while(buttons & Btn_MOVE) {
                    while (psmove_poll(move[id])){
                        psmove_update_leds(move[id]);
                    };
                    buttons = psmove_get_buttons(move[id]);
                }

                getFrontIntersectionPoint(id, xi, yi, zi);
                psmoveData[id].p12 = ofVec3f(xi,yi,zi);

                psmove_update_leds(move[id]);
                printf("Congratulations! You rocks! Very nice!\n");
                printf("P11:\t X: %.2f\t, Y: %.2f\t Z: %.2f\n", psmoveData[id].p11.x, psmoveData[id].p11.y, psmoveData[id].p11.z);
                printf("P12:\t X: %.2f\t, Y: %.2f\t Z: %.2f\n", psmoveData[id].p12.x, psmoveData[id].p12.y, psmoveData[id].p12.z);

                // These points are points of the right plane (P12) and the left plane (P11)



                /*--------------------------------------------------------------------------------------*/
                /*--------------------------------------------------------------------------------------*/
                /*--------------------------------------------------------------------------------------*/


            }
        }
    }

    void Receiver::getPositionAndOrientation(int move_id, float& xgl, float& ygl, float& zgl, glm::vec3& direction){
        // u,v - Coordinates of the controllers sphere and its radius
        float u,v,radius;
        psmove_tracker_get_position(tracker,move[move_id], &u, &v, &radius);

        // Distance between camera and blob (Position vector modulus)
        double distance = psmove_tracker_distance_from_radius(tracker, radius);

        // Convert double values to float
        psmoveData[move_id].intrinsics.convertTo(psmoveData[move_id].intrinsics,CV_32F);

        // xz, yz - X and Y divided by Z (normalized)
        float xz, yz;
        xz = (u - ((float*)psmoveData[move_id].intrinsics.data)[2])/((float*)psmoveData[move_id].intrinsics.data)[0];
        yz = (v - ((float*)psmoveData[move_id].intrinsics.data)[5])/((float*)psmoveData[move_id].intrinsics.data)[4];

        // ax, ay - X and Y axis deviation with respect to Z axis
        float  ax, ay;
        ax = atan(xz);
        ay = atan(yz);

        // x,y,z -  Converted coordinates from tracker to general reference XYZ
        float x, y, z;
        z = (float)distance * cos(ax) * cos(ay); //  0.519 para pasar distancia real a gl dividir para esto
        x = xz * z;//  2.45;
        y = yz * z;//  1.3;

//     printf("U: %f\t V: %f\t R: %f\n", u ,v ,distance);
//     printf("X: %f\t Y: %f\t Z: %f\n", x ,y ,z);
//     printf("ax: %f\t ay: %f\Nos ha llegado información de un uso que en algún caso se ha denegado dicha solicitud n", ax ,ay);

        // Coordinates in GL draw space
        xgl = (float)(-x/2-0.25); // Fixed temp correction factor (FTCF)
        ygl = (float)(y/2+0.25); // FTCF
        zgl = -z;

//     printf("GLSPACE: X: %f\t Y: %f\t Z: %f\n", xgl ,ygl ,zgl);


        float wq, xq, yq, zq;
        psmove_get_orientation(move[move_id], &wq, &xq, &yq, &zq);
        glm::quat q(wq, xq, yq, zq);

        // Extract the vector part of the quaternion
        glm::vec3 uq(q.x, q.y, q.z);

        // Extract the scalar part of the quaternion
        float s = q.w;

        glm::vec3 vo(0., 0., 1.);

        // Do the math
        direction = 2.0f * dot(uq, vo) * uq
                    + (s*s - dot(uq, uq)) * vo
                    + 2.0f * s * cross(uq, vo);
    }

    bool Receiver::getFrontIntersectionPoint(int move_id, float& xi, float& yi, float& zi){
        float xgl, ygl, zgl;
        glm::vec3 direction;
        getPositionAndOrientation(move_id, xgl, ygl, zgl, direction);
        cv::Vec3f nf(0,0,-1);
        cv::Vec3f cf(0,0,0);
        cv::Vec3f x0(xgl,ygl,zgl);
        cv::Vec3f v0(direction.x, direction.y, direction.z);

        cv::Vec3f vecIntersection;
        float flFraction;

        //printf("GL0: X: %.2f\t Y: %.2f\t Z:%.2f\n", x0[0],x0[1],x0[2]);
        //printf("GL1: X: %.2f\t Y: %.2f\t Z:%.2f\n", x1[0],x1[1],x1[2]);

        bool intersect = linePlaneIntersection(nf,cf,x0,v0,vecIntersection, flFraction);
        // printf("INTERSECT: X: %.2f\t Y: %.2f\t Z:%.2f\n", vecIntersection[0],vecIntersection[1],vecIntersection[2]);
        // printf("INTERSECT: K: %.2f\n", flFraction);

        xi = vecIntersection[0];
        yi = vecIntersection[1];
        zi = vecIntersection[2];
        return intersect;
    }

    bool Receiver::getLeftIntersectionPoint(int move_id, float& xi, float& yi, float& zi, cv::Vec3f cl){
        float xgl, ygl, zgl;
        glm::vec3 direction;
        getPositionAndOrientation(move_id, xgl, ygl, zgl, direction);
        cv::Vec3f nl(1,0,0);
        cv::Vec3f x0(xgl,ygl,zgl);
        cv::Vec3f v0(direction.x, direction.y, direction.z);

        cv::Vec3f vecIntersection;
        float flFraction;

        //printf("GL0: X: %.2f\t Y: %.2f\t Z:%.2f\n", x0[0],x0[1],x0[2]);
        //printf("GL1: X: %.2f\t Y: %.2f\t Z:%.2f\n", x1[0],x1[1],x1[2]);

        bool intersect = linePlaneIntersection(nl,cl,x0,v0,vecIntersection, flFraction);
        // printf("INTERSECT: X: %.2f\t Y: %.2f\t Z:%.2f\n", vecIntersection[0],vecIntersection[1],vecIntersection[2]);
        // printf("INTERSECT: K: %.2f\n", flFraction);

        xi = vecIntersection[0];
        yi = vecIntersection[1];
        zi = vecIntersection[2];
        return intersect;
    }

    bool Receiver::getRightIntersectionPoint(int move_id, float& xi, float& yi, float& zi, cv::Vec3f cr){
        float xgl, ygl, zgl;
        glm::vec3 direction;
        getPositionAndOrientation(move_id, xgl, ygl, zgl, direction);
        cv::Vec3f nr(-1,0,0);
        cv::Vec3f x0(xgl,ygl,zgl);
        cv::Vec3f v0(direction.x, direction.y, direction.z);

        cv::Vec3f vecIntersection;
        float flFraction;

        //printf("GL0: X: %.2f\t Y: %.2f\t Z:%.2f\n", x0[0],x0[1],x0[2]);
        //printf("GL1: X: %.2f\t Y: %.2f\t Z:%.2f\n", x1[0],x1[1],x1[2]);

        bool intersect = linePlaneIntersection(nr,cr,x0,v0,vecIntersection, flFraction);
        // printf("INTERSECT: X: %.2f\t Y: %.2f\t Z:%.2f\n", vecIntersection[0],vecIntersection[1],vecIntersection[2]);
        // printf("INTERSECT: K: %.2f\n", flFraction);

        xi = vecIntersection[0];
        yi = vecIntersection[1];
        zi = vecIntersection[2];
        return intersect;
    }

    bool Receiver::linePlaneIntersection(const cv::Vec3f& n, const cv::Vec3f& c, const cv::Vec3f& x0, const cv::Vec3f& v, cv::Vec3f& vecIntersection, float& flFraction)
    {
        // n - plane normal
        // c - any point in the plane
        // x0 - the beginning of our line
        // x1 - the end of our line

        //cv::Vec3f v = x1 - x0;
        cv::Vec3f w = c - x0;

        float k = w.dot(n)/v.dot(n);

        vecIntersection = x0 + k * v;

        flFraction = k;

        return k >= 0;
    }

    //void Receiver::update(ofEventArgs & args)
    void Receiver::threadedFunction() {

        // start

        while(isThreadRunning()) {

            for (int id = 0; id < count; id++) {
                if (bSetup[id]) {
                    int btn = psmove_get_buttons(move[id]);
                    EventArgs args;
                    psmoveData[id].id = id;
                    args.data = &psmoveData[id];
                    if (psmove_poll(move[id])) {
//                        //    Btn_TRIANGLE = 1 << 0x04, /*!< Green triangle */
//                        //    Btn_CIRCLE = 1 << 0x05, /*!< Red circle */
//                        //    Btn_CROSS = 1 << 0x06, /*!< Blue cross */
//                        //    Btn_SQUARE = 1 << 0x07, /*!< Pink square */
//                        //
//                        //    Btn_SELECT = 1 << 0x08, /*!< Select button, left side */
//                        //    Btn_START = 1 << 0x0B, /*!< Start button, right side */
//                        //
//                        //    Btn_MOVE = 1 << 0x13, /*!< Move button, big front button */
//                        //    Btn_T = 1 << 0x14, /*!< Trigger, on the back */
//                        //    Btn_PS = 1 << 0x10, /*!< PS button, front center */
//
//                        //			args.person = person;
//                        //			args.scene = &scene;
//
//                        //			if (m.getAddress() == "/TSPS/personEntered/" || personIsNew){
//                        //				ofNotifyEvent(Events().personEntered, args, this);
//                        if ((psmove_get_buttons(move[id]) & Btn_TRIANGLE) && !psmoveData[id].BTN_TRIANGLE) {
//                            psmoveData[id].BTN_TRIANGLE = true;
//                            ofNotifyEvent(Events().buttonTrianglePressed, args, this);
//                        } else {
//                            psmoveData[id].BTN_TRIANGLE = false;
//                            ofNotifyEvent(Events().buttonTriangleReleased, args, this);
//                        }
//                        if (psmove_get_buttons(move[id]) & Btn_CIRCLE && !psmoveData[id].BTN_CIRCLE) {
//
//                            psmoveData[id].BTN_CIRCLE = true;
//                            ofNotifyEvent(Events().buttonCirclePressed, args, this);
//                        } else {
//                            psmoveData[id].BTN_CIRCLE = false;
//                            ofNotifyEvent(Events().buttonCircleReleased, args, this);
//                        }
//                        if (psmove_get_buttons(move[id]) & Btn_CROSS && !psmoveData[id].BTN_CROSS) {
//                            psmoveData[id].BTN_CROSS = true;
//                            ofNotifyEvent(Events().buttonCrossPressed, args, this);
//                        } else {
//                            psmoveData[id].BTN_CROSS = false;
//                            ofNotifyEvent(Events().buttonCrossReleased, args, this);
//                        }
//                        if (psmove_get_buttons(move[id]) & Btn_SQUARE) {
//                            psmoveData[id].BTN_TRIANGLE = true;
//                            ofNotifyEvent(Events().buttonSquarePressed, args, this);
//                        } else {
//                            psmoveData[id].BTN_TRIANGLE = false;
//                            ofNotifyEvent(Events().buttonSquareReleased, args, this);
//                        }
//                        if (psmove_get_buttons(move[id]) & Btn_SELECT) {
//                            psmoveData[id].BTN_TRIANGLE = true;
//                            ofNotifyEvent(Events().buttonSelectPressed, args, this);
//                        } else {
//                            psmoveData[id].BTN_TRIANGLE = false;
//                            ofNotifyEvent(Events().buttonSelectReleased, args, this);
//                        }
//                        if (psmove_get_buttons(move[id]) & Btn_START) {
//                            psmoveData[id].BTN_TRIANGLE = true;
//                            ofNotifyEvent(Events().buttonStartPressed, args, this);
//                        } else {
//                            psmoveData[id].BTN_TRIANGLE = false;
//                            ofNotifyEvent(Events().buttonStartReleased, args, this);
//                        }
//                        if (psmove_get_buttons(move[id]) & Btn_MOVE) {
//                            psmoveData[id].BTN_MOVE = true;
//                            ofNotifyEvent(Events().buttonMovePressed, args, this);
//                        } else {
//                            psmoveData[id].BTN_MOVE = false;
//                            ofNotifyEvent(Events().buttonMoveReleased, args, this);
//                        }
//                        if (psmove_get_buttons(move[id]) & Btn_T) {
//                            psmoveData[id].BTN_T = true;
//                            ofNotifyEvent(Events().buttonTPressed, args, this);
//                        } else {
//                            psmoveData[id].BTN_T = false;
//                            ofNotifyEvent(Events().buttonTReleased, args, this);
//                        }
//                        if (psmove_get_buttons(move[id]) & Btn_PS) {
//                            psmoveData[id].BTN_PS = true;
//                            ofNotifyEvent(Events().buttonPSPressed, args, this);
//                        } else {
//                            psmoveData[id].BTN_PS = false;
//                            ofNotifyEvent(Events().buttonPSReleased, args, this);
//                        }
//
//                        int _x, _y, _z;
//                        psmove_get_accelerometer(move[id], &_x, &_y, &_z);
//                        psmoveData[id].accelerometer.set(_x, _y, _z);
//
//                        psmove_get_gyroscope(move[id], &_x, &_y, &_z);
//                        psmoveData[id].gyroscope.set(_x, _y, _z);
//
//                        psmove_get_magnetometer(move[id], &_x, &_y, &_z);
//                        psmoveData[id].magnetometer.set(_x, _y, _z);
//
//                        uint trigger = psmove_get_trigger(move[id]);
//                        if (psmoveData[id].TRIGGER != trigger) {
//                            psmoveData[id].TRIGGER = trigger;
//                            ofNotifyEvent(Events().triggerUpdated, args, this);
//                        }
//
//                        int battery = psmove_get_battery(move[id]);
//
//                        if (psmoveData[id].battery != battery) {
//                            psmoveData[id].battery = battery;
//                            ofNotifyEvent(Events().batteryUpdated, args, this);
//                        }
//
//                        int temperature = psmove_get_temperature(move[id]);
//
//                        if (psmoveData[id].temperature != temperature) {
//                            psmoveData[id].temperature = psmove_get_temperature(move[id]);
//                            ofNotifyEvent(Events().temperatureUpdated, args, this);
//                        }
                    }

                    psmove_tracker_update_image(tracker);
                    psmove_tracker_update(tracker, NULL);
                    //psmove_tracker_annotate(tracker);





                    float w, x, y, z;
                    //psmove_tracker_get_position(tracker, move[id], &x, &y, &z);
                    getFrontIntersectionPoint(id, x, y, z);

                    //printf("INTERSECTIONX:\t %f\tINTERSECTIONY: %f\n",x,y);

//                    int xcv, ycv;
//                    xcv = (int)floor(abs(x - psmoveData[id].p11.x) * 640 / abs(psmoveData[id].p12.x-psmoveData[id].p11.x));
//                    ycv = (int)floor(abs(y - psmoveData[id].p11.y) * 480 / abs(psmoveData[id].p12.y-psmoveData[id].p11.y));
//
//
//                    void *frame;
//
//                    frame = psmove_tracker_get_frame(tracker);
//                    cvRectangle(frame, cvPoint(xcv-25, ycv-25), cvPoint(xcv+25, ycv+25), cvScalar(255, 255, 255, 0), CV_FILLED, 8, 0);
//                    if (frame) {
//                        cvShowImage("live camera feed", frame);
//                        cvWaitKey(1);
//                    }

                    int xv, yv;
                    xv = (int)floor(abs(x - psmoveData[id].p11.x) * VRES_WIDTH / abs(psmoveData[id].p12.x-psmoveData[id].p11.x));
                    yv = (int)floor(abs(y - psmoveData[id].p11.y) * VRES_HEIGHT / abs(psmoveData[id].p12.y-psmoveData[id].p11.y));



                    if (xv < 0) {
                        xv = 0;
                    } else if (xv > VRES_WIDTH) {
                        xv = VRES_WIDTH;
                    }

                    if (yv < 0) {
                        yv = 0;
                    } else if (yv > VRES_HEIGHT) {
                        yv = VRES_HEIGHT;
                    }

                    psmoveData[id].position.set(xv, yv, z);

                    // lock access to the resource
                    lock();
                    cursorx = xv;
                    cursory = yv;
                    unlock();


                    //printf("CURSORX:\t %d\tCURSORY: %d\n",xv,yv);

                    psmove_get_orientation(move[id], &w, &x, &y, &z);
                    psmoveData[id].orientation.set(x, y, z, w);



                    //ofNotifyEvent( Events().moved,args,this);



                    /*if (ctype != Conn_USB && !(psmove_get_buttons(move) & Btn_PS)) {
                     int res = psmove_poll(move);
                     if (res) {
                     if (psmove_get_buttons(move) & Btn_TRIANGLE) {
                     ofLog(OF_LOG_VERBOSE,"Triangle pressed, with trigger value: %d\n",
                     psmove_get_trigger(move));
                     psmove_set_rumble(move, psmove_get_trigger(move));
                     } else {
                     psmove_set_rumble(move, 0x00);
                     }

                     psmove_set_leds(move, sin(ofGetFrameNum()/30.0f)*255,cos(ofGetFrameNum()/60.0f)*255, psmove_get_trigger(move));

                     //int x, y, z;

                     ofLog(OF_LOG_VERBOSE,"buttons: %x\n", psmove_get_buttons(move));

                     battery = psmove_get_battery(move);

                     if (battery == Batt_CHARGING) {
                     ofLog(OF_LOG_VERBOSE,"battery charging\n");
                     } else if (battery == Batt_CHARGING_DONE) {
                     ofLog(OF_LOG_VERBOSE,"battery fully charged (on charger)\n");
                     } else if (battery >= Batt_MIN && battery <= Batt_MAX) {
                     ofLog(OF_LOG_VERBOSE,"battery level: %d / %d\n", battery, Batt_MAX);
                     } else {
                     ofLog(OF_LOG_VERBOSE,"battery level: unknown (%x)\n", battery);
                     }

                     ofLog(OF_LOG_VERBOSE,"temperature: %d\n", psmove_get_temperature(move));

                     psmove_update_leds(move);
                     }
                     }*/
                    //				ofNotifyEvent( Event().PSMoveEvent,psmoveData[id],this);
                }
            }
        }

    }

    void Receiver::draw()
    {

    }
    void Receiver::setLedColor(int _id, int r , int g , int b)
    {
        if(_id < count)
        {
            if(move[_id]!=NULL)
            {
                psmove_set_leds(move[_id], r, g, b);
                psmove_update_leds(move[_id]);
            }
        }
    }
}