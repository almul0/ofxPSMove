# ======================= ofxCMake Vers. 0.1 =============
#  PUT THIS FILE INTO YOUR OPENFRAMEWORKS PROJECT FOLDER

# ========================================================
# ===================== CMake Settings ===================
# ========================================================
cmake_minimum_required( VERSION 3.3 )
project( PSMoveExample )

# ========================================================
# ===================== User Settings ====================
# ========================================================
# ---------------------- App name  -----------------------
set( APP_NAME   PSMoveExample )

# ------------------------ OF Path -----------------------
# --- If outside the OF structure, set an absolute OF path
set( OF_DIRECTORY_BY_USER "/opt/openframeworks" )

# --------------------- Source Files ---------------------
set( ${APP_NAME}_SOURCE_FILES
        example/src/main.cpp
        example/src/ofApp.cpp)


# ------------------------ AddOns  -----------------------
set( OFX_ADDONS_ACTIVE
        #        ofxXmlSettings
        #        ofxOsc
        #        ofxGui
        #        ofxNetwork
        #        ofxVectorGraphics
        #        ofxThreadedImageLoader
        #        ofxAssimpModelLoader
        #        ofxKinect
                ofxOpenCv
        #        ofxSvg
        #        ofxEmscripten (untested)
                ofxPSMove
                )

# =========================================================================
# ============================== OpenFrameworks ===========================
# =========================================================================
include( ${OF_DIRECTORY_BY_USER}/addons/ofxCMake/modules/main.cmake )
# =========================================================================

