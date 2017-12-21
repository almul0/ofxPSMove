file(REMOVE_RECURSE
  "../bin/ofxPSMoveDev.pdb"
  "../bin/ofxPSMoveDev"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ofxPSMoveDev.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
