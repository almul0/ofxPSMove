file(REMOVE_RECURSE
  "../bin/ofxPSMoveDev.app/Contents/Frameworks/libopenFrameworksShared.pdb"
  "../bin/ofxPSMoveDev.app/Contents/Frameworks/libopenFrameworksShared.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/of_shared.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
