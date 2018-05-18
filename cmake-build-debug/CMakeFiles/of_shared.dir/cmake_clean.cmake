file(REMOVE_RECURSE
  "../bin/PSMoveExample.app/Contents/Frameworks/libopenFrameworksShared.pdb"
  "../bin/PSMoveExample.app/Contents/Frameworks/libopenFrameworksShared.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/of_shared.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
