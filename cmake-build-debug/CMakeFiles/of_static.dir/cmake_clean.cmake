file(REMOVE_RECURSE
  "libopenFrameworksStatic.pdb"
  "libopenFrameworksStatic.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/of_static.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
