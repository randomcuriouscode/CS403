file(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/compsci403_assignment1/srv"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/_catkin_empty_exported_target.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
