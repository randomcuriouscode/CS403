file(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/compsci403_assignment1/srv"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/download_extra_data.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
