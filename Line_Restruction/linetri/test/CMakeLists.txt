add_executable( triangulation triangulation.cpp )
target_link_libraries( triangulation ${OpenCV_LIBS} )

add_executable(line_matching line_matching.cpp)
target_link_libraries( line_matching linetri)

add_executable(extract_data extract_data.cpp)
target_link_libraries( extract_data linetri)