set(SOURCE_FILE_CROSS_POINT_DETECT ${SOURCE_FILE_CROSS_POINT_DETECT} ${PROJECT_SOURCE_DIR}/src/cross_point_detect/cross_line_extractor.cc)
set(SOURCE_FILE_CROSS_POINT_DETECT ${SOURCE_FILE_CROSS_POINT_DETECT} ${PROJECT_SOURCE_DIR}/src/cross_point_detect/cross_point_feature_creator.cc)
set(SOURCE_FILE_CROSS_POINT_DETECT ${SOURCE_FILE_CROSS_POINT_DETECT} ${PROJECT_SOURCE_DIR}/src/cross_point_detect/cross_points_detector.cc)

set(SOURCE_FILE_BASIC_STRUCTURE ${SOURCE_FILE_BASIC_STRUCTURE} ${PROJECT_SOURCE_DIR}/src/basic_structure/cross_point_tree.cc)
set(SOURCE_FILE_BASIC_STRUCTURE ${SOURCE_FILE_BASIC_STRUCTURE} ${PROJECT_SOURCE_DIR}/src/basic_structure/cross_point.cc)
set(SOURCE_FILE_BASIC_STRUCTURE ${SOURCE_FILE_BASIC_STRUCTURE} ${PROJECT_SOURCE_DIR}/src/basic_structure/cross_pt_tuple.cc)
set(SOURCE_FILE_BASIC_STRUCTURE ${SOURCE_FILE_BASIC_STRUCTURE} ${PROJECT_SOURCE_DIR}/src/basic_structure/cross_ratio.cc)
set(SOURCE_FILE_BASIC_STRUCTURE ${SOURCE_FILE_BASIC_STRUCTURE} ${PROJECT_SOURCE_DIR}/src/basic_structure/search_tree.cc)


set(SOURCE_FILE_LOCATOR ${SOURCE_FILE_LOCATOR} ${PROJECT_SOURCE_DIR}/src/locator/estimator.cc)     
set(SOURCE_FILE_LOCATOR ${SOURCE_FILE_LOCATOR} ${PROJECT_SOURCE_DIR}/src/locator/locator.cc)
set(SOURCE_FILE_LOCATOR ${SOURCE_FILE_LOCATOR} ${PROJECT_SOURCE_DIR}/src/locator/matcher.cc)
set(SOURCE_FILE_LOCATOR ${SOURCE_FILE_LOCATOR} ${PROJECT_SOURCE_DIR}/src/locator/sampler.cc)
set(SOURCE_FILE_LOCATOR ${SOURCE_FILE_LOCATOR} ${PROJECT_SOURCE_DIR}/src/locator/similarity_evaluator.cc)

set(SOURCE_FILE_SHP ${SOURCE_FILE_SHP} ${PROJECT_SOURCE_DIR}/src/shp/draw_shp_elements.cc)
set(SOURCE_FILE_SHP ${SOURCE_FILE_SHP} ${PROJECT_SOURCE_DIR}/src/shp/get_shp_elements.cc)

set(SOURCE_FILE ${SOURCE_FILE} ${SOURCE_FILE_LOCATOR} ${SOURCE_FILE_BASIC_STRUCTURE} ${SOURCE_FILE_CROSS_POINT_DETECT})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# localization with single image
add_executable(rcl_geolocalize ${SOURCE_FILE} ${PROJECT_SOURCE_DIR}/src/exe/main.cc)
TARGET_LINK_LIBRARIES(rcl_geolocalize ${REQUIRED_LIBRARIES} )

# detect all intersections in the reference road map
add_executable(make_reference_map ${SOURCE_FILE_SHP} ${SOURCE_FILE_CROSS_POINT_DETECT}  ${SOURCE_FILE_BASIC_STRUCTURE} ${PROJECT_SOURCE_DIR}/src/exe/make_map.cc)
TARGET_LINK_LIBRARIES(make_reference_map ${REQUIRED_LIBRARIES} )


