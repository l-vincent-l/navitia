
add_executable(ptref_test ptret_test.cpp)
target_link_libraries(ptref_test ed ptreferential data fare routing types pb_lib
    georef utils autocomplete ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY}
    ${Boost_SERIALIZATION_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY} log4cplus protobuf)

ADD_BOOST_TEST(ptref_test)

add_executable(ptref_odt_level_test ptref_odt_level_test.cpp)
target_link_libraries(ptref_odt_level_test ed ptreferential data fare routing types pb_lib
    georef utils autocomplete ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY}
    ${Boost_SERIALIZATION_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY} log4cplus protobuf)

ADD_BOOST_TEST(ptref_odt_level_test)
