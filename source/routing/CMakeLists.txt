SET(BOOST_LIBS ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SERIALIZATION_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

SET(ROUTING_SRC routing.cpp raptor_solutions.cpp raptor_path.cpp raptor.cpp raptor_api.cpp next_stop_time.cpp dataraptor.cpp raptor_utils.cpp raptor_solution_filter.cpp)

add_library(routing ${ROUTING_SRC})

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark routing  boost_program_options data fare routing
    georef utils autocomplete ${BOOST_LIBS} log4cplus pb_lib protobuf)

add_library(routing_cli_utils routing_cli_utils.cpp)
add_executable(standalone single_run.cpp)
target_link_libraries(standalone routing_cli_utils routing pb_lib boost_program_options data fare routing thermometer time_tables pb_lib
    georef utils autocomplete ${BOOST_LIBS} log4cplus pthread protobuf)

add_executable(cli routing_cli.cpp)
target_link_libraries(cli linenoise routing_cli_utils routing pb_lib boost_program_options data fare routing thermometer time_tables pb_lib
    georef utils autocomplete ${BOOST_LIBS} log4cplus pthread protobuf)

add_subdirectory(tests)
