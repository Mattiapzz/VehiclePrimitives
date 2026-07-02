# Fetch GoogleTest directly from GitHub
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
# Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Find all test source files in this directory
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "test_vprim_*.cc")

# Automatically generate a test executable for each file found
foreach(TEST_FILE ${TEST_SOURCES})
    # Extract just the filename without the extension (e.g., "test_vprim_CubicSpiral")
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    
    add_executable(${TEST_NAME} ${TEST_FILE})
    
    # Link against vprim and GoogleTest's main function
    target_link_libraries(${TEST_NAME} PRIVATE vprim GTest::gtest_main)
    
    # Register the test so 'ctest' can run it
    gtest_discover_tests(${TEST_NAME})
endforeach()