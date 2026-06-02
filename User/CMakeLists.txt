file(GLOB USER_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/*.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        ${USER_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
            ARM_MATH_CM4
        )