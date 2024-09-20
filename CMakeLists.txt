set(COMPONENT_ADD_INCLUDEDIRS .)

set(COMPONENT_SRCS "fdc1004.cpp" "fdc1004.c")

idf_component_register(SRC_DIRS "."
    REQUIRES driver
    INCLUDE_DIRS ".")

