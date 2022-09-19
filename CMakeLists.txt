set(srcs
    panelDriver.cpp
)

set(include_dirs
    .
)

set(requires
    esp_lcd
)

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})
