# Edit following two lines to set component requirements (see docs)
# set(COMPONENT_REQUIRES )
# set(COMPONENT_PRIV_REQUIRES )

# set(COMPONENT_SRCS "main.c")
# set(COMPONENT_ADD_INCLUDEDIRS "")

# register_component()
idf_component_register(SRCS "main.c"
                            "HX711.c"
                    INCLUDE_DIRS ".")