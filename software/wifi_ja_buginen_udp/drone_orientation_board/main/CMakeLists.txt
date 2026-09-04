idf_component_register(SRCS 
    "main.c" 
    "utils/sanityChecks.c"
    "utils/networking.c"
    "utils/udp_client.c"
    "utils/drone_state.c"
                    INCLUDE_DIRS "."
                    "utils")
