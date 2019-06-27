add_library( linetri SHARED
    config.cpp
	features.cpp
    auxiliar.cpp
    frame.cpp
)

target_link_libraries( linetri
    ${THIRD_PARTY_LIBS}
)
