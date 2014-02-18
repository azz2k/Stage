SET(epuckSrcs epuckAvoidance.cc)
set_source_files_properties( ${epuckSrcs} PROPERTIES COMPILE_FLAGS "${FLTK_CFLAGS}" )
ADD_LIBRARY( epuckAvoidance MODULE ${epuckSrcs} )
TARGET_LINK_LIBRARIES( epuckAvoidance stage )
SET_TARGET_PROPERTIES( epuckAvoidance PROPERTIES PREFIX "" )

INSTALL( TARGETS epuckAvoidance DESTINATION ${PROJECT_PLUGIN_DIR})
