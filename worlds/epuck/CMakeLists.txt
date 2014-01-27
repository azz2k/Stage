SET( epuckSrcs epuck.cc )
ADD_LIBRARY( epuck MODULE ${epuckSrcs} )
TARGET_LINK_LIBRARIES( epuck stage )
set_source_files_properties( ${epuckSrcs} PROPERTIES COMPILE_FLAGS "${FLTK_CFLAGS}" )
SET_TARGET_PROPERTIES( epuck PROPERTIES PREFIX "" )

INSTALL( TARGETS epuck DESTINATION ${PROJECT_PLUGIN_DIR})
