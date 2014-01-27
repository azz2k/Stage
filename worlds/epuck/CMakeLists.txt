SET( epuck_baseSrcs epuck_base.cc )
ADD_LIBRARY( epuck_base MODULE ${epuck_baseSrcs} )
TARGET_LINK_LIBRARIES( epuck_base stage )
set_source_files_properties( ${epuck_baseSrcs} PROPERTIES COMPILE_FLAGS "${FLTK_CFLAGS}" )
SET_TARGET_PROPERTIES( epuck_base PROPERTIES PREFIX "" )

INSTALL( TARGETS epuck_base DESTINATION ${PROJECT_PLUGIN_DIR})
