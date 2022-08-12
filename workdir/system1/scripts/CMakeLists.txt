# Install the application

file(GLOB MY_XML_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.xml)

yarp_install(FILES ${MY_XML_FILES} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

#install(DIRECTORY "myDir/" 
#        DESTINATION "myDestination" 
#        FILES_MATCHING PATTERN "*.h" )


