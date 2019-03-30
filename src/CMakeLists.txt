ADD_EXECUTABLE(design_adc
                            main.cpp
                            tuv_ams_library.cpp
                            )
                    
target_link_libraries(design_adc
                    	${SYSTEMC_LIBRARIES}
                   	${SYSTEMCAMS_LIBRARIES}		
                     )
