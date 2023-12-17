if(CONFIG_TOOLBOX_ENABLED)
	set(srcs toolbox.c)
else()
	set(srcs toolbox-mock.c)
endif()
idf_component_register(SRCS "toolbox.c" "${srcs}"
                    INCLUDE_DIRS "include"
                    REQUIRES "driver" "esp_timer"
                    )

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

