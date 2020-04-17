set(srcs "am2301_humidity_sensor.c")	
set(include_dirs "./include")
set(requires nvs_flash)
set(priv_requires web_thing_server)

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES "${requires}"
                       PRIV_REQUIRES "${priv_requires}")
