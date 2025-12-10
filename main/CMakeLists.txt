idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")
spiffs_create_partition_image(Datos "${CMAKE_CURRENT_SOURCE_DIR}/../InternalDisk" FLASH_IN_PROJECT)