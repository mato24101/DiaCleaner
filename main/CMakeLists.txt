idf_component_register(SRCS "DiaCleaner.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES mcp23009 esp32-mcp23008-master driver freertos)

