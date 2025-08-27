idf_component_register(
    SRCS "main.c" "src/BH1750.c"
    INCLUDE_DIRS "." "src"
    REQUIRES nvs_flash driver bt
)
