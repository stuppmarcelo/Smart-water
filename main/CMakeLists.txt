idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
)

# Embed certificate into binary
set(CERT_FILE "certs/fullchain.pem")
target_add_binary_data(${COMPONENT_TARGET} ${CERT_FILE} TEXT)
