idf_component_register(SRCS "http_server.c" "wifi_app.c" "main.c" "app_nvs.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "dist/assets/burros_blancos.png"
                                "dist/assets/index.css"
                                "dist/assets/index.js"
                                "dist/index.html")