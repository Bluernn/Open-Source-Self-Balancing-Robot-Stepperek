idf_component_register(
    SRCS "webServerClient.cpp"
    INCLUDE_DIRS .
    REQUIRES "freertos" "driver" "json" "esp_timer"
    EMBED_FILES "../../html/error.html"
    "../../html/favicon.ico"
    "../../html/main.js"
    "../../html/root.html"
    "../../html/main.css"
)
