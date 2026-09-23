# NanoGFX as an ESP-IDF component (header-only). Add it with a path dependency, e.g. in an
# idf_component.yml:   nanogfx: { path: ../NanoGFX }   -- or ESPHome's add_idf_component(path=...).
# Without Arduino (plain ESP-IDF, ESPHome) PackCanvas builds on NanoGFXBase.h automatically.
idf_component_register(INCLUDE_DIRS "src")
